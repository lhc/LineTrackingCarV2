/**
 * @file    printer.cpp
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================
#include "Task/analog.h"
#include "Setup/setup_hw.h"

#include <stdio.h>
#include <string.h>

//==============================================================================
// Private definitions
//==============================================================================

#define ANALOG_QUEUE_NUM_ITEMS           (1)
#define ANALOG_SEMAPHORE_TIMEOUT_MS      (1000)
#define ANALOG_QUEUE_READ_TIMEOUT_MS     (5000)
#define ANALOG_QUEUE_WRITE_TIMEOUT_MS    (5000)


//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

//==============================================================================
// Extern variables
//==============================================================================

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim8; // leitura de AD a cada 10Khz

//==============================================================================
// Private function prototypes
//==============================================================================

static void Analog_Init( void );
static void Analog_Start( void );
static void Analog_Stop( void );
static void Analog_Error( HAL_StatusTypeDef Err );
static void Analog_ISR( BaseType_t *pHigherPriorityTaskWoken );
static void Analog_ISR_ERR( BaseType_t *pHigherPriorityTaskWoken );

//==============================================================================
// Private variables
//==============================================================================

static xQueueHandle gQueueAnalog;
static SemaphoreHandle_t gSemaphoreADC;
static uint16_t gDmaBuffer[ ANALOG_SIZE_BUFFER  ];

//==============================================================================
// Private functions
//==============================================================================

static void Analog_Init( void )
{
  HAL_StatusTypeDef err;
  memset( gDmaBuffer, 0, sizeof( gDmaBuffer ) );

  gQueueAnalog = xQueueCreate( ANALOG_QUEUE_NUM_ITEMS, sizeof(AnalogValues_t) );
  if( gQueueAnalog == NULL )
  {
    Analog_Error( HAL_ERROR );
  }

  gSemaphoreADC = xSemaphoreCreateBinary( );
  if( gQueueAnalog == NULL )
  {
    Analog_Error( HAL_ERROR );
  }

  err = HAL_TIM_Base_Start( &htim8 );
   if( err != HAL_OK )
   {
     Analog_Error( err );
   }
}

static void Analog_Start( void )
{
  HAL_StatusTypeDef err;

  err = HAL_ADC_Start_DMA( &hadc1, ( uint32_t* ) gDmaBuffer, sizeof( gDmaBuffer ) / sizeof( gDmaBuffer[ 0 ] ) );
  if( err != HAL_OK )
  {
    Analog_Error( err );
  }
}

static void Analog_Stop( void )
{
  HAL_ADC_Stop_DMA( &hadc1 );
}

static void Analog_Error( HAL_StatusTypeDef Err )
{
  for( ;; )
  {

  }
}

static void Analog_ISR( BaseType_t *pHigherPriorityTaskWoken )
{
  if( ( NULL != pHigherPriorityTaskWoken ) && ( gSemaphoreADC != NULL ) )
  {
    Analog_Stop();
    xSemaphoreGiveFromISR( gSemaphoreADC, pHigherPriorityTaskWoken );
  }
}

static void Analog_ISR_ERR( BaseType_t *pHigherPriorityTaskWoken )
{
  if( ( pHigherPriorityTaskWoken != NULL ) && ( gSemaphoreADC != NULL ) )
  {
    Analog_Stop();
    xSemaphoreGiveFromISR( gSemaphoreADC, pHigherPriorityTaskWoken );
  }
}

//==============================================================================
// Exported functions
//==============================================================================

bool Analog_Read( AnalogValues_t *Analog )
{
  if( gQueueAnalog == NULL )
  {
    return false;
  }

  if( xQueueReceive( gQueueAnalog, Analog, ANALOG_QUEUE_READ_TIMEOUT_MS ) == pdTRUE )
  {
    return true;
  }

  return false;
}

void Analog_Task( void *Parameters )
{
  AnalogValues_t adcValues;

  Analog_Init();
  Analog_Start();

  /* Infinite loop */
  for( ;; )
  {
    if( xSemaphoreTake( gSemaphoreADC, ANALOG_SEMAPHORE_TIMEOUT_MS ) == pdTRUE )
    {
      memcpy( &adcValues.Values, gDmaBuffer, sizeof(gDmaBuffer) );

      // Adiciona valor calculado a queue, q agora pode ser consumida pela task control
      xQueueSend( gQueueAnalog, &adcValues, ANALOG_QUEUE_WRITE_TIMEOUT_MS );

      // Inicializa captura por DMA novamente
      Analog_Start();
    }
  }
}

// CALLBACKS DA HAL DA ST

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if( hadc->Instance == ADC1 )
  {
    Analog_ISR( &xHigherPriorityTaskWoken );
  }

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_ADC_ErrorCallback( ADC_HandleTypeDef *hadc )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if( hadc->Instance == ADC1 )
  {
    Analog_ISR_ERR( &xHigherPriorityTaskWoken );
  }

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
