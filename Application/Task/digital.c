/**
 * @file    digital.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "digital.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"
#include "button/button.h"

//==============================================================================
// Private definitions
//==============================================================================

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

typedef struct
{
  GPIO_TypeDef *Gpio;
  uint32_t Pin;
} DigitaDrv_t;

typedef struct
{
  TIM_HandleTypeDef *TimerDrv;
  uint32_t TimerChannel;
} PwmDriver_t;

//==============================================================================
// Extern variables
//==============================================================================

extern TIM_HandleTypeDef htim2; // PWM IR channel 3, pino DIR do sensor de distancia 250 Hz

//==============================================================================
// Private function prototypes
//==============================================================================

static void Digital_Error( HAL_StatusTypeDef Err );
static void Digital_Init( void );

//==============================================================================
// Private variables
//==============================================================================

static PwmDriver_t gLineSensorIR = {0};
static Button_t gBtn = {0};

//==============================================================================
// Private functions
//==============================================================================

static void Digital_Error( HAL_StatusTypeDef Err )
{
  // Algum erro ocorreu, verificar
  for( ;; )
  {

  }
}

/**
 * @brief percent:  range: 0 to 100
 */
static void Digital_SetPwm( PwmDriver_t *Motor, uint8_t Percent )
{
  uint32_t compareValue;

  if( Percent <= 100 )
  {
    // Converte a porcentagem do duty cycle em um valor de contagem para o PWM.
    compareValue = ( ( uint32_t ) Percent * __HAL_TIM_GET_AUTORELOAD( Motor->TimerDrv ) ) / 100;
    __HAL_TIM_SET_COMPARE( Motor->TimerDrv, Motor->TimerChannel, compareValue+1 );
  }
}

static void Digital_Init(void)
{
  gBtn.Drv.Gpio = GPIOA;
  gBtn.Drv.Pin = GPIO_PIN_0;
  gBtn.OldStatus = true;
  gBtn.CurrStatus = true;
  gBtn.function_cb = NULL;

  gLineSensorIR.TimerDrv = &htim2;
  gLineSensorIR.TimerChannel = TIM_CHANNEL_3;

  // Configura PWM do pino IR
  HAL_TIM_PWM_Start( gLineSensorIR.TimerDrv, gLineSensorIR.TimerChannel );
  Digital_SetPwm( &gLineSensorIR, 50);
}

//==============================================================================
// Exported functions
//==============================================================================

void Digital_AttachBtn_Callback(void (*function_cb)(bool status))
{
  gBtn.function_cb = function_cb;
}

void Digital_Task( void *Parameters )
{
  Digital_Init();

  /* Infinite loop */
  for( ;; )
  {
    Button_Read( &gBtn );
    vTaskDelay( 50 );
  }
}
