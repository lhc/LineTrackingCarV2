/**
 * @file    control.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "control.h"
#include "digital.h"
#include "serial.h"
#include "analog.h"
#include "Setup/setup_hw.h"
#include "Setup/setup_database.h"
#include "bitwise/bitwise.h"
#include "led/led.h"
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

typedef enum
{
  eMOTOR_HALT = 0,
  eMOTOR_FORWARD,
  eMOTOR_BACKWARD,
} MotorDirection_e;

const char *ControlDirMsg[] =
{
    "Stop",
    "Front",
    "Back",
    "Right",
    "Left"
};

typedef struct
{
  TIM_HandleTypeDef *TimerDrv;
  uint32_t TimerChannel;
} PwmDriver_t;

typedef struct
{
  /* Gpio Driver control */
  GPIO_TypeDef *GpioP;
  uint16_t GpioPinP;
  GPIO_TypeDef *GpioN;
  uint16_t GpioPinN;

  /** Pwm driver control */
  PwmDriver_t PwmDrv;
  uint32_t CompareValue;
} MotorDriver_t;

typedef struct
{
  MotorDriver_t MotorA;
  MotorDriver_t MotorB;
} MotorCtrl_t;

typedef struct
{
  CarDirection_e Dir;
  uint8_t DutyCyclePercentA; /* range 0 to 100%  */
  uint8_t DutyCyclePercentB; /* range 0 to 100%  */
  PidCtrl_t Pid;
} CarCtrl_t;

typedef struct
{
  DigitalValues_t Digital;
  AnalogValues_t Analog;
} CarMeasures_t;

//==============================================================================
// Extern variables
//==============================================================================

extern TIM_HandleTypeDef htim4; // Pwm A channel 1, configurado para 1Khz
extern TIM_HandleTypeDef htim1; // Pwm B channel 1, configurado para 1Khz
extern TIM_HandleTypeDef htim2; // PWM IR channel 3, pino DIR do sensor de distancia 250 Hz

//==============================================================================
// Private function prototypes
//==============================================================================

static void Control_Init( MotorCtrl_t *Ctrl );
static void Control_Motor_SetDir( MotorDriver_t *Motor,  MotorDirection_e Dir );
static void Control_Motor_SetPwm( MotorDriver_t *Motor, uint8_t Percent );

static void Control_Car_SetDir( MotorCtrl_t *MotorCtrl, CarCtrl_t *CarCtrl );
static void Control_Car_GetStatusSensor( CarMeasures_t *Measures  );
static void Control_Car_CalcDirection( CarCtrl_t *Car, CarMeasures_t *Measures );
static void Control_Trace( CarCtrl_t *Car, CarMeasures_t *Measures );

static void Control_ButtonEvent_CB( bool IsPressed );

//==============================================================================
// Private variables
//==============================================================================

static Led_t gLed1 = {0};
static CarCtrl_t gCarCtrl = {0};
static eCarDebugType_e gIsTraceEnable = eCAR_TRACE_DISABLE;
static const char PidFileConfig[] = "/pid.conf";

//==============================================================================
// Private functions
//==============================================================================

#if 0

// Funcao usada para o teste dos motores
static void Control_Test_Raw(void)
{
  uint32_t dutyCycle1  = 0;
  uint32_t dutyCycle2  = 0;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5 , GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6 , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7 , GPIO_PIN_SET);

  for(;;)
  {
    if( dutyCycle2 < __HAL_TIM_GET_AUTORELOAD( &htim4 ) )
    {
      __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_1, ++dutyCycle2 );
    }
    else
    {
      dutyCycle2 = 0;
    }

    if( dutyCycle1 < __HAL_TIM_GET_AUTORELOAD( &htim1 ) )
    {
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, ++dutyCycle1 );
    }
    else
    {
      dutyCycle1 = 0;
    }

    vTaskDelay(10);
  }
}


// Funcao usada para a direcao dos motores
static void Control_Test_Direction(void)
{
  CarCtrl_t gCarCtrl;
  MotorCtrl_t MotorCtrl;

  DigitalValues_t adcValues;

  // Configura GPIO e PWM dos motores
  Control_Init( &MotorCtrl );

  gCarCtrl.DutyCyclePercentA = 50;
  gCarCtrl.DutyCyclePercentB = 50;

  for(;;)
  {
    gCarCtrl.Dir = eCAR_DIR_FRONT;
    Control_Car_SetDir( &MotorCtrl, &gCarCtrl );
    vTaskDelay(1000);

    gCarCtrl.Dir = eCAR_DIR_BACK;
    Control_Car_SetDir( &MotorCtrl, &gCarCtrl );
    vTaskDelay(1000);

    gCarCtrl.Dir = eCAR_DIR_RIGHT;
    Control_Car_SetDir( &MotorCtrl, &gCarCtrl );
    vTaskDelay(1000);

    gCarCtrl.Dir = eCAR_DIR_LEFT;
    Control_Car_SetDir( &MotorCtrl, &gCarCtrl );
    vTaskDelay(1000);
  }
}

#endif

static void Control_Init( MotorCtrl_t *Ctrl )
{
  int32_t result;

  // ================= Configura pinos que controlam os motores  =================
  Ctrl->MotorA.GpioP = GPIOD;
  Ctrl->MotorA.GpioPinP = GPIO_PIN_4;
  Ctrl->MotorA.GpioN = GPIOD;
  Ctrl->MotorA.GpioPinN = GPIO_PIN_5;
  Ctrl->MotorA.PwmDrv.TimerDrv = &htim4;
  Ctrl->MotorA.PwmDrv.TimerChannel = TIM_CHANNEL_1;

  Ctrl->MotorB.GpioP = GPIOD;
  Ctrl->MotorB.GpioPinP = GPIO_PIN_6;
  Ctrl->MotorB.GpioN = GPIOD;
  Ctrl->MotorB.GpioPinN = GPIO_PIN_7;
  Ctrl->MotorB.PwmDrv.TimerDrv = &htim1;
  Ctrl->MotorB.PwmDrv.TimerChannel = TIM_CHANNEL_1;

  __HAL_TIM_SET_COMPARE( Ctrl->MotorA.PwmDrv.TimerDrv, Ctrl->MotorA.PwmDrv.TimerChannel, 0 );
  HAL_TIM_PWM_Start( Ctrl->MotorA.PwmDrv.TimerDrv, Ctrl->MotorA.PwmDrv.TimerChannel );

  __HAL_TIM_SET_COMPARE( Ctrl->MotorB.PwmDrv.TimerDrv, Ctrl->MotorB.PwmDrv.TimerChannel, 0 );
  HAL_TIM_PWM_Start( Ctrl->MotorB.PwmDrv.TimerDrv, Ctrl->MotorB.PwmDrv.TimerChannel );

  // ================= Inicializa parametros gravados na flash externa  =================
  result = Database_Read( PidFileConfig, ( void* ) &gCarCtrl.Pid, sizeof(PidCtrl_t) );
  if( result != 0 )
  {
    result = Database_Write( PidFileConfig, ( void* ) &gCarCtrl.Pid, sizeof(PidCtrl_t) );
  }

  // ================= Set Button Callback =================
  Digital_AttachBtn_Callback( Control_ButtonEvent_CB );

  // ================= Configure led =================
  Led_Init( &gLed1, GPIOA, GPIO_PIN_1 );
}

static void Control_Motor_SetDir( MotorDriver_t *Motor,  MotorDirection_e Dir )
{
  switch( Dir )
  {
    case eMOTOR_HALT:
    {
      HAL_GPIO_WritePin( Motor->GpioP, Motor->GpioPinP, GPIO_PIN_RESET );
      HAL_GPIO_WritePin( Motor->GpioN, Motor->GpioPinN, GPIO_PIN_RESET );
      break;
    }
    case eMOTOR_FORWARD:
    {
      HAL_GPIO_WritePin(Motor->GpioP, Motor->GpioPinP, GPIO_PIN_SET );
      HAL_GPIO_WritePin(Motor->GpioN, Motor->GpioPinN, GPIO_PIN_RESET );
      break;
    }
    case eMOTOR_BACKWARD:
    {
      HAL_GPIO_WritePin( Motor->GpioP, Motor->GpioPinP, GPIO_PIN_RESET );
      HAL_GPIO_WritePin( Motor->GpioN, Motor->GpioPinN, GPIO_PIN_SET );
      break;
    }
    default:
      return;
  }
}

/**
 * @brief percent:  range: 0 to 100
 */
static void Control_Motor_SetPwm( MotorDriver_t *Motor, uint8_t Percent )
{
  if( Percent <= 100 )
  {
    // Converte a porcentagem do duty cycle em um valor de contagem para o PWM.
    Motor->CompareValue = ( ( uint32_t ) Percent * __HAL_TIM_GET_AUTORELOAD( Motor->PwmDrv.TimerDrv ) ) / 100;
    __HAL_TIM_SET_COMPARE( Motor->PwmDrv.TimerDrv, Motor->PwmDrv.TimerChannel, Motor->CompareValue );
  }
}

static void Control_Car_SetDir( MotorCtrl_t *MotorCtrl, CarCtrl_t *CarCtrl )
{
  MotorDirection_e dirMotorA;
  MotorDirection_e dirMotorB;

  switch( CarCtrl->Dir )
  {
    case eCAR_DIR_STOP:
    {
      dirMotorA = eMOTOR_HALT;
      dirMotorB = eMOTOR_HALT;
      break;
    }
    case eCAR_DIR_FRONT:
    {
      dirMotorA = eMOTOR_FORWARD;
      dirMotorB = eMOTOR_FORWARD;
      break;
    }
    case eCAR_DIR_BACK:
    {
      dirMotorA = eMOTOR_BACKWARD;
      dirMotorB = eMOTOR_BACKWARD;
      break;
    }
    case eCAR_DIR_RIGHT:
    {
      dirMotorA = eMOTOR_HALT;
      dirMotorB = eMOTOR_FORWARD;
      break;
    }
    case eCAR_DIR_LEFT:
    {
      dirMotorA = eMOTOR_FORWARD;
      dirMotorB = eMOTOR_HALT;
      break;
    }
    default:
      return;
  }

  // Motor A
  Control_Motor_SetDir( &MotorCtrl->MotorA, dirMotorA );
  Control_Motor_SetPwm( &MotorCtrl->MotorA, CarCtrl->DutyCyclePercentA );

  // Motor B
  Control_Motor_SetDir( &MotorCtrl->MotorB, dirMotorB );
  Control_Motor_SetPwm( &MotorCtrl->MotorB, CarCtrl->DutyCyclePercentB );
}

#warning "Reescrever toda esta rotina, esta um lixo"
static void Control_Car_GetStatusSensor( CarMeasures_t *Measures )
{
  bool status;
  uint16_t indexCnt;
  uint16_t numAquisitios;
  uint16_t numChannels;
  uint16_t minAdc[ANALOG_NUM_ADC_CHANNELS];
  uint16_t maxAdc[ANALOG_NUM_ADC_CHANNELS];
  uint16_t diff[ANALOG_NUM_ADC_CHANNELS];

  indexCnt = 0;

  // Init min value
  for( numChannels = 0; numChannels < ANALOG_NUM_ADC_CHANNELS; numChannels++ )
  {
    diff[numChannels] = 0;
    maxAdc[numChannels] = 0;
    minAdc[numChannels] = 4096;
  }

  for(numAquisitios = 0; numAquisitios < ANALOG_NUM_AQUISITIONS; numAquisitios++)
  {
    for( numChannels = 0; numChannels < ANALOG_NUM_ADC_CHANNELS; numChannels++ )
    {
      if( minAdc[ numChannels ] > Measures->Analog.Values[ indexCnt ] )
      {
        minAdc[ numChannels ] = Measures->Analog.Values[ indexCnt ];
      }
      if( maxAdc[ numChannels ] < Measures->Analog.Values[ indexCnt ] )
      {
        maxAdc[ numChannels ] = Measures->Analog.Values[ indexCnt ];
      }
      indexCnt++;
    }
  }

  for( numChannels = 0; numChannels < ANALOG_NUM_ADC_CHANNELS; numChannels++ )
  {
    diff[numChannels] = maxAdc[ numChannels ] - minAdc[ numChannels ];
    status = (diff[numChannels] > 130 ) ?  false : true;
    _BIT_WR_BOL( Measures->Digital.Values, numChannels, status );
  }
}

static void Control_Car_CalcDirection( CarCtrl_t *Car, CarMeasures_t *Measures )
{
  static uint16_t percent; // Variavel global que n perder o valor(static)

  if( ++percent > 100 )
  {
    percent = 0;
  }

  Car->DutyCyclePercentA = percent;
  Car->DutyCyclePercentB = percent;
}

__inline static void Control_Trace(CarCtrl_t *Car, CarMeasures_t *Measures )
{
  uint16_t numAquisitios;
  uint16_t numChannels;
  uint16_t indexCnt;

  if( gIsTraceEnable == eCAR_TRACE_DIGITAL)
  {
    Serial_Message( "S: 0b"_BYTE_TO_BINARY_PATTERN", Dir: %s, M1: %d %%, M2: %d %%\r\n", _BYTE_TO_BINARY(Measures->Digital.Values), ControlDirMsg[Car->Dir], Car->DutyCyclePercentA, Car->DutyCyclePercentB );
  }
  else if( gIsTraceEnable == eCAR_TRACE_ANALOG)
  {
    indexCnt = 0;

    for(numAquisitios = 0; numAquisitios < ANALOG_NUM_AQUISITIONS; numAquisitios++)
    {
      for(numChannels = 0; numChannels < ANALOG_NUM_ADC_CHANNELS; numChannels++)
      {
        Serial_Message( "%d,", Measures->Analog.Values[indexCnt++] );
      }
      Serial_Message( "0\r\n" );
    }
  }
}

static void Control_ButtonEvent_CB( bool IsPressed )
{
  static uint8_t state = 0;

  if( IsPressed )
  {
    switch( state )
    {
      case 0:
      {
        Led_Blink( &gLed1, eLED_BLINK_1, 1 );
        gCarCtrl.Dir = eCAR_DIR_STOP;
        state = 1;
        break;
      }
      case 1:
      {
        Led_Blink( &gLed1, eLED_BLINK_2, 2 );
        gCarCtrl.Dir = eCAR_DIR_FRONT;
        state = 0;
        break;
      }
      default:
        state = 0;
        break;
    }
  }
}

//==============================================================================
// Exported functions
//==============================================================================

void Control_EnableTrace( eCarDebugType_e Type )
{
  gIsTraceEnable = Type;
}

void Control_SetDirection( CarDirection_e Direction )
{
  if( Direction < eCAR_DIR_MAX )
  {
    gCarCtrl.Dir = Direction;
  }
}

CarDirection_e Control_GetDirection( void )
{
  return gCarCtrl.Dir;
}

const char* Control_GetDirectionString( CarDirection_e Direction )
{
  if( Direction < eCAR_DIR_MAX )
  {
    return ControlDirMsg[Direction];
  }

  return NULL;
}

void Control_SetParamPID( PidTypeParam_e Type, float Value )
{
  switch( Type )
  {
    case ePID_KP:
    {
      gCarCtrl.Pid.Kp = Value;
      break;
    }
    case ePID_KI:
    {
      gCarCtrl.Pid.Ki = Value;
      break;
    }
    case ePID_KD:
    {
      gCarCtrl.Pid.Kd = Value;
      break;
    }
    case ePID_SetPoint:
    {
      gCarCtrl.Pid.SetPoint = Value;
      break;
    }
    default:
      return;
      break;
  }

  // Salva parametro
  Database_Write( PidFileConfig, ( void* ) &gCarCtrl.Pid, sizeof(PidCtrl_t) );
}

float Control_GetParamPID( PidTypeParam_e Type )
{
  switch( Type )
  {
    case ePID_KP:
    {
      return gCarCtrl.Pid.Kp;
    }
    case ePID_KI:
    {
      return gCarCtrl.Pid.Ki;
    }
    case ePID_KD:
    {
      return gCarCtrl.Pid.Kd;
    }
    case ePID_SetPoint:
    {
      return gCarCtrl.Pid.SetPoint;
    }
    default:
      return 0;
  }
}

void Control_Task(void *Parameters)
{
  bool update;
  MotorCtrl_t motorCtrl;
  CarMeasures_t measures;

  vTaskDelay(3000);

  // Configura GPIO e PWM dos motores
  Control_Init( &motorCtrl );

  for( ;; )
  {
    update = Analog_Read( &measures.Analog );
    if( update )
    {
      Control_Car_GetStatusSensor( &measures );
      Control_Car_CalcDirection( &gCarCtrl, &measures );
      Control_Car_SetDir( &motorCtrl, &gCarCtrl );
      Control_Trace( &gCarCtrl, &measures );
    }
  }
}
