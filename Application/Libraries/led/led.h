/**
 * @file    led.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _LED_H_
#define	_LED_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include "setup_hw.h"

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
//Exported constants
//==============================================================================

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

typedef enum
{
  eLED_BLINK_1 = 0,
  eLED_BLINK_2,
} LedBlinkType_e;

typedef struct
{
  GPIO_TypeDef *Gpio;
  uint16_t GpioPin;
} Led_t;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

void Led_Init( Led_t *Led,  GPIO_TypeDef *Gpio, uint16_t GpioPin );
void Led_Blink( Led_t *Led, LedBlinkType_e Type, uint8_t NumBlink );

//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* _LED_H_ */
