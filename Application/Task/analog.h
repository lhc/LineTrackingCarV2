/**
 * @file    buzzer.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _TASK_BUZZER_H
#define _TASK_BUZZER_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include <stdint.h>
#include <stdbool.h>

//==============================================================================
//Exported constants
//==============================================================================

#define ANALOG_NUM_ADC_CHANNELS          (8)
#define ANALOG_NUM_AQUISITIONS           (20)
#define ANALOG_SIZE_BUFFER               (ANALOG_NUM_ADC_CHANNELS*ANALOG_NUM_AQUISITIONS)

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

typedef struct
{
  uint16_t Values[ ANALOG_SIZE_BUFFER ];
} AnalogValues_t;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

bool Analog_Read( AnalogValues_t *Analog );
void Analog_Task(void *Parameters);

//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif  /* _TASK_BUZZER_H */