/**
 * @file    setup_hw.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _SETUP_HW_
#define	_SETUP_HW_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include "main.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"

#include <stdint.h>
#include <stdbool.h>

//==============================================================================
//Exported constants
//==============================================================================

/* Littlefs Debug */
#define LFS_NO_DEBUG            /**< Disable messages */
#define LFS_NO_WARN             /**< Disable warn messages */
#define LFS_NO_ERROR            /**< Disable error messages */
#define LFS_NO_ASSERT           /**< Disable asserts of lib */
//#define LFS_NO_MALLOC         /**< No use malloc and free function */
//#define LFS_YES_TRACE         /**< Enable messages of trace */
//#define LFS_NO_INTRINSICS     /**< ?? */
//#define LFS_READONLY          /**< Configuring library just to use read mode */
//#define LFS_THREADSAFE        /**< Enable functions to lock and unlock when entry in some routine of read and write */

#define SETUP_FIRMWARE_VERSION   "v1.0"

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

void Setup_Init( void );

//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* _SETUP_HW_ */
