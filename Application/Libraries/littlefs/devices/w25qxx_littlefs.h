/**
* @file    w25qxx_littlefs.h
* @brief   Library to use littlefs with w25qxx memory
*/

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================

#ifndef _W25QXX_LITTLEFS_H
#define _W25QXX_LITTLEFS_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include "setup_hw.h"
#include "../lfs.h"
#include "lfs_file.h"
#include "w25qxx.h"

//==============================================================================
//Exported constants
//==============================================================================

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

int W25QXX_LittleFs_Init( LfsHandler_t *pLfsHandler, W25QXX_Handler_t *pDevice );

//==============================================================================
// Exported functions
//==============================================================================

#ifdef __cplusplus
}
#endif

#endif /* _W25QXX_LITTLEFS_H */
