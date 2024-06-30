/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    bsp_driver_mmc.h (based on stm32l4r9i_eval_mmc.h)
 * @brief   This file contains the common defines and functions prototypes for
 *          the bsp_driver_mmc.c driver.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4_MMC_H
#define __STM32L4_MMC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "mmc_ops.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MMC Card information structure
 */
#define BSP_MMC_CardInfo HAL_MMC_CardInfoTypeDef

/* Exported constants --------------------------------------------------------*/
/**
 * @brief  MMC status structure definition
 */
#define MMMC_OK ((uint8_t)0x00)
#define MMMC_ERROR ((uint8_t)0x01)
#define MMMC_ERROR_MMC_NOT_PRESENT ((uint8_t)0x02)

/**
 * @brief  MMC transfer state definition
 */
#define MMC_TRANSFER_OK ((uint8_t)0x00)
#define MMC_TRANSFER_BUSY ((uint8_t)0x01)
#define MMC_TRANSFER_ERROR ((uint8_t)0x02)

#define MMC_DATATIMEOUT ((uint32_t)100000000)

#define MMC_PRESENT ((uint8_t)0x01)
#define MMC_NOT_PRESENT ((uint8_t)0x00)

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN BSP_H_CODE */
/* Exported functions --------------------------------------------------------*/
uint8_t BSP_MMC_Init(void);
uint8_t BSP_MMC_ITConfig(void);
void BSP_MMC_DetectIT(void);
void BSP_MMC_DetectCallback(void);
uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_MMC_GetCardState(void);
void BSP_MMC_GetCardInfo(BSP_MMC_CardInfo *CardInfo);
uint8_t BSP_MMC_IsDetected(void);
/* USER CODE END BSP_H_CODE */
#endif
  /* USER CODE BEGIN CallBacksSection_H */
  /* These __weak functions can be surcharged by application code in case the current settings
     (eg. interrupt priority, callbacks implementation) need to be changed for specific application needs */
  void BSP_MMC_AbortCallback(void);
  void BSP_MMC_WriteCpltCallback(void);
  void BSP_MMC_ReadCpltCallback(void);
/* USER CODE END CallBacksSection_H */
#ifdef __cplusplus
}
#endif

#endif /* __STM32L4_MMC_H */
