/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    bsp_driver_mmc.c for L4 (based on stm32l4r9i_eval_mmc.c)
 * @brief   This file includes a generic uMMC card driver.
 *          To be completed by the user according to the board used for the project.
 * @note    Some functions generated as weak: they can be overridden by
 *          - code in user files
 *          - or BSP code from the FW pack files
 *          if such files are added to the generated project (by the user).
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

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_mmc.h"

/* Extern variables ---------------------------------------------------------*/

extern MMC_HandleTypeDef hmmc1;

/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
/* USER CODE END BeforeInitSection */
/**
 * @brief  Initializes the MMC card device.
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_Init(void)
{
  uint8_t mmc_state = MMMC_OK;
  /* Check if the MMC card is plugged in the slot */
  if (BSP_MMC_IsDetected() != MMC_PRESENT)
  {
    return MMMC_ERROR_MMC_NOT_PRESENT;
  }
  /* HAL MMC initialization */
  mmc_state = HAL_MMC_Init(&hmmc1);
  /* Configure MMC Bus width (4 bits mode selected) */
  if (mmc_state == MMMC_OK)
  {
    /* Enable wide operation */
    if (HAL_MMC_ConfigWideBusOperation(&hmmc1, SDMMC_BUS_WIDE_4B) != HAL_OK)
    {
      mmc_state = MMMC_ERROR;
    }
  }

  return mmc_state;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/* USER CODE BEGIN InterruptMode */
/**
 * @brief  Configures Interrupt mode for MMC detection pin.
 * @retval Returns 0
 */
__weak uint8_t BSP_MMC_ITConfig(void)
{
  /* Code to be updated by the user or replaced by one from the FW pack (in a stmxxxx_mmc.c file) */

  return (uint8_t)0;
}

/* USER CODE END InterruptMode */

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
 * @brief  Reads block(s) from a specified address in an MMC card, in polling mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  ReadAddr: Address from where data is to be read
 * @param  NumOfBlocks: Number of MMC blocks to read
 * @param  Timeout: Timeout for read operation
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  return MMC_read_blocks((uint8_t *)pData, ReadAddr, NumOfBlocks) == HAL_OK ? MMMC_OK : MMMC_ERROR;
}

/* USER CODE BEGIN BeforeWriteBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteBlocksSection */
/**
 * @brief  Writes block(s) to a specified address in an MMC card, in polling mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  WriteAddr: Address from where data is to be written
 * @param  NumOfBlocks: Number of MMC blocks to write
 * @param  Timeout: Timeout for write operation
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  return MMC_write_blocks((uint8_t *)pData, WriteAddr, NumOfBlocks) == HAL_OK ? MMMC_OK : MMMC_ERROR;
}

/* USER CODE BEGIN BeforeReadDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadDMABlocksSection */
/**
 * @brief  Reads block(s) from a specified address in an MMC card, in DMA mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  ReadAddr: Address from where data is to be read
 * @param  NumOfBlocks: Number of MMC blocks to read
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t mmc_state = MMMC_OK;

  /* Read block(s) in DMA transfer mode */
  if (HAL_MMC_ReadBlocks_DMA(&hmmc1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    mmc_state = MMMC_ERROR;
  }

  return mmc_state;
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
 * @brief  Writes block(s) to a specified address in an MMC card, in DMA mode.
 * @param  pData: Pointer to the buffer that will contain the data to transmit
 * @param  WriteAddr: Address from where data is to be written
 * @param  NumOfBlocks: Number of MMC blocks to write
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t mmc_state = MMMC_OK;

  /* Write block(s) in DMA transfer mode */
  if (HAL_MMC_WriteBlocks_DMA(&hmmc1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
  {
    mmc_state = MMMC_ERROR;
  }

  return mmc_state;
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
 * @brief  Erases the specified memory area of the given MMC card.
 * @param  StartAddr: Start byte address
 * @param  EndAddr: End byte address
 * @retval MMC status
 */
__weak uint8_t BSP_MMC_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t mmc_state = MMMC_OK;

  if (HAL_MMC_Erase(&hmmc1, StartAddr, EndAddr) != HAL_OK)
  {
    mmc_state = MMMC_ERROR;
  }

  return mmc_state;
}

/* USER CODE BEGIN BeforeGetCardStateSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeGetCardStateSection */

/**
 * @brief  Gets the current MMC card data status.
 * @param  None
 * @retval Data transfer state.
 *          This value can be one of the following values:
 *            @arg  MMC_TRANSFER_OK: No data transfer is acting
 *            @arg  MMC_TRANSFER_BUSY: Data transfer is acting
 */
__weak uint8_t BSP_MMC_GetCardState(void)
{
  return ((HAL_MMC_GetCardState(&hmmc1) == HAL_MMC_CARD_TRANSFER) ? MMC_TRANSFER_OK : MMC_TRANSFER_BUSY);
}

/**
 * @brief  Get MMC information about specific MMC card.
 * @param  CardInfo: Pointer to HAL_MMC_CardInfoTypedef structure
 * @retval None
 */
__weak void BSP_MMC_GetCardInfo(BSP_MMC_CardInfo *CardInfo)
{
  /* Get MMC card Information */
  HAL_MMC_GetCardInfo(&hmmc1, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
#if 0 
/* USER CODE END BeforeCallBacksSection */
/**
 * @brief MMC Abort callbacks
 * @param hmmc: MMC handle
 * @retval None
 */
void HAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_AbortCallback();
}

/**
 * @brief Tx Transfer completed callback
 * @param hmmc: MMC handle
 * @retval None
 */
void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_WriteCpltCallback();
}

/**
 * @brief Rx Transfer completed callback
 * @param hmmc: MMC handle
 * @retval None
 */
void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc)
{
  BSP_MMC_ReadCpltCallback();
}

/* USER CODE BEGIN CallBacksSection_C */
/**
 * @brief BSP MMC Abort callback
 * @retval None
 * @note empty (up to the user to fill it in or to remove it if useless)
 */
__weak void BSP_MMC_AbortCallback(void)
{
}

/**
 * @brief BSP Tx Transfer completed callback
 * @retval None
 * @note empty (up to the user to fill it in or to remove it if useless)
 */
__weak void BSP_MMC_WriteCpltCallback(void)
{
}

/**
 * @brief BSP Rx Transfer completed callback
 * @retval None
 * @note empty (up to the user to fill it in or to remove it if useless)
 */
__weak void BSP_MMC_ReadCpltCallback(void)
{
}
#endif
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if MMC card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if MMC is detected or not
 */
__weak uint8_t BSP_MMC_IsDetected(void)
{
  __IO uint8_t status = MMC_PRESENT;

  /* USER CODE BEGIN 1 */
  /* user code can be inserted here */
  /* USER CODE END 1 */

  return status;
}

/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
/* USER CODE END AdditionalCode */
