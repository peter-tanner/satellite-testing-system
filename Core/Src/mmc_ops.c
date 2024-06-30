#include "mmc_ops.h"

extern MMC_HandleTypeDef hmmc1;
extern volatile uint8_t mmc_transaction_blks_left;

int8_t MMC_read_blocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    hmmc1.ErrorCode = HAL_MMC_ERROR_NONE;
    while (HAL_MMC_GetCardState(&hmmc1) != HAL_MMC_CARD_TRANSFER)
        ; // FIXME: IMPLEMENT TIMEOUT FAILSAFE
#ifdef BLOCKING
    if (HAL_MMC_ReadBlocks(&hmmc1, buf, blk_addr, blk_len, TIMEOUT) != HAL_OK)
        return HAL_ERROR;
#else
    mmc_transaction_blks_left = 1;
    if (MMC_DMA_direction(DMA_PERIPH_TO_MEMORY) != HAL_OK)
    {
        mmc_transaction_blks_left = 0;
        return HAL_ERROR;
    }
    if (HAL_MMC_ReadBlocks_DMA(&hmmc1, buf, blk_addr, blk_len) != HAL_OK)
    {
        mmc_transaction_blks_left = 0;
        return HAL_ERROR;
    }
    while (mmc_transaction_blks_left)
        ;
#endif
    hmmc1.ErrorCode = HAL_MMC_ERROR_NONE;
    while (HAL_MMC_GetCardState(&hmmc1) != HAL_MMC_CARD_TRANSFER)
        ; // FIXME: IMPLEMENT TIMEOUT FAILSAFE
    return HAL_OK;
}

int8_t MMC_write_blocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    hmmc1.ErrorCode = HAL_MMC_ERROR_NONE;
    while (HAL_MMC_GetCardState(&hmmc1) != HAL_MMC_CARD_TRANSFER)
        ; // FIXME: IMPLEMENT TIMEOUT FAILSAFE
#ifdef BLOCKING
    if (HAL_MMC_WriteBlocks(&hmmc1, buf, blk_addr, blk_len, TIMEOUT) != HAL_OK)
        return HAL_ERROR;
#else
    mmc_transaction_blks_left = 1;
    if (MMC_DMA_direction(DMA_MEMORY_TO_PERIPH) != HAL_OK)
    {
        mmc_transaction_blks_left = 0;
        return HAL_ERROR;
    }
    if (HAL_MMC_WriteBlocks_DMA(&hmmc1, buf, blk_addr, blk_len) != HAL_OK)
    {
        mmc_transaction_blks_left = 0;
        return HAL_ERROR;
    }
    while (mmc_transaction_blks_left)
        ;
#endif
    hmmc1.ErrorCode = HAL_MMC_ERROR_NONE;
    while (HAL_MMC_GetCardState(&hmmc1) != HAL_MMC_CARD_TRANSFER)
        ; // FIXME: IMPLEMENT TIMEOUT FAILSAFE
    return HAL_OK;
}
