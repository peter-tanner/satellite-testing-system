/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    user_diskio.c
 * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "bsp_driver_mmc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define QUEUE_SIZE (uint32_t)10
#define READ_CPLT_MSG (uint32_t)1
#define WRITE_CPLT_MSG (uint32_t)2
/*
==================================================================
enable the defines below to send custom rtos messages
when an error or an abort occurs.
Notice: depending on the HAL/MMC driver the HAL_MMC_ErrorCallback()
may not be available.
See BSP_MMC_ErrorCallback() and BSP_MMC_AbortCallback() below
==================================================================

#define RW_ERROR_MSG       (uint32_t) 3
#define RW_ABORT_MSG       (uint32_t) 4
*/
/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_MMC_ReadCpltCallback() or BSP_MMC_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define MMC_TIMEOUT 30 * 1000

#define MMC_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the MMC card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_MMC_Init() call in the MMC_Initialize() and add a call to
 * BSP_MMC_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableMMCInit */
/* #define DISABLE_MMC_INIT */
/* USER CODE END disableMMCInit */

/*
 * when using cacheable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableMMCDmaCacheMaintenance */
/* #define ENABLE_MMC_DMA_CACHE_MAINTENANCE  1 */
/* USER CODE END enableMMCDmaCacheMaintenance */

/*
 * Some DMA requires 4-Byte aligned address buffer to correctly read/write data,
 * in FatFs some accesses aren't thus we need a 4-byte aligned scratch buffer to correctly
 * transfer data
 */
/* USER CODE BEGIN enableScratchBuffer */
/* #define ENABLE_SCRATCH_BUFFER */
/* USER CODE END enableScratchBuffer */

/* Private variables ---------------------------------------------------------*/
#if defined(ENABLE_SCRATCH_BUFFER)
#if defined(ENABLE_MMC_DMA_CACHE_MAINTENANCE)
ALIGN_32BYTES(static uint8_t scratch[BLOCKSIZE]); // 32-Byte aligned for cache maintenance
#else
__ALIGN_BEGIN static uint8_t scratch[BLOCKSIZE] __ALIGN_END;
#endif
#endif

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

#if (osCMSIS <= 0x20000U)
static osMessageQId MMCQueueID = NULL;
#else
static osMessageQueueId_t MMCQueueID = NULL;
#endif

static int MMC_CheckStatusWithTimeout(uint32_t timeout)
{
  uint32_t timer;
  /* block until MMCIO peripheral is ready again or a timeout occur */
#if (osCMSIS <= 0x20000U)
  timer = osKernelSysTick();
  while (osKernelSysTick() - timer < timeout)
#else
  timer = osKernelGetTickCount();
  while (osKernelGetTickCount() - timer < timeout)
#endif
  {
    if (BSP_MMC_GetCardState() == MMC_TRANSFER_OK)
    {
      return 0;
    }
  }

  return -1;
}

static DSTATUS MMC_CheckStatus(BYTE pdrv)
{
  Stat = STA_NOINIT;

  if (BSP_MMC_GetCardState() == MMC_TRANSFER_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
 * @brief Tx Transfer completed callbacks
 * @param hmmc: MMC handle
 * @retval None
 */
void BSP_MMC_WriteCpltCallback(void)
{

  /*
   * No need to add an "osKernelRunning()" check here, as the MMC_initialize()
   * is always called before any MMC_Read()/MMC_Write() call
   */
#if (osCMSIS < 0x20000U)
  osMessagePut(MMCQueueID, WRITE_CPLT_MSG, 0);
#else
  const uint16_t msg = WRITE_CPLT_MSG;
  osMessageQueuePut(MMCQueueID, (const void *)&msg, 0, 0);
#endif
}

/**
 * @brief Rx Transfer completed callbacks
 * @param hmmc: MMC handle
 * @retval None
 */
void BSP_MMC_ReadCpltCallback(void)
{
  /*
   * No need to add an "osKernelRunning()" check here, as the MMC_initialize()
   * is always called before any MMC_Read()/MMC_Write() call
   */
#if (osCMSIS < 0x20000U)
  osMessagePut(MMCQueueID, READ_CPLT_MSG, 0);
#else
  const uint16_t msg = READ_CPLT_MSG;
  osMessageQueuePut(MMCQueueID, (const void *)&msg, 0, 0);
#endif
}

/* USER CODE BEGIN ErrorAbortCallbacks */
/*
void BSP_MMC_AbortCallback(void)
{
#if (osCMSIS < 0x20000U)
   osMessagePut(MMCQueueID, RW_ABORT_MSG, 0);
#else
   const uint16_t msg = RW_ABORT_MSG;
   osMessageQueuePut(MMCQueueID, (const void *)&msg, 0, 0);
#endif
}
*/
/* USER CODE END ErrorAbortCallbacks */

/* USER CODE BEGIN lastSection */
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
  // NOTE: ONLY WORKS FOR PDRV=0
  static uint8_t initialized = 0;

  if (!initialized)
  {
    initialized = 1;
    Stat = STA_NOINIT;

    /*
     * check that the kernel has been started before continuing
     * as the osMessage API will fail otherwise
     */
    // #if (osCMSIS <= 0x20000U)
    //     if (osKernelRunning())
    // #else
    //     if (osKernelGetState() == osKernelRunning)
    // #endif
    {
#if !defined(DISABLE_MMC_INIT)

      if (BSP_MMC_Init() == MMMC_OK)
      {
        Stat = MMC_CheckStatus(pdrv);
      }

#else
      Stat = MMC_CheckStatus(pdrv);
#endif

      /*
       * if the MMC is correctly initialized, create the operation queue
       * if not already created
       */

      // if (Stat != STA_NOINIT)
      // {
      if (MMCQueueID == NULL)
      {
#if (osCMSIS <= 0x20000U)
        osMessageQDef(MMC_Queue, QUEUE_SIZE, uint16_t);
        MMCQueueID = osMessageCreate(osMessageQ(MMC_Queue), NULL);
#else
        MMCQueueID = osMessageQueueNew(QUEUE_SIZE, 2, NULL);
#endif
      }

      if (MMCQueueID == NULL)
      {
        Stat |= STA_NOINIT;
      }
      // }
    }
  }
  else
  {
    Stat = RES_OK;
  }
  return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
  return MMC_CheckStatus(pdrv);
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
  uint8_t ret;
  DRESULT res = RES_ERROR;
  uint32_t timer;
#if (osCMSIS < 0x20000U)
  osEvent event;
#else
  uint16_t event;
  osStatus_t status;
#endif
#if (ENABLE_MMC_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif
  /*
   * ensure the MMCCard is ready for a new operation
   */

  if (MMC_CheckStatusWithTimeout(MMC_TIMEOUT) < 0)
  {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x3))
  {
#endif
    /* Fast path cause destination buffer is correctly aligned */
    ret = BSP_MMC_ReadBlocks_DMA((uint32_t *)buff, (uint32_t)(sector), count);

    if (ret == MMMC_OK)
    {
#if (osCMSIS < 0x20000U)
      /* wait for a message from the queue or a timeout */
      event = osMessageGet(MMCQueueID, MMC_TIMEOUT);

      if (event.status == osEventMessage)
      {
        if (event.value.v == READ_CPLT_MSG)
        {
          timer = osKernelSysTick();
          /* block until MMCIO IP is ready or a timeout occur */
          while (osKernelSysTick() - timer < MMC_TIMEOUT)
#else
    status = osMessageQueueGet(MMCQueueID, (void *)&event, NULL, MMC_TIMEOUT);
    if ((status == osOK) && (event == READ_CPLT_MSG))
    {
      timer = osKernelGetTickCount();
      /* block until MMCIO IP is ready or a timeout occur */
      while (osKernelGetTickCount() - timer < MMC_TIMEOUT)
#endif
          {
            if (BSP_MMC_GetCardState() == MMC_TRANSFER_OK)
            {
              res = RES_OK;
#if (ENABLE_MMC_DMA_CACHE_MAINTENANCE == 1)
              /*
              the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
              adjust the address and the D-Cache size to invalidate accordingly.
              */
              alignedAddr = (uint32_t)buff & ~0x1F;
              SCB_InvalidateDCache_by_Addr((uint32_t *)alignedAddr, count * BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif
              break;
            }
          }
#if (osCMSIS < 0x20000U)
        }
      }
#else
    }
#endif
    }

#if defined(ENABLE_SCRATCH_BUFFER)
  }
  else
  {
    /* Slow path, fetch each sector a part and memcpy to destination buffer */
    int i;

    for (i = 0; i < count; i++)
    {
      ret = BSP_MMC_ReadBlocks_DMA((uint32_t *)scratch, (uint32_t)sector++, 1);
      if (ret == MMMC_OK)
      {
        /* wait until the read is successful or a timeout occurs */
#if (osCMSIS < 0x20000U)
        /* wait for a message from the queue or a timeout */
        event = osMessageGet(MMCQueueID, MMC_TIMEOUT);

        if (event.status == osEventMessage)
        {
          if (event.value.v == READ_CPLT_MSG)
          {
            timer = osKernelSysTick();
            /* block until MMCIO IP is ready or a timeout occur */
            while (osKernelSysTick() - timer < MMC_TIMEOUT)
#else
        status = osMessageQueueGet(MMCQueueID, (void *)&event, NULL, MMC_TIMEOUT);
        if ((status == osOK) && (event == READ_CPLT_MSG))
        {
          timer = osKernelGetTickCount();
          /* block until MMCIO IP is ready or a timeout occur */
          ret = MMMC_ERROR;
          while (osKernelGetTickCount() - timer < MMC_TIMEOUT)
#endif
            {
              ret = BSP_MMC_GetCardState();

              if (ret == MMMC_OK)
              {
                break;
              }
            }

            if (ret != MMMC_OK)
            {
              break;
            }
#if (osCMSIS < 0x20000U)
          }
        }
#else
        }
#endif
#if (ENABLE_MMC_DMA_CACHE_MAINTENANCE == 1)
        /*
         *
         * invalidate the scratch buffer before the next read to get the actual data instead of the cached one
         */
        SCB_InvalidateDCache_by_Addr((uint32_t *)scratch, BLOCKSIZE);
#endif
        memcpy(buff, scratch, BLOCKSIZE);
        buff += BLOCKSIZE;
      }
      else
      {
        break;
      }
    }

    if ((i == count) && (ret == MMMC_OK))
      res = RES_OK;
  }
#endif
  return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  DRESULT res = RES_ERROR;
  uint32_t timer;

#if (osCMSIS < 0x20000U)
  osEvent event;
#else
  uint16_t event;
  osStatus_t status;
#endif

#if defined(ENABLE_SCRATCH_BUFFER)
  int32_t ret;
#endif

  /*
   * ensure the MMCCard is ready for a new operation
   */

  if (MMC_CheckStatusWithTimeout(MMC_TIMEOUT) < 0)
  {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x3))
  {
#endif
#if (ENABLE_MMC_DMA_CACHE_MAINTENANCE == 1)
    uint32_t alignedAddr;
    /*
      the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
      adjust the address and the D-Cache size to clean accordingly.
    */
    alignedAddr = (uint32_t)buff & ~0x1F;
    SCB_CleanDCache_by_Addr((uint32_t *)alignedAddr, count * BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif

    if (BSP_MMC_WriteBlocks_DMA((uint32_t *)buff,
                                (uint32_t)(sector),
                                count) == MMMC_OK)
    {
#if (osCMSIS < 0x20000U)
      /* Get the message from the queue */
      event = osMessageGet(MMCQueueID, MMC_TIMEOUT);

      if (event.status == osEventMessage)
      {
        if (event.value.v == WRITE_CPLT_MSG)
        {
#else
    status = osMessageQueueGet(MMCQueueID, (void *)&event, NULL, MMC_TIMEOUT);
    if ((status == osOK) && (event == WRITE_CPLT_MSG))
    {
#endif
#if (osCMSIS < 0x20000U)
          timer = osKernelSysTick();
          /* block until MMCIO IP is ready or a timeout occur */
          while (osKernelSysTick() - timer < MMC_TIMEOUT)
#else
      timer = osKernelGetTickCount();
      /* block until MMCIO IP is ready or a timeout occur */
      while (osKernelGetTickCount() - timer < MMC_TIMEOUT)
#endif
          {
            if (BSP_MMC_GetCardState() == MMC_TRANSFER_OK)
            {
              res = RES_OK;
              break;
            }
          }
#if (osCMSIS < 0x20000U)
        }
      }
#else
    }
#endif
    }
#if defined(ENABLE_SCRATCH_BUFFER)
    else
    {
      /* Slow path, fetch each sector a part and memcpy to destination buffer */
      int i;

#if (ENABLE_MMC_DMA_CACHE_MAINTENANCE == 1)
      /*
       * invalidate the scratch buffer before the next write to get the actual data instead of the cached one
       */
      SCB_InvalidateDCache_by_Addr((uint32_t *)scratch, BLOCKSIZE);
#endif
      for (i = 0; i < count; i++)
      {
        memcpy((void *)scratch, buff, BLOCKSIZE);
        buff += BLOCKSIZE;

        ret = BSP_MMC_WriteBlocks_DMA((uint32_t *)scratch, (uint32_t)sector++, 1);
        if (ret == MMMC_OK)
        {
          /* wait until the read is successful or a timeout occurs */
#if (osCMSIS < 0x20000U)
          /* wait for a message from the queue or a timeout */
          event = osMessageGet(MMCQueueID, MMC_TIMEOUT);

          if (event.status == osEventMessage)
          {
            if (event.value.v == READ_CPLT_MSG)
            {
              timer = osKernelSysTick();
              /* block until MMCIO IP is ready or a timeout occur */
              while (osKernelSysTick() - timer < MMC_TIMEOUT)
#else
          status = osMessageQueueGet(MMCQueueID, (void *)&event, NULL, MMC_TIMEOUT);
          if ((status == osOK) && (event == READ_CPLT_MSG))
          {
            timer = osKernelGetTickCount();
            /* block until MMCIO IP is ready or a timeout occur */
            ret = MMMC_ERROR;
            while (osKernelGetTickCount() - timer < MMC_TIMEOUT)
#endif
              {
                ret = BSP_MMC_GetCardState();

                if (ret == MMMC_OK)
                {
                  break;
                }
              }

              if (ret != MMMC_OK)
              {
                break;
              }
#if (osCMSIS < 0x20000U)
            }
          }
#else
          }
#endif
        }
        else
        {
          break;
        }
      }

      if ((i == count) && (ret == MMMC_OK))
        res = RES_OK;
    }
  }
#endif

  return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
  DRESULT res = RES_ERROR;
  BSP_MMC_CardInfo CardInfo;

  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC:
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT:
    BSP_MMC_GetCardInfo(&CardInfo);
    *(DWORD *)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE:
    BSP_MMC_GetCardInfo(&CardInfo);
    *(WORD *)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE:
    BSP_MMC_GetCardInfo(&CardInfo);
    *(DWORD *)buff = CardInfo.LogBlockSize / MMC_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

