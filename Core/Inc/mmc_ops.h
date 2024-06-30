#ifndef __MMC_OPS_H
#define __MMC_OPS_H
#include "main.h"

int8_t MMC_read_blocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t MMC_write_blocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);

#endif