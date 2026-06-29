/*
 * sd_ops.h
 *
 *  Created on: June 28, 2026
 *      Author: thomashamilton
 * 
 */

#ifndef INC_SD_OPS_H_
#define INC_SD_OPS_H_

#include <stdint.h>

/**
 * SD opps driver API
 * 
 * Function pointer table that defines the SD ops driver interface.
 * Both SPI and SDIO implementation must provide these functions.
 */
struct sd_ops
{
    int (*init)(void* cfg);
    int (*ReadBlock)(void* cfg, uint8_t* pData, uint32_t addr, uint32_t count);
    int (*WriteBlock)(void* cfg, uint8_t* pData, uint32_t addr, uint32_t count);
};

#endif /* INC_SD_OPS_H_ */