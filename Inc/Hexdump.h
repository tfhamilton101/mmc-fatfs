/*
 * Hexdump.h
 *
 *  Created on: Sep 20, 2020
 *      Author: thomashamilton
 */

#ifndef INC_HEXDUMP_H_
#define INC_HEXDUMP_H_

#include "FAT.h"
#include "Utils.h"
#include "stm32f4xx_usart_driver.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/************************************************************************************
 *                            Structure Declarations                                *
 ************************************************************************************/
typedef struct
{
    uint8_t* pbuff;
    uint32_t buffSize;
    uint32_t blockSize;
    uint32_t addrUnit;
} dumpInfo_t;

#define DUMP_BUF_SIZE 256

/************************************************************************************
 *                            Function Declarations                                 *
 ************************************************************************************/
void Hexdump(void* pReadHandle, dumpInfo_t (*readBlock)(void*, uint32_t, uint32_t), uint32_t addr, uint32_t sectors);
void HexdumpBuffer(uint8_t* buf, uint32_t bufSize);

// Sub Functions
dumpInfo_t FATdumpAddr(void* handle, uint32_t addr, uint32_t blocks);
dumpInfo_t FATdumpCluster(void* handle, uint32_t cluster, uint32_t blocks);

#endif /* INC_HEXDUMP_H_ */
