/*
 * Hexdump.h
 *
 *  Created on: Sep 20, 2020
 *      Author: thomashamilton
 */

#ifndef INC_HEXDUMP_H_
#define INC_HEXDUMP_H_

#include "CS43L22.h"
#include "FAT.h"
#include "Utils.h"
#include "stm32f407xx_usart_driver.h"
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
void Hexdump(USART_Handle_t* pUSARTHandle, void* pReadHandle, dumpInfo_t (*readBlock)(void*, uint32_t, uint32_t), uint32_t addr, uint32_t sectors);
void HexdumpBuffer(USART_Handle_t* pUSARTHandle, uint8_t* buf, uint32_t bufSize);

// Sub Functions
dumpInfo_t FATdump(void* handle, uint32_t addr, uint32_t blocks);
dumpInfo_t CodecDump(void* handle, uint32_t addr, uint32_t blocks);

#endif /* INC_HEXDUMP_H_ */
