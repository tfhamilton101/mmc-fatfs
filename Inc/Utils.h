/*
 * Utils.h
 *
 *  Created on: Nov 4th, 2020
 *      Author: thomashamilton
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <ctype.h>
#include <stdint.h>

/************************************************************************************
 *			        		Utility Function Macros									*
 ************************************************************************************/
typedef enum
{
    DATA_SIZE_BYTE = 1,
    DATA_SIZE_HALF_WORD = 2,
    DATA_SIZE_WORD = 4,
} DataSize_t;

/************************************************************************************
 *			        		Function Declarations									*
 ************************************************************************************/
uint32_t ToLittleEndian(uint8_t* data, DataSize_t size);
uint32_t ToBigEndian(uint8_t* data, DataSize_t size);
void ToEndianBuf(uint8_t* data, uint32_t value, DataSize_t size);

/************************************************************************************
 *			        			Useful Macros 										*
 ************************************************************************************/
#define LowerNibble(x) (x & 0x0F)
#define UpperNibble(x) ((x >> 4) & 0x0F)
void bytesToHex(uint8_t* buf, uint32_t num, DataSize_t size);
// Copy a string and convert each character to lower case
void strncpylower(uint8_t* dest, uint8_t* source, uint8_t n);
void strncpyUpper(uint8_t* dest, uint8_t* source, uint8_t n);
#define strntolower(str, n) (strncpylower(str, str, n))
#define strntoUpper(str, n) (strncpyUpper(str, str, n))

#define HIBYTE(x) ((uint8_t)(((uint16_t)(x) >> 8) & 0x00FF))
#define LOBYTE(x) ((uint8_t)((uint16_t)(x)&0x00FF))
#define HIBITS(x) ((uint8_t)(((uint8_t)(x) >> 4) & 0x0F))
#define LOBITS(x) ((uint8_t)((uint8_t)(x)&0x0F))
#define SWAP16BIT(x) (HIBYTE(x) | ((uint16_t)LOBYTE(x) << 8))
#define MAKEWORD(x, y) (((uint32_t)x << 16) | (uint16_t)y)
#define MAKEHALFWORD(x, y) (((uint16_t)x << 8) | (uint8_t)y)

// Generic Bit Definitions
typedef enum
{
    BIT0 = (1 << 0),
    BIT1 = (1 << 1),
    BIT2 = (1 << 2),
    BIT3 = (1 << 3),
    BIT4 = (1 << 4),
    BIT5 = (1 << 5),
    BIT6 = (1 << 6),
    BIT7 = (1 << 7),
    BIT8 = (1 << 8),
    BIT9 = (1 << 9),
    BIT10 = (1 << 10),
    BIT11 = (1 << 11),
    BIT12 = (1 << 12),
    BIT13 = (1 << 13),
    BIT14 = (1 << 14),
    BIT15 = (1 << 15),
    BIT16 = (1 << 16),
    BIT17 = (1 << 17),
    BIT18 = (1 << 18),
    BIT19 = (1 << 19),
    BIT20 = (1 << 20),
    BIT21 = (1 << 21),
    BIT22 = (1 << 22),
    BIT23 = (1 << 23),
    BIT24 = (1 << 24),
    BIT25 = (1 << 25),
    BIT26 = (1 << 26),
    BIT27 = (1 << 27),
    BIT28 = (1 << 28),
    BIT29 = (1 << 29),
    BIT30 = (1 << 30),
    BIT31 = (1 << 31),
} bit_definition_t;

#endif /* INC_UTILS_H_ */
