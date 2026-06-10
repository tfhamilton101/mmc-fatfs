/*
 * sd_spec.h
 *
 *  Created on: June 10, 2026
 *      Author: thomashamilton
 */

#ifndef INC_SD_SPEC_H_
#define INC_SD_SPEC_H_

#include <stdint.h>

 /************************************************************************************
 *                          SD block size Definitions
 *************************************************************************************/
#define SD_DEFAULT_BLOCK_SIZE (512)

/************************************************************************* 
 *                         SD Command Typedefs                           * 
 *************************************************************************/

/*
 *  Structure for SPI R1 Response Flags
 */
typedef union {
    uint8_t Flags;
    struct
    {
        uint8_t Idle : 1;
        uint8_t Erase_Reset : 1;
        uint8_t Illigal_Command : 1;
        uint8_t Command_CRC_Err : 1;
        uint8_t Erase_Seq_Err : 1;
        uint8_t Address_Err : 1;
        uint8_t Paramenter_Error : 1;
        uint8_t Not_Used : 1;
    };
} R1_Response_t;

/*
 *  Structure for SPI R3 Response Flags
 */
typedef struct
{
    uint32_t OCR;
} R3_Response_t;

/*
 *  Structure for SPI R7 Response Flags
 */
typedef struct
{
    uint8_t Check_Pattern;
    uint8_t Voltage_Accepted;
    uint16_t Reserved;
    uint8_t Command_Version;
} R7_Response_t;

/*
 *  Structure to Hold Parsed SPI Command Response Flags
 */
typedef struct
{
    R1_Response_t R1;
    R3_Response_t R3;
    R7_Response_t R7;
} Command_Response_t;

/*
 *  SD Command Response Types
 */
typedef enum
{
    RESPONSE_R1 = 1,
    RESPONSE_R3 = 3,
    RESPONSE_R7 = 7,
} sd_response_t;


/************************************************************************************
 *			        		SD Response Macros
 *************************************************************************************/
#define R1_IDLE 0x01
#define CMD0_MAX_ATTEMPTS 8

/************************************************************************************
 *			        		Command Generic Macros
 *************************************************************************************/

// Command Index byte, 32-bit Argument and optional CRC.
#define COMMAND_FRAME_LEN 6

/* Command response time */
#define NCR_BYTES 8

/* Bit Definitions for SD Command */
typedef enum
{
    TRAN_BIT = (1 << 6),
    STOP_BIT = (1 << 0),
} command_frame_bits_t;

/************************************************************************************
 *			        		SD Card Register Bit positions
 *************************************************************************************/
typedef enum
{
    OCR_CCS = 30,
    OCR_PWR_UP_STATUS = 31,
} OCR_bit_pos_t;

/************************************************************************************
 *			        		SD Command Macros
 *************************************************************************************/

/* Command ID Definitions */
typedef enum
{
    /***  GO_IDLE_STATE  ***/
    CMD0 = 0,
    /***  SEND_IF_COND  ***/
    CMD8 = 8,
    /***  STOP_TRANSMISSION  ***/
    CMD12 = 12,
    /***  SET_BLOCKLEN  ***/
    CMD16 = 16,
    /***  READ_SINGLE_BLOCK  ***/
    CMD17 = 17,
    /***  READ_MULTIPLE_BLOCK  ***/
    CMD18 = 18,
    /***  SET_BLOCK_COUNT  ***/
    CMD23 = 23,
    /***  WRITE_BLOCK  ***/
    CMD24 = 24,
    /***  WRITE_MULTIPLE_BLOCK  ***/
    CMD25 = 25,
    /***  APP_CMD  ***/
    CMD55 = 55,
    /***  READ_OCR  ***/
    CMD58 = 58,
    /***  APP_SEND_OP_COND  ***/
    ACMD41 = 41,
} sd_cmd_ID_t;

/* Command Args Definitions */
typedef enum
{
    CMD_ARG_NULL = 0x00000000,
    /***  SEND_IF_COND  ***/
    /* Argument:
     * [31:12] Reserved
     * [11:8] Voltage Applied
     * [7:0] Check Pattern
     */
    CMD8_ARG = 0x000001AA,
    /***  SET_BLOCKLEN  ***/
    CMD16_ARG = SD_DEFAULT_BLOCK_SIZE,
    /***  APP_SEND_OP_COND  ***/
    /* Argument:
     * [31] Reserved bit
     * [30] HCS OCR[30]
     * [29] reserved for eSD
     * [28] XPC
     * [27:25] reserved bits
     * [24] S18R
     * [23:0] Vdd voltage (OCR[23:0])
     */
    ACMD41_ARG = 0x40000000,
} sd_cmd_args_t;

/* Command CRC Definitions */
typedef enum
{
    CMD_CRC_NULL = 0x00,
    /***  GO_IDLE_STATE  ***/
    CMD0_CRC = 0x95,
    /***  SEND_IF_COND  ***/
    CMD8_CRC = 0x87,
} sd_cmd_crc_t;

/* Command Token Definitions */
typedef enum
{
    BLOCK_READ_TOKEN = 0xFE,
    SINGLE_BLOCK_WRITE_TOKEN = 0xFE,
    MULT_BLOCK_WRITE_TOKEN = 0xFC,
    STOP_WRITE_TOKEN = 0xFD,
} sd_cmd_tokens_t;

/**
 *  Write Token Definitions 
 * 
 *  Format: 
 *  [7:5] Dont Care
 *  [4] '0'
 *  [3:1] Status
 *  [0] '1'
 **/
typedef enum
{
    WRITE_DATA_ACCEPTED = 0xE5,
    WRITE_DATA_REJECTED_CRC = 0xEB,
    WRITE_DATA_REJECTED_ERR = 0xED,
} sd_write_tokens_t;

/*
 * Card Types
 */
typedef enum
{
    SD_CARDTYPE_SDSC = 0,
    SD_CARDTYPE_SDXC_SDHC = 1,
} sd_card_types_t;


#endif /* INC_SD_SPEC_H_ */