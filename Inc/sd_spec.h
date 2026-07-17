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

/************************************************************************* 
 *                    SDIO Response Structures                           * 
 *************************************************************************/

/*
 *  SDIO R1 Response - 48-bit short response with card status
 *  Format: [47:40] Command index, [39:8] Card status (32-bit), [7:1] CRC7, [0] End bit
 */
typedef struct
{
    uint32_t CardStatus;  /* 32-bit card status field from response */
} SDIO_R1_Response_t;

/*
 *  SDIO R2 Response - 136-bit long response (CID or CSD register)
 *  Format: [127:1] CID or CSD register contents, [0] End bit
 */
typedef struct
{
    uint32_t Data[4];  /* 128-bit register data (CID or CSD) */
} SDIO_R2_Response_t;

/*
 *  SDIO R3 Response - 48-bit short response with OCR
 *  Format: [47:40] Reserved, [39:8] OCR register, [7:1] Reserved, [0] End bit
 */
typedef struct
{
    uint32_t OCR;  /* 32-bit OCR register */
} SDIO_R3_Response_t;

/*
 *  SDIO R6 Response - 48-bit short response with RCA
 *  Format: [47:40] Command index, [39:24] RCA, [23:8] Card status bits, [7:1] CRC7, [0] End bit
 */
typedef struct
{
    uint16_t RCA;          /* 16-bit Relative Card Address */
    uint16_t CardStatus;   /* 16-bit card status bits */
} SDIO_R6_Response_t;

/*
 *  SDIO R7 Response - 48-bit short response (interface condition)
 *  Format: [47:40] Command index, [39:20] Reserved, [19:16] Voltage accepted, [15:8] Check pattern, [7:1] CRC7, [0] End bit
 */
typedef struct
{
    uint8_t CheckPattern;      /* 8-bit check pattern echo */
    uint8_t VoltageAccepted;   /* 4-bit voltage accepted (bits [19:16]) */
} SDIO_R7_Response_t;

/*
 *  SDIO Card Status Bit Positions (32-bit field in R1 response)
 *  Physical Layer Specification Version 3.01, Section 4.10.1
 */
typedef enum
{
    CARD_STATUS_AKE_SEQ_ERROR = 3,
    CARD_STATUS_APP_CMD = 5,
    CARD_STATUS_READY_FOR_DATA = 8,
    CARD_STATUS_CURRENT_STATE = 9,   /* Bits [12:9] - 4-bit state field */
    CARD_STATUS_ERASE_RESET = 13,
    CARD_STATUS_CARD_ECC_DISABLED = 14,
    CARD_STATUS_WP_ERASE_SKIP = 15,
    CARD_STATUS_CSD_OVERWRITE = 16,
    CARD_STATUS_ERROR = 19,
    CARD_STATUS_CC_ERROR = 20,
    CARD_STATUS_CARD_ECC_FAILED = 21,
    CARD_STATUS_ILLEGAL_COMMAND = 22,
    CARD_STATUS_COM_CRC_ERROR = 23,
    CARD_STATUS_LOCK_UNLOCK_FAILED = 24,
    CARD_STATUS_CARD_IS_LOCKED = 25,
    CARD_STATUS_WP_VIOLATION = 26,
    CARD_STATUS_ERASE_PARAM = 27,
    CARD_STATUS_ERASE_SEQ_ERROR = 28,
    CARD_STATUS_BLOCK_LEN_ERROR = 29,
    CARD_STATUS_ADDRESS_ERROR = 30,
    CARD_STATUS_OUT_OF_RANGE = 31,
} sdio_card_status_bits_t;

/*
 *  SDIO Card State Values (bits [12:9] of card status)
 */
typedef enum
{
    CARD_STATE_IDLE = 0,
    CARD_STATE_READY = 1,
    CARD_STATE_IDENT = 2,
    CARD_STATE_STBY = 3,
    CARD_STATE_TRAN = 4,
    CARD_STATE_DATA = 5,
    CARD_STATE_RCV = 6,
    CARD_STATE_PRG = 7,
    CARD_STATE_DIS = 8,
} sdio_card_state_t;

/* Helper macro to extract card state from status */
#define GET_CARD_STATE(status) (((status) >> CARD_STATUS_CURRENT_STATE) & 0xF)

/* Card status error bits mask */
#define CARD_STATUS_ERROR_MASK 0xFDFFE008

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
    /***  ALL_SEND_CID (SDIO)  ***/
    CMD2 = 2,
    /***  SEND_RELATIVE_ADDR (SDIO)  ***/
    CMD3 = 3,
    /***  SELECT_CARD / DESELECT_CARD (SDIO)  ***/
    CMD7 = 7,
    /***  SEND_IF_COND  ***/
    CMD8 = 8,
    /***  SEND_CSD (SDIO)  ***/
    CMD9 = 9,
    /***  STOP_TRANSMISSION  ***/
    CMD12 = 12,
    /***  SEND_STATUS (SDIO)  ***/
    CMD13 = 13,
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
    /***  SET_BUS_WIDTH (SDIO App Command)  ***/
    ACMD6 = 6,
    /***  APP_SEND_OP_COND  ***/
    ACMD41 = 41,
    /***  SEND_SCR (SDIO App Command)  ***/
    ACMD51 = 51,
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
    /***  SET_BUS_WIDTH (SDIO App Command)  ***/
    /* Argument:
     * [31:2] Reserved (stuff bits)
     * [1:0] Bus width: 00=1-bit, 10=4-bit
     */
    ACMD6_ARG_1BIT = 0x00000000,
    ACMD6_ARG_4BIT = 0x00000002,
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

#endif /* INC_SD_SPEC_H_ */