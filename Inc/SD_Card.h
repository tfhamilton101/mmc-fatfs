/*
 * SD_Card.h
 *
 *  Created on: Sep 13, 2020
 *      Author: thomashamilton
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

// Standard Libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "Utils.h"
#include "stm32f407xx_dma_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_timer_driver.h"

/************************************************************************* 
 *                         SD Handler Subtypes                           *
 *************************************************************************/

/*
 *  @SD_States_t
 *  Possible States for SD
 */
typedef enum
{
    SD_STATE_IDLE = 0,
    SD_STATE_FAIL,
    SD_STATE_INIT,
    SD_STATE_BUSY,
    SD_STATE_READY,
    SD_STATE_NO_CARD,
} SD_States_t;

/*
 *  @SD_mode
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_MODE_SPI = 0,
    SD_MODE_SDIO = 1,
} sd_mode_t;

/*
 *  @SD_CRCEn
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_CRC_DI = 0,
    SD_CRC_EN,
} sd_crc_modes_t;

/*
 *  @SD_TransferMode
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_TRANSFER_NON_DMA = 0,
    SD_TRANSFER_DMA,
} sd_trans_modes_t;

/*
 * @SD_CardType
 */
typedef enum
{
    SD_CARDTYPE_SDSC = 0,
    SD_CARDTYPE_SDXC_SDHC = 1,
} sd_card_types_t;

/*
 * @bufferInfo_t
 * Buffer Related Variables
 */
typedef struct
{
    uint32_t Size;     /*   Size of Data TX & RX buffer               */
    uint8_t* pBufA;    /*   Memory location of the TX buffer          */
    uint8_t* pBufB;    /*   Memory location of the TX buffer          */
    uint8_t* pCurrBuf; /*   Memory location of the current buffer     */
} bufferInfo_t;

typedef enum
{
    TIMEOUT_NON_EXPIRED = 0,
    TIMEOUT_EXPIRED,
} timeout_status_t;

/*
 * Command Timeout
 */
typedef struct
{
    TIM_Handle_t TimHandle;
    timeout_status_t Status;
} Timeout_t;

/*
 * SD DMA Streams
 */
typedef enum
{
    SD_DMA_STREAM_WRITE = 0,
    SD_DMA_STREAM_READ,
    SD_DMA_CHANNELS,
} sd_dma_streams_t;

/*
 *  Configuration structure for SD Card
 */
typedef struct
{
    sd_mode_t SD_Mode;                /*!  < possible values from @SD_mode>          */
    sd_crc_modes_t SD_CRCEn;          /*!  < possible values from @SD_CRCEn>         */
    sd_card_types_t SD_CardType;      /*!  < possible values from @SD_CardType>      */
    SD_States_t SD_CardState;         /*!  < possible values from @SD_CardState>     */
    sd_trans_modes_t SD_TransferMode; /*   < possible values from @SD_TransferMode   */
    bufferInfo_t bufferInfo;          /*   Buffer Info Structure                     */
    SPI_RegDef_t* pSPIx;              /*   Holds the base address SPIx  peripheral   */
    GPIO_Handle_t ChipSelHandle;      /*   Handler Chip select GPIOx                 */
    GPIO_Handle_t CardDetHandle;      /*   Handler Card Detect GPIOx                 */
    gpio_pin_state_t CardDetPol;      /*   SD Card detect polarity                   */
    Timeout_t cmdTimeout;             /*   Timer Handler for Command Timeouts        */
} SD_Handle_t;

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
 *  SD Read/Write Return types
 */
typedef enum
{
    SD_READ_WRITE_FAIL = 0,
    SD_READ_WRITE_SUCCESS,
} sd_read_write_t;

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
    TRAN_BIT = BIT6,
    STOP_BIT = BIT0,
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
 *                          SD Card Buffer Definitions
 *************************************************************************************/
#define SD_DEFAULT_BLOCK_SIZE (512)

// The SD RX buffer size must be a Powers of 2 multiple of the default block side
#define SD_BUFFER_SIZE (32 * SD_DEFAULT_BLOCK_SIZE)

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

/************************************************************************************
 *			        		SD Card Detection
 *************************************************************************************/
typedef enum
{
    CD_REMOVED,
    CD_DETECTED,
} card_detect_t;

/************************************************************************************
 *                              SD Init States                                      
 *************************************************************************************/
typedef enum
{
    CMD0_FAIL = 0,
    CMD8_FAIL,
    INIT_FAIL,
    INIT_SUCCESS,
} SD_Init_States_t;

/************************************************************************************
 *                              SD Hardware Type
 *************************************************************************************/
typedef enum
{
    SD_HARDWARE_SPI = 0,
    SD_HARDWARE_SDIO,
} sd_hardware_type_t;

/************************************************************************************
 *                              SD Timers Definitions
 *************************************************************************************/

/*
 * Set prescaler for the Command delay
 * Prescaler = 1 * ( 250ms ) * ( 40MHz ) / 65535 = 
 *
 * Note: When TIMPRE bit of the RCC_DCKCFGR register is reset, if APBx prescaler is 1 (RCC_APBx_AHB_DIV_NONE),
 *       then TIMxCLK = PCLKx, otherwise TIMxCLK = 2x PCLKx. See clock tree for more details.
 */
#define SD_TIMEOUT_PRESCALE 152

// Timer peripheral for Command Timeout
#define SD_TIMEOUT_TIMER TIM3

/************************************************************************************
 *                              Public Functions
 *************************************************************************************/

/* Init and De-init */
void SD_Init(SD_Handle_t* pSDHandle);

/*** Hardware Init Functions ***/
void SD_Init_Timers(SD_Handle_t* pSDHandle, TIM_RegDef_t* pTIMx);
void SD_Init_Hardware(SD_Handle_t* pSDHandle, sd_hardware_type_t type);

/* Read / Write function */
sd_read_write_t SD_ReadBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount);
sd_read_write_t SD_WriteBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount);

/* Helper function */
SD_States_t SD_GetState(SD_Handle_t* pSDHandle);
bool SD_IsReady(SD_Handle_t* pSDHandle);
uint8_t* SD_GetBuffAddr(SD_Handle_t* pSDHandle);
uint32_t SD_GetBuffSize(SD_Handle_t* pSDHandle);
void SD_ToggleCurrBuff(SD_Handle_t* pSDHandle);

/* I/O Functions */
// TODO: Consider making this static
card_detect_t SD_GetCDStatus(SD_Handle_t* pSDHandle);
timeout_status_t SD_GetTimeoutStatus(SD_Handle_t* pSDHandle);

/* IRQ Functions */
void SD_IRQHandling(SD_Handle_t* pSDHandle);

/************************************************************************************
 *			        		Externs for other files to use						*
 ************************************************************************************/

/* Handler for SD Card */
extern SD_Handle_t SD_Handle;

#endif /* INC_SD_CARD_H_ */
