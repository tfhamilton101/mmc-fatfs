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
#include "Utils.h"
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
 *  SD Read/Write Return types
 */
typedef enum
{
    SD_READ_WRITE_FAIL = 0,
    SD_READ_WRITE_SUCCESS,
} sd_read_write_t;


/************************************************************************************
 *                          SD Card Buffer Definitions
 *************************************************************************************/
#define SD_DEFAULT_BLOCK_SIZE (512)

// The SD RX buffer size must be a Powers of 2 multiple of the default block side
#define SD_BUFFER_SIZE (32 * SD_DEFAULT_BLOCK_SIZE)

/************************************************************************************
 *			        		SD Card Detection
 *************************************************************************************/
typedef enum
{
    CD_REMOVED,
    CD_DETECTED,
} card_detect_t;

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

/************************************************************************************
 *                              Public Functions
 *************************************************************************************/

/* Init and De-init */
void SD_Init(SD_Handle_t* pSDHandle);

/*** Hardware Init Functions ***/
void SD_Init_Timers(SD_Handle_t* pSDHandle, TIM_RegDef_t* pTIMx, irq_no_t irqNo);
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
