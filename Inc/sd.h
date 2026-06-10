/*
 * SD_Card.h
 *
 *  Created on: Sep 13, 2020
 *      Author: thomashamilton
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

// Standard Libraries
#include <stdint.h>
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_timer_driver.h"

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
 *  @mode
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_MODE_SPI = 0,
    SD_MODE_SDIO = 1,
} sd_mode_t;

/*
 *  @crcEn
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_CRC_DI = 0,
    SD_CRC_EN,
} sd_crc_modes_t;

/*
 *  @transferMode
 *  Possible Modes for SD Communication
 */
typedef enum
{
    SD_TRANSFER_NON_DMA = 0,
    SD_TRANSFER_DMA,
} sd_trans_modes_t;

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
 *  Configuration structure for SD Card
 */
typedef struct SD_Handle_t
{
    sd_mode_t mode;                /*!  < possible values from @mode>          */
    sd_crc_modes_t crcEn;          /*!  < possible values from @crcEn>         */
    SD_States_t CardState;         /*!  < possible values from @CardState>     */
    sd_trans_modes_t transferMode; /*   < possible values from @transferMode   */
    bufferInfo_t bufferInfo;       /*   Buffer Info Structure                     */
    SPI_Handle_t SPI;              /*   SPI Handle Structure                      */
    GPIO_Handle_t chipSelect;      /*   Handler Chip select GPIOx                 */
    GPIO_Handle_t cardDetect;      /*   Handler Card Detect GPIOx                 */
    gpio_pin_state_t cardDetPol;   /*   SD Card detect polarity                   */
    Timeout_t cmdTimeout;          /*   Timer Handler for Command Timeouts        */
} SD_Handle_t;

/************************************************************************************
 *                          SD block size Definitions
 *************************************************************************************/
#define SD_DEFAULT_BLOCK_SIZE (512)

/************************************************************************************
 *                              Public Functions
 *************************************************************************************/

/* Init and De-init */
int SD_Init(SD_Handle_t* pSDHandle);

/*** Hardware Init Functions ***/
void SD_Init_Timers(SD_Handle_t* pSDHandle, TIM_RegDef_t* pTIMx, irq_no_t irqNo);
void SD_Init_Hardware(SD_Handle_t* pSDHandle, SPI_RegDef_t* pSPIx, DMA_Handle_t* pTxDma, DMA_Handle_t* pRxDma);

/* Read / Write function */
/* Returns 0 on success, negative errno on failure (-EINVAL, -ETIMEDOUT, or -EIO) */
int SD_ReadBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount);
int SD_WriteBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount);

/* Helper function */
SD_States_t SD_GetState(SD_Handle_t* pSDHandle);
uint8_t* SD_GetBuffAddr(SD_Handle_t* pSDHandle);
uint32_t SD_GetBuffSize(SD_Handle_t* pSDHandle);
void SD_ToggleCurrBuff(SD_Handle_t* pSDHandle);

/* IRQ Functions */
void SD_IRQHandling(SD_Handle_t* pSDHandle);

/************************************************************************************
 *			        		Externs for other files to use						*
 ************************************************************************************/

/* Handler for SD Card */
extern SD_Handle_t SD_Handle;

#endif /* INC_SD_CARD_H_ */
