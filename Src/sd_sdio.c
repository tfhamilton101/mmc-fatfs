/*
 * sd_sdio.c
 *
 *  Created on: July 10, 2026
 *      Author: thomashamilton
 */

#include "sd.h"
#include "sd_spec.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "stm32f4xx_dma_driver.h"
#include "stm32f4xx_nvic_driver.h"
#include "sd_ops.h"

/*
 *  Helper macro to get SDIO handle from SD_Handle_t
 */
#define SD_GET_SDIO_HANDLE(h) ((SDIO_Handle_t*)(h)->hwConfig.pHwHandle)

/****************************************************************************************
 *                              FIFO Helper Functions                                   *
 ****************************************************************************************/

/*
 *  @fn             - SDIO_ReadFIFO
 *
 *  @brief          - Read a 32-bit word from SDIO FIFO
 *
 *  @param[pHandle] - Pointer to SDIO handle
 *
 *  @return         - 32-bit data word from FIFO
 *
 *  @note           - Inline helper for cleaner code
 */
static inline uint32_t SDIO_ReadFIFO(SDIO_Handle_t* pHandle)
{
    return pHandle->pSDIOx->FIFO;
}

/*
 *  @fn             - SDIO_WriteFIFO
 *
 *  @brief          - Write a 32-bit word to SDIO FIFO
 *
 *  @param[pHandle] - Pointer to SDIO handle
 *  @param[data]    - 32-bit data word to write
 *
 *  @return         - None
 *
 *  @note           - Inline helper for cleaner code
 */
static inline void SDIO_WriteFIFO(SDIO_Handle_t* pHandle, uint32_t data)
{
    pHandle->pSDIOx->FIFO = data;
}

/*
 *  @fn             - SDIO_GetDataCounter
 *
 *  @brief          - Get remaining data count from SDIO DCOUNT register
 *
 *  @param[pHandle] - Pointer to SDIO handle
 *
 *  @return         - Number of bytes remaining to be transferred
 *
 *  @note           - Inline helper for cleaner code
 */
static inline uint32_t SDIO_GetDataCounter(SDIO_Handle_t* pHandle)
{
    return pHandle->pSDIOx->DCOUNT;
}

//-----------   Private function declarations    -----------//
static int init_card(SD_Handle_t* pSDHandle);
static int readBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount);
static int writeBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount);

//-----------   Response parsing helpers    -----------//
static Sdio_result_t SDIO_GetCmdError(SDIO_Handle_t* pSDIOHandle);
static Sdio_result_t SDIO_GetCmdResp1(SDIO_Handle_t* pSDIOHandle, uint8_t cmdIndex, SDIO_R1_Response_t* pResp);
static Sdio_result_t SDIO_GetCmdResp2(SDIO_Handle_t* pSDIOHandle, SDIO_R2_Response_t* pResp);
static Sdio_result_t SDIO_GetCmdResp3(SDIO_Handle_t* pSDIOHandle, SDIO_R3_Response_t* pResp);
static Sdio_result_t SDIO_GetCmdResp6(SDIO_Handle_t* pSDIOHandle, uint8_t cmdIndex, SDIO_R6_Response_t* pResp);
static Sdio_result_t SDIO_GetCmdResp7(SDIO_Handle_t* pSDIOHandle, SDIO_R7_Response_t* pResp);

//-----------   Command wrapper functions    -----------//
static Sdio_result_t CmdGoIdleState(SD_Handle_t* pSDHandle);
static Sdio_result_t CmdOperCond(SD_Handle_t* pSDHandle, SDIO_R7_Response_t* pResp);
static Sdio_result_t CmdAppCommand(SD_Handle_t* pSDHandle, uint16_t rca);
static Sdio_result_t CmdAppOperCommand(SD_Handle_t* pSDHandle, SDIO_R3_Response_t* pResp);
static Sdio_result_t CmdSendCID(SD_Handle_t* pSDHandle, SDIO_R2_Response_t* pResp);
static Sdio_result_t CmdSetRelAddr(SD_Handle_t* pSDHandle, uint16_t* pRCA);
static Sdio_result_t CmdSendCSD(SD_Handle_t* pSDHandle, uint16_t rca, SDIO_R2_Response_t* pResp);
static Sdio_result_t CmdSelDesel(SD_Handle_t* pSDHandle, uint16_t rca);
static Sdio_result_t CmdSendSCR(SD_Handle_t* pSDHandle, uint16_t rca, uint32_t* pSCR);
static Sdio_result_t CmdBusWidth(SD_Handle_t* pSDHandle, uint16_t rca, uint8_t busWidth);
static Sdio_result_t CmdSetBlockLen(SD_Handle_t* pSDHandle, uint32_t blockLen);
static Sdio_result_t CmdSendStatus(SD_Handle_t* pSDHandle, uint16_t rca, uint32_t* pCardStatus);
static Sdio_result_t CmdReadSingleBlock(SD_Handle_t* pSDHandle, uint32_t addr);
static Sdio_result_t CmdReadMultiBlock(SD_Handle_t* pSDHandle, uint32_t addr);
static Sdio_result_t CmdWriteSingleBlock(SD_Handle_t* pSDHandle, uint32_t addr);
static Sdio_result_t CmdWriteMultiBlock(SD_Handle_t* pSDHandle, uint32_t addr);
static Sdio_result_t CmdStopTransmission(SD_Handle_t* pSDHandle);

// Populate the sd_ops interface for SDIO operation
const struct sd_ops sd_ops_sdio =
{
    .init = init_card,
    .ReadBlock = readBlock,
    .WriteBlock = writeBlock,
};

/****************************************************************************************
 *	@fn 			     - init_card
 *
 * 	@brief			     - Function to initialize SD Card in SDIO mode
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note				 - Performs full card enumeration and configuration
 * 						 - Switches from 1-bit to 4-bit bus mode
 * 						 - Increases clock from 400kHz to 25MHz
 */
int init_card(SD_Handle_t* pSDHandle)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_R7_Response_t r7Resp = {0};
    SDIO_R3_Response_t r3Resp = {0};
    SDIO_R2_Response_t r2Resp = {0};
    uint16_t rca = 0;
    uint32_t scr[2] = {0};
    uint32_t attempts = 0;
    
    /* Check if card is present */
    if (SD_IsCardPresent(pSDHandle) == -ENODEV)
    {
        pSDHandle->CardState = SD_STATE_NO_CARD;
        return -ENODEV;
    }
    
    pSDHandle->CardState = SD_STATE_INIT;
    
    /* Configure SDIO peripheral for initialization */
    /* Note: pSDIOHandle->pSDIOx and pHwHandle are already set by SD_Init_Hardware_SDIO() */
    /* Configure for 400kHz initialization clock, 1-bit bus, no hardware flow control */
    pSDIOHandle->SDIOConfig.ClockDiv = 118;  /* 48MHz / (118+2) = 400kHz */
    pSDIOHandle->SDIOConfig.BusWidth = SDIO_BUS_WIDTH_1BIT;
    pSDIOHandle->SDIOConfig.ClockEdge = SDIO_CLK_EDGE_RISING;
    pSDIOHandle->SDIOConfig.ClockBypass = DISABLE;
    pSDIOHandle->SDIOConfig.PowerSave = DISABLE;
    pSDIOHandle->SDIOConfig.HardwareFlowControl = DISABLE;
    
    /* Configure DMA based on transfer mode */
    if (pSDHandle->transferMode == SD_TRANSFER_DMA)
    {
        pSDIOHandle->DMAConfig.RxBufDmaConfig = ENABLE;
        pSDIOHandle->DMAConfig.TxBufDmaConfig = ENABLE;
    }
    else
    {
        pSDIOHandle->DMAConfig.RxBufDmaConfig = DISABLE;
        pSDIOHandle->DMAConfig.TxBufDmaConfig = DISABLE;
    }
    
    /* Initialize SDIO peripheral */
    SDIO_Init(pSDIOHandle);
    
    /* Small delay after initialization */
    for (volatile uint32_t i = 0; i < 10000; i++);
    
    /* Step 1: CMD0 - Reset card to idle state */
    if (CmdGoIdleState(pSDHandle) != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Small delay after reset */
    for (volatile uint32_t i = 0; i < 10000; i++);
    
    /* Step 2: CMD8 - Check interface operating condition (SD v2.0+) */
    if (CmdOperCond(pSDHandle, &r7Resp) != SDIO_OK)
    {
        /* CMD8 failure indicates SD v1.x or MMC card - not supported */
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Verify voltage acceptance and check pattern echo */
    if (r7Resp.VoltageAccepted != 0x1 || r7Resp.CheckPattern != 0xAA)
    {
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Step 3: Loop ACMD41 until card initialization complete */
    /* OCR[31]=1 indicates power-up sequence has finished */
    attempts = 0;
    do
    {
        if (CmdAppOperCommand(pSDHandle, &r3Resp) != SDIO_OK)
        {
            pSDHandle->CardState = SD_STATE_FAIL;
            return -EIO;
        }
        
        /* Small delay between attempts */
        for (volatile uint32_t i = 0; i < 10000; i++);
        
        if (++attempts > 1000)
        {
            /* Timeout waiting for card ready */
            pSDHandle->CardState = SD_STATE_FAIL;
            return -ETIMEDOUT;
        }
        
    } while (!(r3Resp.OCR & (1 << OCR_PWR_UP_STATUS)));
    
    /* Check Card Capacity Status (CCS) bit - determines card type */
    /* CCS=1: SDHC/SDXC (block addressing), CCS=0: SDSC (byte addressing) */
    pSDHandle->cardInfo.cardType = (r3Resp.OCR & (1 << OCR_CCS)) ? 
                                    SD_CARDTYPE_SDXC_SDHC : SD_CARDTYPE_SDSC;
    
    /* Step 4: CMD2 - Get Card Identification (CID) */
    if (CmdSendCID(pSDHandle, &r2Resp) != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Step 5: CMD3 - Get Relative Card Address (RCA) */
    if (CmdSetRelAddr(pSDHandle, &rca) != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Store RCA for future commands */
    pSDHandle->cardInfo.RCA = rca;
    
    /* Step 6: CMD9 - Get Card-Specific Data (CSD) - optional but useful */
    if (CmdSendCSD(pSDHandle, rca, &r2Resp) != SDIO_OK)
    {
        /* Non-fatal - continue without CSD */
    }
    
    /* Step 7: CMD7 - Select card (move to Transfer state) */
    if (CmdSelDesel(pSDHandle, rca) != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_FAIL;
        return -EIO;
    }
    
    /* Step 8: ACMD51 - Read SD Configuration Register (SCR) */
    /* SCR contains bus width capabilities */
    if (CmdSendSCR(pSDHandle, rca, scr) != SDIO_OK)
    {
        /* Non-fatal - stay in 1-bit mode */
    }
    else
    {
        /* Step 9: ACMD6 - Switch to 4-bit bus mode */
        /* Check if 4-bit mode is supported (SCR bit 50 = SD_BUS_WIDTHS[0]) */
        if (scr[0] & 0x00040000)  /* 4-bit bus support */
        {
            if (CmdBusWidth(pSDHandle, rca, 4) == SDIO_OK)
            {
                /* Update SDIO peripheral bus width */
                SDIO_SetBusWidth(pSDIOHandle->pSDIOx, SDIO_BUS_WIDTH_4BIT);
                pSDIOHandle->SDIOConfig.BusWidth = SDIO_BUS_WIDTH_4BIT;
            }
        }
    }
    
    /* Step 10: Set block length for SDSC cards */
    if (pSDHandle->cardInfo.cardType == SD_CARDTYPE_SDSC)
    {
        if (CmdSetBlockLen(pSDHandle, SD_DEFAULT_BLOCK_SIZE) != SDIO_OK)
        {
            pSDHandle->CardState = SD_STATE_FAIL;
            return -EIO;
        }
    }
    /* SDHC/SDXC cards have fixed 512-byte blocks */
    
    /* Step 11: Increase clock frequency to 25MHz for data transfer */
    SDIO_SetClockFrequency(pSDIOHandle->pSDIOx, 25000000);
    
    /* Initialization complete */
    pSDHandle->CardState = SD_STATE_READY;
    
    return 0;
}


/****************************************************************************************
 *                         Data Transfer Command Wrappers                               *
 ****************************************************************************************/

/****************************************************************************************
 *	@fn 			     - CmdReadSingleBlock
 *
 * 	@brief			     - Send CMD17 to read a single block
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[addr]		 - Block address (block addressing for SDHC/SDXC, byte for SDSC)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Data path must be configured before calling
 */
static Sdio_result_t CmdReadSingleBlock(SD_Handle_t* pSDHandle, uint32_t addr)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD17: READ_SINGLE_BLOCK */
    cmd.Argument = addr;
    cmd.CmdIndex = CMD17;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD17, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdReadMultiBlock
 *
 * 	@brief			     - Send CMD18 to read multiple blocks
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[addr]		 - Starting block address
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must send CMD12 to stop transfer when done
 */
static Sdio_result_t CmdReadMultiBlock(SD_Handle_t* pSDHandle, uint32_t addr)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD18: READ_MULTIPLE_BLOCK */
    cmd.Argument = addr;
    cmd.CmdIndex = CMD18;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD18, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdWriteSingleBlock
 *
 * 	@brief			     - Send CMD24 to write a single block
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[addr]		 - Block address
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Data path must be configured before calling
 */
static Sdio_result_t CmdWriteSingleBlock(SD_Handle_t* pSDHandle, uint32_t addr)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD24: WRITE_BLOCK */
    cmd.Argument = addr;
    cmd.CmdIndex = CMD24;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD24, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdWriteMultiBlock
 *
 * 	@brief			     - Send CMD25 to write multiple blocks
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[addr]		 - Starting block address
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must send CMD12 to stop transfer when done
 */
static Sdio_result_t CmdWriteMultiBlock(SD_Handle_t* pSDHandle, uint32_t addr)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD25: WRITE_MULTIPLE_BLOCK */
    cmd.Argument = addr;
    cmd.CmdIndex = CMD25;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD25, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdStopTransmission
 *
 * 	@brief			     - Send CMD12 to stop multi-block transfer
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Used after CMD18 (read) or CMD25 (write)
 */
static Sdio_result_t CmdStopTransmission(SD_Handle_t* pSDHandle)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD12: STOP_TRANSMISSION */
    cmd.Argument = 0;
    cmd.CmdIndex = CMD12;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD12, &resp);
}

/****************************************************************************************
 *                              Data Transfer Functions                                 *
 ****************************************************************************************/

/****************************************************************************************
 *	@fn 			     - readBlock
 *
 * 	@brief			     - Function to read block(s) of data from SD card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[pData]		 - Pointer to data buffer
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note				 - Polling-based FIFO read implementation
 */
int readBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_DataInitTypeDef dataConfig = {0};
    uint32_t addr;
    uint32_t* pDataWord = (uint32_t*)pData;
    uint32_t totalBytes = BlockCount * SD_DEFAULT_BLOCK_SIZE;
    uint32_t wordsRemaining = totalBytes / 4;
    uint32_t timeout;
    Sdio_result_t cmdResult;
    
    /* Validate parameters */
    if (BlockCount == 0 || pData == NULL)
    {
        return -EINVAL;
    }
    
    /* Check card state */
    if (pSDHandle->CardState != SD_STATE_READY)
    {
        return -EIO;
    }
    
    pSDHandle->CardState = SD_STATE_BUSY;
    
    /* Convert address based on card type */
    /* SDHC/SDXC use block addressing, SDSC uses byte addressing */
    if (pSDHandle->cardInfo.cardType == SD_CARDTYPE_SDSC)
    {
        addr = BlockAddr * SD_DEFAULT_BLOCK_SIZE;  /* Byte address */
    }
    else
    {
        addr = BlockAddr;  /* Block address */
    }
    
    /* Configure data path */
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = totalBytes;
    dataConfig.DataBlockSize = SDIO_BLOCK_SIZE_512B;
    dataConfig.TransferDir = SDIO_TRANSFER_DIR_TO_CONTROLLER;
    dataConfig.TransferMode = DISABLE;  /* Block mode */
    dataConfig.DPSM_Enable = ENABLE;
    
    SDIO_ConfigureDataPath(pSDIOHandle, &dataConfig);
    
    /* Send read command */
    if (BlockCount == 1)
    {
        cmdResult = CmdReadSingleBlock(pSDHandle, addr);
    }
    else
    {
        cmdResult = CmdReadMultiBlock(pSDHandle, addr);
    }
    
    if (cmdResult != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_READY;
        return -EIO;
    }
    
    /* Poll FIFO and read data */
    timeout = 1000000;  /* Large timeout for read operation */
    
    while (wordsRemaining > 0)
    {
        /* Check for errors */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DTIMEOUTC);
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
        
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DCRCFAIL))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DCRCFAILC);
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
        
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_RXOVERR))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_RXOVERRC);
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
        
        /* Check if FIFO has data available */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_RXDAVL))
        {
            /* Read 32-bit word from FIFO */
            *pDataWord++ = SDIO_ReadFIFO(pSDIOHandle);
            wordsRemaining--;
            timeout = 1000000;  /* Reset timeout on successful read */
        }
        
        if (--timeout == 0)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
    }
    
    /* Wait for DATAEND flag */
    timeout = 100000;
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DATAEND)))
    {
        if (--timeout == 0)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
    }
    
    /* Clear data flags */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DATAENDC) | 
                                (1 << SDIO_ICR_DBCKENDC);
    
    /* Send STOP command for multi-block read */
    if (BlockCount > 1)
    {
        if (CmdStopTransmission(pSDHandle) != SDIO_OK)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
    }
    
    pSDHandle->CardState = SD_STATE_READY;
    return 0;
}

/****************************************************************************************
 *	@fn 			     - writeBlock
 *
 * 	@brief			     - Function to write block(s) of data to SD card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[pData]		 - Pointer to data buffer
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note				 - Polling-based FIFO write implementation
 * 						 - Polls card status after write until TRAN state
 */
int writeBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_DataInitTypeDef dataConfig = {0};
    uint32_t addr;
    uint32_t* pDataWord = (uint32_t*)pData;
    uint32_t totalBytes = BlockCount * SD_DEFAULT_BLOCK_SIZE;
    uint32_t wordsRemaining = totalBytes / 4;
    uint32_t timeout;
    uint32_t cardStatus;
    Sdio_result_t cmdResult;
    
    /* Validate parameters */
    if (BlockCount == 0 || pData == NULL)
    {
        return -EINVAL;
    }
    
    /* Check card state */
    if (pSDHandle->CardState != SD_STATE_READY)
    {
        return -EIO;
    }
    
    pSDHandle->CardState = SD_STATE_BUSY;
    
    /* Convert address based on card type */
    /* SDHC/SDXC use block addressing, SDSC uses byte addressing */
    if (pSDHandle->cardInfo.cardType == SD_CARDTYPE_SDSC)
    {
        addr = BlockAddr * SD_DEFAULT_BLOCK_SIZE;  /* Byte address */
    }
    else
    {
        addr = BlockAddr;  /* Block address */
    }
    
    /* Configure data path */
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = totalBytes;
    dataConfig.DataBlockSize = SDIO_BLOCK_SIZE_512B;
    dataConfig.TransferDir = SDIO_TRANSFER_DIR_TO_CARD;
    dataConfig.TransferMode = DISABLE;  /* Block mode */
    dataConfig.DPSM_Enable = ENABLE;
    
    SDIO_ConfigureDataPath(pSDIOHandle, &dataConfig);
    
    /* Send write command */
    if (BlockCount == 1)
    {
        cmdResult = CmdWriteSingleBlock(pSDHandle, addr);
    }
    else
    {
        cmdResult = CmdWriteMultiBlock(pSDHandle, addr);
    }
    
    if (cmdResult != SDIO_OK)
    {
        pSDHandle->CardState = SD_STATE_READY;
        return -EIO;
    }
    
    /* Poll FIFO and write data */
    timeout = 1000000;  /* Large timeout for write operation */
    
    while (wordsRemaining > 0)
    {
        /* Check for errors */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DTIMEOUTC);
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
        
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DCRCFAIL))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DCRCFAILC);
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
        
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_TXUNDERR))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_TXUNDERRC);
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
        
        /* Check if FIFO has space for more data */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_TXFIFOHE))
        {
            /* Write 32-bit word to FIFO */
            SDIO_WriteFIFO(pSDIOHandle, *pDataWord++);
            wordsRemaining--;
            timeout = 1000000;  /* Reset timeout on successful write */
        }
        
        if (--timeout == 0)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
    }
    
    /* Wait for DATAEND flag */
    timeout = 100000;
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DATAEND)))
    {
        if (--timeout == 0)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
    }
    
    /* Clear data flags */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DATAENDC) | 
                                (1 << SDIO_ICR_DBCKENDC);
    
    /* Send STOP command for multi-block write */
    if (BlockCount > 1)
    {
        if (CmdStopTransmission(pSDHandle) != SDIO_OK)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
    }
    
    /* Poll card status until it returns to TRAN state (not BUSY) */
    /* Card needs time to program the written data to flash */
    timeout = 100000;
    do
    {
        if (CmdSendStatus(pSDHandle, pSDHandle->cardInfo.RCA, &cardStatus) != SDIO_OK)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -EIO;
        }
        
        /* Check if card is in TRAN state and ready for data */
        if ((GET_CARD_STATE(cardStatus) == CARD_STATE_TRAN) && 
            (cardStatus & (1 << CARD_STATUS_READY_FOR_DATA)))
        {
            break;  /* Card is ready */
        }
        
        /* Small delay between status polls */
        for (volatile uint32_t i = 0; i < 100; i++);
        
        if (--timeout == 0)
        {
            pSDHandle->CardState = SD_STATE_READY;
            return -ETIMEDOUT;
        }
        
    } while (1);
    
    pSDHandle->CardState = SD_STATE_READY;
    return 0;
}

/****************************************************************************************
 *                         Response Parsing Helper Functions                            *
 ****************************************************************************************/

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdError
 *
 * 	@brief			     - Check for command path errors (no response expected)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 *
 * 	@return			     - SDIO_OK, SDIO_ERROR_TIMEOUT, or SDIO_ERROR_CRC
 *
 * 	@note				 - Used for commands that expect no response (CMD0)
 */
static Sdio_result_t SDIO_GetCmdError(SDIO_Handle_t* pSDIOHandle)
{
    uint32_t timeout = 500000;
    
    /* Wait for CMDSENT flag */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDSENT)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
    }
    
    /* Clear CMDSENT flag */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDSENTC);
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdResp1
 *
 * 	@brief			     - Parse R1 response (short response with card status)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 * 	@param[cmdIndex]	 - Expected command index
 * 	@param[pResp]		 - Pointer to response structure to fill
 *
 * 	@return			     - SDIO_OK, SDIO_ERROR_TIMEOUT, or SDIO_ERROR_CRC
 *
 * 	@note				 - Used for most commands (CMD3, CMD7, CMD13, etc.)
 */
static Sdio_result_t SDIO_GetCmdResp1(SDIO_Handle_t* pSDIOHandle, uint8_t cmdIndex, SDIO_R1_Response_t* pResp)
{
    uint32_t timeout = 500000;
    uint32_t response;
    
    /* Wait for CMDREND flag (command response received) */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDREND)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for CRC error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CCRCFAIL))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CCRCFAILC);
            return SDIO_ERROR_CRC;
        }
    }
    
    /* Verify command index matches */
    if ((pSDIOHandle->pSDIOx->RESPCMD & 0x3F) != cmdIndex)
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    /* Read response (card status is in RESP1) */
    response = pSDIOHandle->pSDIOx->RESP1;
    
    /* Clear CMDREND flag */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDRENDC);
    
    /* Fill response structure */
    pResp->CardStatus = response;
    
    /* Check for card status errors */
    if (response & CARD_STATUS_ERROR_MASK)
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdResp2
 *
 * 	@brief			     - Parse R2 response (long response with CID/CSD)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 * 	@param[pResp]		 - Pointer to response structure to fill
 *
 * 	@return			     - SDIO_OK, SDIO_ERROR_TIMEOUT, or SDIO_ERROR_CRC
 *
 * 	@note				 - Used for CMD2 (CID) and CMD9 (CSD)
 * 						 - CRC check is disabled for R2 responses per SDIO spec
 */
static Sdio_result_t SDIO_GetCmdResp2(SDIO_Handle_t* pSDIOHandle, SDIO_R2_Response_t* pResp)
{
    uint32_t timeout = 500000;
    
    /* Wait for CMDREND flag */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDREND)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Note: CRC check is disabled for R2, so we don't check CCRCFAIL */
    }
    
    /* Read all four response registers (128-bit CID/CSD) */
    pResp->Data[0] = pSDIOHandle->pSDIOx->RESP1;
    pResp->Data[1] = pSDIOHandle->pSDIOx->RESP2;
    pResp->Data[2] = pSDIOHandle->pSDIOx->RESP3;
    pResp->Data[3] = pSDIOHandle->pSDIOx->RESP4;
    
    /* Clear CMDREND flag */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDRENDC);
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdResp3
 *
 * 	@brief			     - Parse R3 response (short response with OCR)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 * 	@param[pResp]		 - Pointer to response structure to fill
 *
 * 	@return			     - SDIO_OK or SDIO_ERROR_TIMEOUT
 *
 * 	@note				 - Used for ACMD41 (SD_APP_OP_COND)
 * 						 - No CRC in R3 response per SD spec
 */
static Sdio_result_t SDIO_GetCmdResp3(SDIO_Handle_t* pSDIOHandle, SDIO_R3_Response_t* pResp)
{
    uint32_t timeout = 500000;
    
    /* Wait for CMDREND flag */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDREND)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
    }
    
    /* Read response (OCR is in RESP1) */
    pResp->OCR = pSDIOHandle->pSDIOx->RESP1;
    
    /* Clear CMDREND and CCRCFAIL flags (CRC error is normal for R3) */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDRENDC) | (1 << SDIO_ICR_CCRCFAILC);
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdResp6
 *
 * 	@brief			     - Parse R6 response (short response with RCA)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 * 	@param[cmdIndex]	 - Expected command index
 * 	@param[pResp]		 - Pointer to response structure to fill
 *
 * 	@return			     - SDIO_OK, SDIO_ERROR_TIMEOUT, or SDIO_ERROR_CRC
 *
 * 	@note				 - Used for CMD3 (SEND_RELATIVE_ADDR)
 */
static Sdio_result_t SDIO_GetCmdResp6(SDIO_Handle_t* pSDIOHandle, uint8_t cmdIndex, SDIO_R6_Response_t* pResp)
{
    uint32_t timeout = 500000;
    uint32_t response;
    
    /* Wait for CMDREND flag */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDREND)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for CRC error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CCRCFAIL))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CCRCFAILC);
            return SDIO_ERROR_CRC;
        }
    }
    
    /* Verify command index matches */
    if ((pSDIOHandle->pSDIOx->RESPCMD & 0x3F) != cmdIndex)
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    /* Read response */
    response = pSDIOHandle->pSDIOx->RESP1;
    
    /* Clear CMDREND flag */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDRENDC);
    
    /* Parse R6 response: [31:16] RCA, [15:0] Card status bits */
    pResp->RCA = (uint16_t)(response >> 16);
    pResp->CardStatus = (uint16_t)(response & 0xFFFF);
    
    /* Check for card status errors (limited set in R6) */
    /* Bits: 13=ERROR, 12=ILLEGAL_CMD, 11=COM_CRC_ERROR */
    if (pResp->CardStatus & 0x3800)
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - SDIO_GetCmdResp7
 *
 * 	@brief			     - Parse R7 response (interface condition response)
 *
 * 	@param[pSDIOHandle]	 - Pointer to SDIO handle
 * 	@param[pResp]		 - Pointer to response structure to fill
 *
 * 	@return			     - SDIO_OK, SDIO_ERROR_TIMEOUT, or SDIO_ERROR_CRC
 *
 * 	@note				 - Used for CMD8 (SEND_IF_COND)
 */
static Sdio_result_t SDIO_GetCmdResp7(SDIO_Handle_t* pSDIOHandle, SDIO_R7_Response_t* pResp)
{
    uint32_t timeout = 500000;
    uint32_t response;
    
    /* Wait for CMDREND flag */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CMDREND)))
    {
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for command timeout error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CTIMEOUT))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CTIMEOUTC);
            return SDIO_ERROR_TIMEOUT;
        }
        
        /* Check for CRC error */
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_CCRCFAIL))
        {
            pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CCRCFAILC);
            return SDIO_ERROR_CRC;
        }
    }
    
    /* Read response */
    response = pSDIOHandle->pSDIOx->RESP1;
    
    /* Clear CMDREND flag */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_CMDRENDC);
    
    /* Parse R7 response: [19:16] Voltage accepted, [7:0] Check pattern */
    pResp->VoltageAccepted = (uint8_t)((response >> 16) & 0x0F);
    pResp->CheckPattern = (uint8_t)(response & 0xFF);
    
    return SDIO_OK;
}

/****************************************************************************************
 *                         SD Command Wrapper Functions                                 *
 ****************************************************************************************/

/****************************************************************************************
 *	@fn 			     - CmdGoIdleState
 *
 * 	@brief			     - Send CMD0 to reset card to idle state
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - No response expected (broadcast command)
 */
static Sdio_result_t CmdGoIdleState(SD_Handle_t* pSDHandle)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    
    /* Configure CMD0: GO_IDLE_STATE */
    cmd.Argument = 0;
    cmd.CmdIndex = CMD0;
    cmd.Response = SDIO_RESPONSE_NONE;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Check for errors (no response expected) */
    return SDIO_GetCmdError(pSDIOHandle);
}

/****************************************************************************************
 *	@fn 			     - CmdOperCond
 *
 * 	@brief			     - Send CMD8 to check interface operating condition
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[pResp]		 - Pointer to R7 response structure
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - R7 response contains voltage acceptance and check pattern
 */
static Sdio_result_t CmdOperCond(SD_Handle_t* pSDHandle, SDIO_R7_Response_t* pResp)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    
    /* Configure CMD8: SEND_IF_COND */
    cmd.Argument = CMD8_ARG;  /* Voltage: 2.7-3.6V, Check pattern: 0xAA */
    cmd.CmdIndex = CMD8;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R7 response */
    return SDIO_GetCmdResp7(pSDIOHandle, pResp);
}

/****************************************************************************************
 *	@fn 			     - CmdAppCommand
 *
 * 	@brief			     - Send CMD55 to indicate next command is application-specific
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address (0 during initialization)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must be sent before any ACMD command
 */
static Sdio_result_t CmdAppCommand(SD_Handle_t* pSDHandle, uint16_t rca)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD55: APP_CMD */
    cmd.Argument = (uint32_t)rca << 16;  /* RCA in upper 16 bits */
    cmd.CmdIndex = CMD55;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    Sdio_result_t result = SDIO_GetCmdResp1(pSDIOHandle, CMD55, &resp);
    if (result != SDIO_OK)
    {
        return result;
    }
    
    /* Verify APP_CMD bit is set in card status */
    if (!(resp.CardStatus & (1 << CARD_STATUS_APP_CMD)))
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - CmdAppOperCommand
 *
 * 	@brief			     - Send ACMD41 to complete card initialization
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[pResp]		 - Pointer to R3 response structure (OCR)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must be preceded by CMD55
 * 						 - Loop until OCR[31]=1 (power-up complete)
 */
static Sdio_result_t CmdAppOperCommand(SD_Handle_t* pSDHandle, SDIO_R3_Response_t* pResp)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    
    /* Send CMD55 first */
    if (CmdAppCommand(pSDHandle, 0) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Configure ACMD41: SD_APP_OP_COND */
    cmd.Argument = ACMD41_ARG;  /* HCS=1 (supports SDHC/SDXC) */
    cmd.CmdIndex = ACMD41;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R3 response (no CRC check) */
    return SDIO_GetCmdResp3(pSDIOHandle, pResp);
}

/****************************************************************************************
 *	@fn 			     - CmdSendCID
 *
 * 	@brief			     - Send CMD2 to request Card Identification
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[pResp]		 - Pointer to R2 response structure (CID)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Broadcast command, card sends 128-bit CID register
 */
static Sdio_result_t CmdSendCID(SD_Handle_t* pSDHandle, SDIO_R2_Response_t* pResp)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    
    /* Configure CMD2: ALL_SEND_CID */
    cmd.Argument = 0;
    cmd.CmdIndex = CMD2;
    cmd.Response = SDIO_RESPONSE_LONG;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R2 response (CID) */
    return SDIO_GetCmdResp2(pSDIOHandle, pResp);
}

/****************************************************************************************
 *	@fn 			     - CmdSetRelAddr
 *
 * 	@brief			     - Send CMD3 to request Relative Card Address
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[pRCA]		 - Pointer to store the assigned RCA
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Card publishes its RCA in response
 */
static Sdio_result_t CmdSetRelAddr(SD_Handle_t* pSDHandle, uint16_t* pRCA)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R6_Response_t resp = {0};
    
    /* Configure CMD3: SEND_RELATIVE_ADDR */
    cmd.Argument = 0;
    cmd.CmdIndex = CMD3;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R6 response */
    Sdio_result_t result = SDIO_GetCmdResp6(pSDIOHandle, CMD3, &resp);
    if (result != SDIO_OK)
    {
        return result;
    }
    
    /* Store RCA */
    *pRCA = resp.RCA;
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - CmdSendCSD
 *
 * 	@brief			     - Send CMD9 to request Card-Specific Data
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address
 * 	@param[pResp]		 - Pointer to R2 response structure (CSD)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Card sends 128-bit CSD register
 */
static Sdio_result_t CmdSendCSD(SD_Handle_t* pSDHandle, uint16_t rca, SDIO_R2_Response_t* pResp)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    
    /* Configure CMD9: SEND_CSD */
    cmd.Argument = (uint32_t)rca << 16;
    cmd.CmdIndex = CMD9;
    cmd.Response = SDIO_RESPONSE_LONG;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R2 response (CSD) */
    return SDIO_GetCmdResp2(pSDIOHandle, pResp);
}

/****************************************************************************************
 *	@fn 			     - CmdSelDesel
 *
 * 	@brief			     - Send CMD7 to select/deselect card
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address (0 to deselect all)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Puts card in Transfer state for data operations
 */
static Sdio_result_t CmdSelDesel(SD_Handle_t* pSDHandle, uint16_t rca)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD7: SELECT_CARD */
    cmd.Argument = (uint32_t)rca << 16;
    cmd.CmdIndex = CMD7;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD7, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdSendSCR
 *
 * 	@brief			     - Send ACMD51 to read SD Configuration Register
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address
 * 	@param[pSCR]		 - Pointer to store 64-bit SCR data
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must be preceded by CMD55
 * 						 - SCR contains bus width capabilities
 */
static Sdio_result_t CmdSendSCR(SD_Handle_t* pSDHandle, uint16_t rca, uint32_t* pSCR)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_DataInitTypeDef dataConfig = {0};
    SDIO_R1_Response_t resp = {0};
    uint32_t timeout = 100000;
    
    /* Send CMD55 first */
    if (CmdAppCommand(pSDHandle, rca) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Configure data path for 8-byte SCR read */
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = 8;
    dataConfig.DataBlockSize = SDIO_BLOCK_SIZE_8B;
    dataConfig.TransferDir = SDIO_TRANSFER_DIR_TO_CONTROLLER;
    dataConfig.TransferMode = DISABLE;  /* Block mode */
    dataConfig.DPSM_Enable = ENABLE;
    
    SDIO_ConfigureDataPath(pSDIOHandle, &dataConfig);
    
    /* Configure ACMD51: SEND_SCR */
    cmd.Argument = 0;
    cmd.CmdIndex = ACMD51;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    if (SDIO_GetCmdResp1(pSDIOHandle, ACMD51, &resp) != SDIO_OK)
    {
        return SDIO_ERROR_HARDWARE;
    }
    
    /* Read SCR data from FIFO (8 bytes = 2 x 32-bit words) */
    while (!(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_RXOVERR)) &&
           !(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DCRCFAIL)) &&
           !(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DTIMEOUT)) &&
           !(pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DATAEND)))
    {
        if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_RXDAVL))
        {
            *pSCR++ = pSDIOHandle->pSDIOx->FIFO;
            *pSCR++ = pSDIOHandle->pSDIOx->FIFO;
        }
        
        if (--timeout == 0)
        {
            return SDIO_ERROR_TIMEOUT;
        }
    }
    
    /* Check for data errors */
    if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DTIMEOUT))
    {
        pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DTIMEOUTC);
        return SDIO_ERROR_TIMEOUT;
    }
    
    if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_DCRCFAIL))
    {
        pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DCRCFAILC);
        return SDIO_ERROR_CRC;
    }
    
    if (pSDIOHandle->pSDIOx->STA & (1 << SDIO_STA_RXOVERR))
    {
        pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_RXOVERRC);
        return SDIO_ERROR_HARDWARE;
    }
    
    /* Clear data flags */
    pSDIOHandle->pSDIOx->ICR = (1 << SDIO_ICR_DATAENDC);
    
    return SDIO_OK;
}

/****************************************************************************************
 *	@fn 			     - CmdBusWidth
 *
 * 	@brief			     - Send ACMD6 to set bus width
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address
 * 	@param[busWidth]	 - Bus width: 0=1-bit, 2=4-bit
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Must be preceded by CMD55
 * 						 - Must update SDIO peripheral bus width after success
 */
static Sdio_result_t CmdBusWidth(SD_Handle_t* pSDHandle, uint16_t rca, uint8_t busWidth)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Send CMD55 first */
    if (CmdAppCommand(pSDHandle, rca) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Configure ACMD6: SET_BUS_WIDTH */
    cmd.Argument = (busWidth == 4) ? ACMD6_ARG_4BIT : ACMD6_ARG_1BIT;
    cmd.CmdIndex = ACMD6;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, ACMD6, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdSetBlockLen
 *
 * 	@brief			     - Send CMD16 to set block length
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[blockLen]	 - Block length in bytes (typically 512)
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Only needed for SDSC cards (SDHC/SDXC fixed at 512)
 */
static Sdio_result_t CmdSetBlockLen(SD_Handle_t* pSDHandle, uint32_t blockLen)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD16: SET_BLOCKLEN */
    cmd.Argument = blockLen;
    cmd.CmdIndex = CMD16;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    return SDIO_GetCmdResp1(pSDIOHandle, CMD16, &resp);
}

/****************************************************************************************
 *	@fn 			     - CmdSendStatus
 *
 * 	@brief			     - Send CMD13 to get card status
 *
 * 	@param[pSDHandle]	 - Pointer to SD handle
 * 	@param[rca]			 - Relative Card Address
 * 	@param[pCardStatus]	 - Pointer to store 32-bit card status
 *
 * 	@return			     - SDIO_OK or error code
 *
 * 	@note				 - Used to poll card state after write operations
 */
static Sdio_result_t CmdSendStatus(SD_Handle_t* pSDHandle, uint16_t rca, uint32_t* pCardStatus)
{
    SDIO_Handle_t* pSDIOHandle = SD_GET_SDIO_HANDLE(pSDHandle);
    SDIO_CmdInitTypeDef cmd = {0};
    SDIO_R1_Response_t resp = {0};
    
    /* Configure CMD13: SEND_STATUS */
    cmd.Argument = (uint32_t)rca << 16;
    cmd.CmdIndex = CMD13;
    cmd.Response = SDIO_RESPONSE_SHORT;
    cmd.WaitForInterrupt = DISABLE;
    cmd.CPSM_Enable = ENABLE;
    
    /* Send command */
    if (SDIO_SendCommand(pSDIOHandle, &cmd) != SDIO_OK)
    {
        return SDIO_ERROR_TIMEOUT;
    }
    
    /* Get R1 response */
    Sdio_result_t result = SDIO_GetCmdResp1(pSDIOHandle, CMD13, &resp);
    if (result != SDIO_OK)
    {
        return result;
    }
    
    /* Store card status */
    *pCardStatus = resp.CardStatus;
    
    return SDIO_OK;
}

/****************************************************************************************
 *                              Card Initialization Functions                           *
 ****************************************************************************************/

/*
    For the SD card, the identification process starts at clock rate Fod, and the SDIO_CMD line
    output drives are push-pull drivers instead of open-drain. The registration process is
    accomplished as follows:

    1.The bus is activated.
    2.The SDIO card host broadcasts SD_APP_OP_COND (ACMD41).
    3.The cards respond with the contents of their operation condition registers.
    4.The incompatible cards are placed in the inactive state.
    5.The SDIO card host broadcasts ALL_SEND_CID (CMD2) to all active cards.
    6.The cards send back their unique card identification numbers (CIDs) and enter the
        Identification state.
    7.The SDIO card host issues SET_RELATIVE_ADDR (CMD3) to an active card with an
        address. This new address is called the relative card address (RCA); it is shorter than
        the CID and addresses the card. The assigned card changes to the Standby state. The
        SDIO card host can reissue this command to change the RCA. The RCA of the card is
        the last assigned value.
    8.The SDIO card host repeats steps 5 through 7 with all active cards.
*/