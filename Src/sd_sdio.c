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

//-----------   Private function declarations    -----------//
static int init_card(SD_Handle_t* pSDHandle);
static int readBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount);
static int writeBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount);

// Populate the sd_ops interface for SPI opperation
const struct sd_ops sd_ops_sdio =
{
    .init = init_card,
    .ReadBlock = readBlock,
    .WriteBlock = writeBlock,
};

/****************************************************************************************
 *	@fn 			     - init_card
 *
 * 	@brief			     - Function to initialize SD Card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note
 */
int init_card(SD_Handle_t* pSDHandle)
{
    if (SD_IsCardPresent(pSDHandle) == -ENODEV)
    {
        pSDHandle->CardState = SD_STATE_NO_CARD;
        return -ENODEV;
    }

    // if (initSpi(pSDHandle) == INIT_SUCCESS)
    // {
    //     pSDHandle->CardState = SD_STATE_READY;
    //     return 0;
    // }

    pSDHandle->CardState = SD_STATE_FAIL;
    return -EIO;
}


/****************************************************************************************
 *	@fn 			     - readBlock
 *
 * 	@brief			     - Function to read block of data
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note				 -
 */
int readBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount)
{
    if (BlockCount == 0)
    {
        return -EINVAL;
    }

    return 0;
}

/****************************************************************************************
 *	@fn 			     - writeBlock
 *
 * 	@brief			     - Function to write block of data
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note				 -
 */
int writeBlock(SD_Handle_t* pSDHandle, uint8_t* pData, uint32_t BlockAddr, uint32_t BlockCount)
{
    if (BlockCount == 0)
    {
        return -EINVAL;
    }

    return 0;
}