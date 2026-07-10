/*
 * sd.c
 *
 *  Created on: Sep 13, 2020
 *      Author: thomashamilton
 */

#include <errno.h>
#include "sd.h"
#include "sd_ops.h"
#include "sd_sdio.h"
#include "sd_spi.h"

/************************************************************************************
 *			        		SD Card Detection
 *************************************************************************************/
typedef enum
{
    CD_REMOVED,
    CD_DETECTED,
} card_detect_t;

/* I/O Functions */
static card_detect_t getCdStatus(SD_Handle_t* pSDHandle);
static const struct sd_ops* getSdOps(SD_Handle_t* pSDHandle);

static const struct sd_ops* getSdOps(SD_Handle_t* pSDHandle)
{
    if (pSDHandle->mode == SD_MODE_SDIO)
    {
        return &sd_ops_sdio;
    }

    return &sd_ops_spi;
}


int SD_Init(SD_Handle_t* pSDHandle)
{
    const struct sd_ops* sd_ops = getSdOps(pSDHandle);
    return sd_ops->init(pSDHandle);
}

int SD_ReadBlock(SD_Handle_t* pSDHandle, uint8_t *pData, uint32_t addr, uint32_t count)
{
    const struct sd_ops* sd_ops = getSdOps(pSDHandle);
    return sd_ops->ReadBlock(pSDHandle, pData, addr, count);
}

int SD_WriteBlock(SD_Handle_t* pSDHandle, uint8_t *pData, uint32_t addr, uint32_t count)
{
    const struct sd_ops* sd_ops = getSdOps(pSDHandle);
    return sd_ops->WriteBlock(pSDHandle, pData, addr, count);
}

/****************************************************************************************
 *  @fn                - getCdStatus
 *
 *  @brief             - Read Card Detect Input Pin
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *
 *  @return            -  CD_DETECTED or CD_REMOVED
 *
 *  @note              - 
 */
static card_detect_t getCdStatus(SD_Handle_t* pSDHandle)
{
    GPIO_Handle_t cd = pSDHandle->hwConfig.cardDetect;

    // Active High Switch
    if (GPIO_ReadFromInputPin(cd.pGPIOx, cd.GPIO_PinConfig.GPIO_PinNumber) == pSDHandle->hwConfig.cardDetPol)
    {
        return CD_DETECTED;
    }

    return CD_REMOVED;
}

/*******************       IRQ Handling and callback       *******************/

/****************************************************************************************
 *  @fn                 - SD_IRQHandling
 *
 *  @brief              - Handle Card Detect GPIO IRQ Events
 *
 *  @param[pSDHandle]   - Handler structure for SD Card
 *
 *  @return             - None
 *
 *  @note               -
 */
void SD_IRQHandling(SD_Handle_t* pSDHandle)
{
    if (getCdStatus(pSDHandle) == CD_REMOVED)
    {
        pSDHandle->CardState = SD_STATE_NO_CARD;
    }
}
