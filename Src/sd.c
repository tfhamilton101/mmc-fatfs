/*
 * sd.c
 *
 *  Created on: Sep 13, 2020
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
    GPIO_Handle_t cd = pSDHandle->cardDetect;

    // Active High Switch
    if (GPIO_ReadFromInputPin(cd.pGPIOx, cd.GPIO_PinConfig.GPIO_PinNumber) == pSDHandle->cardDetPol)
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
