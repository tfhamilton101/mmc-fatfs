/*
 * SD_Card.c
 *
 *  Created on: Sep 13, 2020
 *      Author: thomashamilton
 */

#include "SD_Card.h"

#include <stdlib.h>
#include <string.h>
#include "stm32f407xx_dma_driver.h"

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
 *                              SD Init States                                      
 *************************************************************************************/
typedef enum
{
    CMD0_FAIL = 0,
    CMD8_FAIL,
    INIT_FAIL,
    INIT_SUCCESS,
} SD_Init_States_t;


//-----------   Private function declarations    -----------//
static SD_Init_States_t SD_Init_SPI(SD_Handle_t* pSDHandle);
static void SD_ChipSelectControl(SD_Handle_t* pSDHandle, gpio_pin_state_t state);
static Command_Response_t SD_GetResponse(SD_Handle_t* pSDHandle, sd_response_t Format);
static Command_Response_t SD_ParseResponse(uint8_t* Response, sd_response_t Format);
static void SD_TimeoutConfig(SD_Handle_t* pSDHandle, EnOrDi_t EnOrDi);
static void SD_WaitBusyState(SD_Handle_t* pSDHandle, uint8_t* response);

/********		 SD Command Functions 		********/
static Command_Response_t SD_Command(SD_Handle_t* pSDHandle, sd_cmd_ID_t cmdID, uint32_t argument, sd_cmd_crc_t crc);
static Command_Response_t SD_App_Command(SD_Handle_t* pSDHandle, sd_cmd_ID_t cmdID, uint32_t argument, sd_cmd_crc_t crc);

/********		 SD Initialization Functions 		********/
static void SD_PowerSequence(SD_Handle_t* pSDHandle);
static Command_Response_t SD_GoIdleState(SD_Handle_t* pSDHandle);
static Command_Response_t SD_CheckSupplyRange(SD_Handle_t* pSDHandle);
static Command_Response_t SD_SendOpCond(SD_Handle_t* pSDHandle);
static Command_Response_t SD_ReadOcrRegister(SD_Handle_t* pSDHandle);
static Command_Response_t SD_SetBlockLength(SD_Handle_t* pSDHandle);
static timeout_status_t SD_GetTimeoutStatus(SD_Handle_t* pSDHandle);

/****************************************************************************************
 *	@fn 			     - SD_Init_Hardware
 *
 * 	@brief			     - Function to initialize SPI peripheral
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note
 */
void SD_Init_Hardware(SD_Handle_t* pSDHandle, sd_hardware_type_t type)
{
    if (type == SD_HARDWARE_SPI)
    {
        // Note: SD initialization must run at 100-400kHz
        SPI_Handle_t SPIx_handle;

        SPIx_handle.pSPIx = pSDHandle->pSPIx;
        SPIx_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
        SPIx_handle.SPIConfig.SPI_BusConfig = SPI_BUS_CON_FD;
        // Set the SPI clock to 312khz (20MHz/64)
        SPIx_handle.SPIConfig.SPI_SclkDiv = SPI_FindClockDiv(pSDHandle->pSPIx, 400000);
        SPIx_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
        SPIx_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
        SPIx_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
        SPIx_handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

        // DMA Config
        SPIx_handle.DMAConfig.RxBufDmaConfig = DISABLE;
        SPIx_handle.DMAConfig.TxBufDmaConfig = DISABLE;

        // Configure the peripheral
        SPI_Init(&SPIx_handle);
    }
    else if (type == SD_HARDWARE_SDIO)
    {
        // TODO: Write SDIO drivers
    }
}

/****************************************************************************************
 *  @fn                 - SD_Init_Timers
 *
 *  @brief              - This function initializes TIM for SD command timeouts
 *
 *  @param[pSDHandle]   - Handler structure for SD Card
 *  @param[pTIMx]       - base address of the TIM peripheral
 *  @param[irqNo]       - IRQ number for the TIM peripheral
 * 
 *  @return             - none
 *
 *  @note               - 
 */
void SD_Init_Timers(SD_Handle_t* pSDHandle, TIM_RegDef_t* pTIMx, irq_no_t irqNo)
{
    TIM_Handle_t TIMHandle = {0};

    // Configure Timer
    TIMHandle.pTIMx = pTIMx;
    TIMHandle.TIMConfig.TIM_Auto_Reload = TIM_AUTO_RELOAD_DI;
    TIMHandle.TIMConfig.TIM_MMS = TIM_MMS_RESET;
    TIMHandle.TIMConfig.TIM_Update_Disable = TIM_UPDATE_ENABLE;
    TIMHandle.TIMConfig.TIM_Update_Req_Source = TIM_UP_REQEST_SRCE_OVERFLOW;
    TIMHandle.TIMConfig.TIM_Type = TIM_TYPE_BASIC;
    TIMHandle.TIMConfig.TIM_Mode = TIM_MODE_BASE_GENERATION;

    // Store in the SD Handler
    pSDHandle->cmdTimeout.TimHandle = TIMHandle;

    // Configure TIM
    TIM_Init(&TIMHandle);

    // Set Auto reload value & Prescaler
    TIM_Set_AutoReload(pTIMx, 65535);
    // Fixed 100ms timeout for now.
    TIM_Set_PreScaler(pTIMx, SD_TIMEOUT_PRESCALE);

    // Enable TIMx interrupt
    TIM_DIERConfig(pTIMx, TIM_DIER_UIE, ENABLE);

    // Configure TIM priority
    TIM_IRQPriorityConfig(irqNo, NVIC_IRQ_PRI12);

    // Configure TIM IRQ
    TIM_IRQInterruptConfig(irqNo, ENABLE);
}

/****************************************************************************************
 *	@fn 			     - SD_Init
 *
 * 	@brief			     - Function to initialize SD Card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note
 */
void SD_Init(SD_Handle_t* pSDHandle)
{
    if (pSDHandle->SD_Mode == SD_MODE_SPI)
    {
        if (SD_GetCDStatus(pSDHandle) == CD_REMOVED)
        {
            pSDHandle->SD_CardState = SD_STATE_NO_CARD;
        }
        else if (SD_Init_SPI(pSDHandle) == INIT_SUCCESS)
        {
            pSDHandle->SD_CardState = SD_STATE_READY;
        }
        else
        {
            pSDHandle->SD_CardState = SD_STATE_FAIL;
        }
    }
    else if (pSDHandle->SD_Mode == SD_MODE_SDIO)
    {
        //TODO: Write SDIO driver
    }
}

/****************************************************************************************
 *	@fn 			     - SD_Init_SPI
 *
 * 	@brief			     - Function to initialize SD Card in SPI mode
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note
 */
static SD_Init_States_t SD_Init_SPI(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    // Reduce SPI Clock frequency incase this is an re-init
    SPI_UpdateClockFreq(pSDHandle->pSPIx, 400000);

    // Enable the SPI Peripheral
    SPI_PeripheralControl(pSDHandle->pSPIx, ENABLE);

    /*********      Send Power Sequence    ********/
    SD_PowerSequence(pSDHandle);

    // Assert chip select low
    SD_ChipSelectControl(pSDHandle, LOW);

    /*********     	  Go Idle State       *********/
    /* Instruct the card to use the SPI interface */
    uint8_t maxResetAttemps = CMD0_MAX_ATTEMPTS;
    do
    {
        CmdResponse = SD_GoIdleState(pSDHandle);
        maxResetAttemps--;
    } while ((CmdResponse.R1.Flags != R1_IDLE) && maxResetAttemps != 0);

    if (maxResetAttemps == 0)
    {
        // Assert chip select HIGH
        SD_ChipSelectControl(pSDHandle, HIGH);
        return CMD0_FAIL;
    }

    /*********     Check Supply range (SEND_IF_COND) 	 *********/
    /*  Verify the SD Memory Card Interface operating condition  */
    CmdResponse = SD_CheckSupplyRange(pSDHandle);

    if ((CmdResponse.R7.Check_Pattern != 0xAA) && (CmdResponse.R7.Voltage_Accepted != 0x1))
    {
        // Assert chip select HIGH
        SD_ChipSelectControl(pSDHandle, HIGH);
        return CMD8_FAIL;
    }

    /******		 Initiate initialization process 		******/
    /**   	  Check command until initialization has completed  **/
    // TODO: Implement Error check or timeout
    do
    {
        CmdResponse = SD_SendOpCond(pSDHandle);
    } while (CmdResponse.R1.Flags == R1_IDLE);

    /*****		 Read OCR Register 		*****/
    CmdResponse = SD_ReadOcrRegister(pSDHandle);

    bool pwrUpStatus = (CmdResponse.R3.OCR >> OCR_PWR_UP_STATUS) & 0x1;
    
    /* Check if the power up procedure has finished */
    if (!pwrUpStatus)
    {
        // Assert chip select HIGH
        SD_ChipSelectControl(pSDHandle, HIGH);
        return INIT_FAIL;
    }

    pSDHandle->SD_CardType = ((CmdResponse.R3.OCR >> OCR_CCS) & 0x1);

    if (pSDHandle->SD_CardType == SD_CARDTYPE_SDSC)
    {
        SD_SetBlockLength(pSDHandle);
    }
    // Assert chip select HIGH
    SD_ChipSelectControl(pSDHandle, HIGH);

    // Update SPI Clock frequency for higher performance
    SPI_UpdateClockFreq(pSDHandle->pSPIx, 50000000);
    
    return INIT_SUCCESS;
}

/****************************************************************************************
 *	@fn 			     - SD_Command
 *
 * 	@brief			     - Send SPI SD Command
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[Cmd]	 		 - Command Index
 * 	@param[Argument]	 - Command Argument
 * 	@param[Crc]	 		 - Crc for command. Most Commonly zero, otherwise Macro Hard-coded
 *
 * 	@return			     - Command Response
 *
 * 	@note
 */
static Command_Response_t SD_Command(SD_Handle_t* pSDHandle, sd_cmd_ID_t cmdID, uint32_t argument, sd_cmd_crc_t crc)
{
    Command_Response_t CmdResponse = {0};
    uint8_t dummy_write[2] = {0xFF, 0xFF};
    uint8_t dummy_read;

    /**** First Construct the Command Frame  ****/
    uint8_t CommandFrame[COMMAND_FRAME_LEN] = {0};

    // Ensure the SPI Peripheral is in 8-bit mode
    SPI_UpdateDFF(pSDHandle->pSPIx, SPI_DFF_8BITS);

    /* Dummy clock (force DO hi-z for multiple slave SPI) */
    SPI_SendData(pSDHandle->pSPIx, dummy_write, 2);

    /****	6-bit Command Index	+ (Start & Transmission bit)	****/
    CommandFrame[0] = ((cmdID & 0x3F) | TRAN_BIT);

    /****	Copy over the argument	****/
    CommandFrame[1] = ((argument >> 24) & 0xFF);
    CommandFrame[2] = ((argument >> 16) & 0xFF);
    CommandFrame[3] = ((argument >> 8) & 0xFF);
    CommandFrame[4] = ((argument >> 0) & 0xFF);

    /*******		   Command CRC + Stop bit 				******/
    /* Crc is zero in most cases, otherwise hard-coded by Macro  */
    CommandFrame[5] = crc | STOP_BIT;

    // Transmit the Command Frame
    SPI_SendData(pSDHandle->pSPIx, CommandFrame, COMMAND_FRAME_LEN);

    // Dummy Read
    SPI_ReceiveData(pSDHandle->pSPIx, &dummy_read, 1);

    if (cmdID == CMD8)
    {
        CmdResponse = SD_GetResponse(pSDHandle, RESPONSE_R7);
    }
    else if (cmdID == CMD58)
    {
        CmdResponse = SD_GetResponse(pSDHandle, RESPONSE_R3);
    }
    else
    {
        CmdResponse = SD_GetResponse(pSDHandle, RESPONSE_R1);
    }

    // CMD12 Requires a R1b busy state
    if (cmdID == CMD12)
    {
        SD_WaitBusyState(pSDHandle, &CmdResponse.R1.Flags);
    }

    // Error Handling
    if (SD_GetTimeoutStatus(pSDHandle) == TIMEOUT_EXPIRED)
    {
        pSDHandle->SD_CardState = SD_STATE_FAIL;
    }

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_App_Command
 *
 * 	@brief			     - Send SPI SD Application Command
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[Cmd]	 		 - Command Index
 * 	@param[Argument]	 - Command Argument
 * 	@param[Crc]	 		 - Crc for command. Most Commonly zero, otherwise Macro Hard-coded
 *
 * 	@return			     - App Command Response
 *
 * 	@note  				 - 	ACMD<n> means a command sequence of CMD55-CMD<n>
 */
static Command_Response_t SD_App_Command(SD_Handle_t* pSDHandle, sd_cmd_ID_t cmdID, uint32_t argument, sd_cmd_crc_t crc)
{
    Command_Response_t CmdResponse = {0};

    /*****   		  First Send APP_CMD			  *****/
    /* 		This tells the card to interpret the next     */
    /* 		command as application-specific			      */
    /***   Argument: [31:16] RCA, [15:0] stuff bit      ***/
    SD_Command(pSDHandle, CMD55, CMD_ARG_NULL, CMD_CRC_NULL);

    /*****   	Next Send Application Specific Command	  *****/
    CmdResponse = SD_Command(pSDHandle, cmdID, argument, crc);

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_GetResponse
 *
 * 	@brief			     - Function to receive and parse SPI Command response
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[Format]	 	 - Response Format
 *
 * 	@return			     - Parsed Command Response
 *
 * 	@note
 */
static Command_Response_t SD_GetResponse(SD_Handle_t* pSDHandle, sd_response_t Format)
{
    uint8_t CmdResponse[5] = {0};
    uint8_t* ResponsePtr = CmdResponse;

    // Wait until SPI peripheral is not busy
    while (SPI_GetFlagStatus(pSDHandle->pSPIx, SPI_FLAG_BSY))
    {
    }

    /* Receive Command Response
	 * Sent back within command response time (NCR)
	 * 0 to 8 bytes for SDC, 1 to 8 bytes for MMC */
    uint8_t count = 0;

    *ResponsePtr = 0xFF;
    // loop until the bus is not idle
    do
    {
        SPI_MasterTransfer(pSDHandle->pSPIx, ResponsePtr, 1);
        count++;
    } while ((*ResponsePtr == 0xFF) && (count < (NCR_BYTES + 1)));

    if ((Format == RESPONSE_R3) || (Format == RESPONSE_R7))
    {
        ResponsePtr++;
        // Read in the 32-bit extended command
        SPI_MasterTransfer(pSDHandle->pSPIx, ResponsePtr, 4);
    }

    return SD_ParseResponse(CmdResponse, Format);
}

/****************************************************************************************
 *	@fn 			     - SD_WaitBusyState
 *
 * 	@brief			     - Wait during SD busy state (Multi-block read/writes)
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note        // TODO: Conisder making this interrupt driven for better multitasking??
 */
static void SD_WaitBusyState(SD_Handle_t* pSDHandle, uint8_t* response)
{
    // R1b response is an R1 response followed by an optional busy state.
    // The card will hold MISO low until the card is done processing the current task
    SPI_MasterTransfer(pSDHandle->pSPIx, response, 1);

    // Start Cmd Timeout
    SD_TimeoutConfig(pSDHandle, ENABLE);

    uint8_t idle = 0x00;

    do
    {
        SPI_MasterTransfer(pSDHandle->pSPIx, &idle, 1);
    } while ((idle != 0xFF) && (SD_GetTimeoutStatus(pSDHandle) != TIMEOUT_EXPIRED));

    // Stop Cmd Timeout
    SD_TimeoutConfig(pSDHandle, DISABLE);
}

/****************************************************************************************
 *	@fn 			     - SD_ParseResponse
 *
 * 	@brief			     - Function to parse response of SPI Command
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[Format]	 	 - Response Format
 *
 * 	@return			     - Structure for Command Response
 *
 * 	@note
 */
static Command_Response_t SD_ParseResponse(uint8_t* Response, sd_response_t Format)
{
    Command_Response_t temp = {0};
    uint32_t ext_resp;
    // Construct the 32-bit extended response into one word because its clearer to deal with.
    ext_resp = Response[4] | ((uint32_t)Response[3] << 8) | ((uint32_t)Response[2] << 16) | ((uint32_t)Response[1] << 24);

    // All Commands Contain R1
    temp.R1.Flags = Response[0];

    if (Format == RESPONSE_R3)
    {
        temp.R3.OCR = ext_resp;
    }
    else if (Format == RESPONSE_R7)
    {
        temp.R7.Check_Pattern = (ext_resp & 0xFF);
        temp.R7.Voltage_Accepted = ((ext_resp >> 8) & 0xF);
        temp.R7.Reserved = ((ext_resp >> 12) & 0xFFFF);
        temp.R7.Command_Version = ((ext_resp >> 28) & 0xF);
    }

    return temp;
}

/**********************************************************************************************
* 								   SPI Init Commands Section
**********************************************************************************************/

/****************************************************************************************
 *	@fn 			     - SD_PowerSequence
 *
 * 	@brief			     - Function to run SPI power sequence
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - none
 *
 * 	@note
 */
static void SD_PowerSequence(SD_Handle_t* pSDHandle)
{
    /****** Send 80 dummy clock cycles with CI & MOSI high  ******/
    SD_ChipSelectControl(pSDHandle, HIGH);

    uint8_t n;
    uint8_t data = 0xFF;
    for (n = 0; n < 15; n++)
    {
        SPI_SendData(pSDHandle->pSPIx, &data, 1);
    }
}

/****************************************************************************************
 *	@fn 			     - SD_GoIdleState (CMD0)
 *
 * 	@brief			     - Function to reset the SD Memory Card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - Returns R1 response
 *
 * 	@note
 */
static Command_Response_t SD_GoIdleState(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    /*******   		Software reset (GO_IDLE_STATE)		*****/
    /**   CMD0 tells the card to use the SPI interface.     **/
    /** This cannot be changed once the part is powered on  **/
    CmdResponse = SD_Command(pSDHandle, CMD0, CMD_ARG_NULL, CMD0_CRC);

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_CheckSupplyRange
 *
 * 	@brief			     - Function to ask card if it can operate in supplied range
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - Returns R7 response
 *
 * 	@note
 */
static Command_Response_t SD_CheckSupplyRange(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    /*******   Check Supply Range (SD_CheckSupplyRange)	*****/
    CmdResponse = SD_Command(pSDHandle, CMD8, CMD8_ARG, CMD8_CRC);

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_SendOpCond
 *
 * 	@brief			     - Function to Initiate initialization process
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - Returns R1 response
 *
 * 	@note
 */
static Command_Response_t SD_SendOpCond(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    /* Send host capacity support information (HCS) and
	 *  asks the card to send its operating condition register (OCR)
	 * content in the response on the CMD line	*/
    CmdResponse = SD_App_Command(pSDHandle, ACMD41, ACMD41_ARG, CMD_CRC_NULL);

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_ReadOcrRegister
 *
 * 	@brief			     - Function to Read OCR Register
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - Structure for Command Response
 *
 * 	@note				 - Returns R3 response (OCR)
 */
static Command_Response_t SD_ReadOcrRegister(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    /* Reads the OCR register of a card. CCS bit is assigned to OCR[30]
	 * The 32-bit operation conditions register stores the VDD voltage profile
	 * of the card.	Additionally, this register includes status information bits
	 * The Card power up status bit is set if the power up procedure has finished */
    CmdResponse = SD_Command(pSDHandle, CMD58, CMD_ARG_NULL, CMD_CRC_NULL);

    return CmdResponse;
}

/****************************************************************************************
 *	@fn 			     - SD_SetBlockLength
 *
 * 	@brief			     - Function to set block length of SDSC Card
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - Structure for Command Response
 *
 * 	@note				 - Returns R1 response
 */
static Command_Response_t SD_SetBlockLength(SD_Handle_t* pSDHandle)
{
    Command_Response_t CmdResponse = {0};

    /* For SDSC Card, block length is set by this command
	 * For SDHC and SDXC Cards, block length of the memory
	 * access commands are fixed to 512 bytes */
    CmdResponse = SD_Command(pSDHandle, CMD16, CMD16_ARG, CMD_CRC_NULL);

    return CmdResponse;
}

/**********************************************************************************************
* 						    SPI Read/Write Commands Section
**********************************************************************************************/

/****************************************************************************************
 *	@fn 			     - SD_ReadBlock
 *
 * 	@brief			     - Function to read block of data
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - Read status
 *
 * 	@note				 -
 */
sd_read_write_t SD_ReadBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount)
{
    Command_Response_t CmdResponse = {0};

    if (BlockCount == 0 || ((BlockCount * SD_DEFAULT_BLOCK_SIZE) > SD_GetBuffSize(pSDHandle)))
    {
        return SD_READ_WRITE_FAIL;
    }
    if (BlockCount == 1)
    {
        // Assert chip select low
        SD_ChipSelectControl(pSDHandle, LOW);

        /*******   READ_SINGLE_BLOCK	*****/
        CmdResponse = SD_Command(pSDHandle, CMD17, BlockAddr, CMD_CRC_NULL);
    }
    else
    {
        // Assert chip select low
        SD_ChipSelectControl(pSDHandle, LOW);

        /*******    READ_MULTIPLE_BLOCK  	*****/
        CmdResponse = SD_Command(pSDHandle, CMD18, BlockAddr, CMD_CRC_NULL);
    }

    // Confirm command was successful. All flags will be cleared
    if (CmdResponse.R1.Flags != 0x00)
    {
        SD_ChipSelectControl(pSDHandle, HIGH);
        return SD_READ_WRITE_FAIL;
    }

    /***   Receive Block Read    ****/
    if (pSDHandle->SD_TransferMode == SD_TRANSFER_DMA || pSDHandle->SD_TransferMode == SD_TRANSFER_NON_DMA)
    {
        /*            Data Packet Format
         * ----------------------------------------
         * |Data Token |  Data Block    | CRC16   |
         * ----------------------------------------
         * | 1 byte   | 1 - 2048 bytes | 2 bytes  |
         * ----------------------------------------
         */

        uint8_t* rxBuffer = SD_GetBuffAddr(pSDHandle);
        uint8_t CRC[2];
        uint16_t n;

        for (n = 0; n < BlockCount; n++, rxBuffer += SD_DEFAULT_BLOCK_SIZE)
        {
            uint8_t token = 0xFF;

            // Start Cmd Timeout
            SD_TimeoutConfig(pSDHandle, ENABLE);

            // Read Until data Command Token Arrives
            do
            {
                SPI_MasterTransfer(pSDHandle->pSPIx, &token, 1);
            } while ((token != BLOCK_READ_TOKEN) && (SD_GetTimeoutStatus(pSDHandle) != TIMEOUT_EXPIRED));

            // Stop Cmd Timeout
            SD_TimeoutConfig(pSDHandle, DISABLE);

            // Error Handling
            if (SD_GetTimeoutStatus(pSDHandle) == TIMEOUT_EXPIRED)
            {
                // Release chip select
                SD_ChipSelectControl(pSDHandle, HIGH);

                // Set Fail flags
                pSDHandle->SD_CardState = SD_STATE_FAIL;
                return SD_READ_WRITE_FAIL;
            }

            // Update SPI Peripheral to 16-bit for faster block read
            SPI_UpdateDFF(pSDHandle->pSPIx, SPI_DFF_16BITS);

            // Read the data block
            SPI_MasterTransfer(pSDHandle->pSPIx, rxBuffer, SD_DEFAULT_BLOCK_SIZE);

            // Receive the 16-bit checksum
            SPI_MasterTransfer(pSDHandle->pSPIx, CRC, 2);

            // Switch back to 8-bit Mode
            SPI_UpdateDFF(pSDHandle->pSPIx, SPI_DFF_8BITS);
        }

        if (BlockCount > 1)
        {
            /*******    STOP_TRANSMISSION  	*****/
            CmdResponse = SD_Command(pSDHandle, CMD12, CMD_ARG_NULL, CMD_CRC_NULL);
        }

        // Release chip select
        SD_ChipSelectControl(pSDHandle, HIGH);
    }

    return SD_READ_WRITE_SUCCESS;
}

/****************************************************************************************
 *	@fn 			     - SD_WriteBlock
 *
 * 	@brief			     - Function to write block of data
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[BlockAddr]	 - Block Address
 * 	@param[BlockCount]	 - Block Count
 *
 * 	@return			     - Read status
 *
 * 	@note				 -
 */
sd_read_write_t SD_WriteBlock(SD_Handle_t* pSDHandle, uint32_t BlockAddr, uint32_t BlockCount)
{
    Command_Response_t CmdResponse = {0};

    if (BlockCount == 0 || ((BlockCount * SD_DEFAULT_BLOCK_SIZE) > SD_GetBuffSize(pSDHandle)))
    {
        return SD_READ_WRITE_FAIL;
    }
    if (BlockCount == 1)
    {
        // Assert chip select low
        SD_ChipSelectControl(pSDHandle, LOW);

        /*****   WRITE_BLOCK	*****/
        CmdResponse = SD_Command(pSDHandle, CMD24, BlockAddr, CMD_CRC_NULL);
    }
    else
    {
        // Assert chip select low
        SD_ChipSelectControl(pSDHandle, LOW);

        // TODO: Verify multiblock write
        /*****     WRITE_MULTIPLE_BLOCK  	*****/
        CmdResponse = SD_Command(pSDHandle, CMD25, BlockAddr, CMD_CRC_NULL);
    }

    // Confirm command was successful. All flags will be cleared
    if (CmdResponse.R1.Flags != 0x00)
    {
        SD_ChipSelectControl(pSDHandle, HIGH);
        return SD_READ_WRITE_FAIL;
    }

    /***   Receive Block Read    ****/
    if (pSDHandle->SD_TransferMode == SD_TRANSFER_DMA || pSDHandle->SD_TransferMode == SD_TRANSFER_NON_DMA)
    {
        /*            Data Packet Format
         * ----------------------------------------
         * |Data Token |  Data Block    | CRC16   |
         * ----------------------------------------
         * | 1 byte   | 1 - 2048 bytes | 2 bytes  |
         * ----------------------------------------
         */

        uint8_t* TxBuffer = SD_GetBuffAddr(pSDHandle);
        uint8_t dummy[2] = {0xFF, 0xFF};
        uint8_t CRC[2] = {0xFF, 0xFF};
        uint16_t n;
        uint8_t dataResp;

        // Must wait at least one byte before transferring Data Packet
        SPI_MasterTransfer(pSDHandle->pSPIx, dummy, 1);

        for (n = 0; n < BlockCount; n++, TxBuffer += SD_DEFAULT_BLOCK_SIZE)
        {
            uint8_t token = (BlockCount == 1) ? SINGLE_BLOCK_WRITE_TOKEN : MULT_BLOCK_WRITE_TOKEN;

            // Transfer Data Token
            SPI_SendData(pSDHandle->pSPIx, &token, 1);

            // Send Data Block
            SPI_SendData(pSDHandle->pSPIx, TxBuffer, SD_DEFAULT_BLOCK_SIZE);

            // Transfer CRC16
            SPI_SendData(pSDHandle->pSPIx, CRC, 2);

            // Dummy Read for CRC16
            SPI_ReceiveData(pSDHandle->pSPIx, dummy, 1);

            // Wait for Card to process the command
            SD_WaitBusyState(pSDHandle, &dataResp);

            // Error Handling
            if (SD_GetTimeoutStatus(pSDHandle) == TIMEOUT_EXPIRED || (dataResp != WRITE_DATA_ACCEPTED))
            {
                // Release chip select
                SD_ChipSelectControl(pSDHandle, HIGH);

                // Set Fail flags
                pSDHandle->SD_CardState = SD_STATE_FAIL;
                return SD_READ_WRITE_FAIL;
            }
        }

        if (BlockCount > 1)
        {
            uint8_t stopToken = STOP_WRITE_TOKEN;

            // Transfer Stop Data Token
            SPI_SendData(pSDHandle->pSPIx, &stopToken, 1);

            // Dummy Read
            SPI_ReceiveData(pSDHandle->pSPIx, dummy, 1);

            // Start Wait for busy state to conclude
            SD_WaitBusyState(pSDHandle, dummy);
        }

        // Release chip select
        SD_ChipSelectControl(pSDHandle, HIGH);
    }

    if (SD_GetTimeoutStatus(pSDHandle) == TIMEOUT_EXPIRED)
    {
        return SD_READ_WRITE_FAIL;
    }
    else
    {
        return SD_READ_WRITE_SUCCESS;
    }
}

/**********************************************************************************************
* 						  		  SD Helper Functions
**********************************************************************************************/

/****************************************************************************************
 *  @fn                - SD_GetState
 *
 *  @brief             - Get SD Card State
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *
 *  @return            -  CSD_CardState
 *
 *  @note              - 
 */
SD_States_t SD_GetState(SD_Handle_t* pSDHandle)
{
    return pSDHandle->SD_CardState;
}

/****************************************************************************************
 *  @fn                - SD_GetTimeoutStatus
 *
 *  @brief             - Get SD Command Timeout Status
 *
 *  @param[pSDHandle]  - Handler structure for SD Card
 *
 *  @return            - CSD_CardState
 *
 *  @note              - TIMEOUT_NON_EXPIRED or TIMEOUT_EXPIRED
 */
static timeout_status_t SD_GetTimeoutStatus(SD_Handle_t* pSDHandle)
{
    return pSDHandle->cmdTimeout.Status;
}

/****************************************************************************************
 *  @fn                - SD_TimeoutConfig
 *
 *  @brief             - Start Timeout for Command
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *  @param[EnOrDi]     -  Enable or Disable
 * 
 *  @return            -  
 *
 *  @note              - 
 */
static void SD_TimeoutConfig(SD_Handle_t* pSDHandle, EnOrDi_t EnOrDi)
{
    TIM_RegDef_t* sdTIM = pSDHandle->cmdTimeout.TimHandle.pTIMx;

    if (EnOrDi == ENABLE)
    {
        pSDHandle->cmdTimeout.Status = TIMEOUT_NON_EXPIRED;
        TIM_PeripheralControl(sdTIM, ENABLE);
        TIM_ReInit(sdTIM);
    }
    else
    {
        TIM_PeripheralControl(sdTIM, DISABLE);
    }
}

/****************************************************************************************
 *	@fn 			     - SD_IsReady
 *
 * 	@brief			     - Function to set read block of data
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - True or False
 *
 * 	@note				 -
 */
bool SD_IsReady(SD_Handle_t* pSDHandle)
{
    if (SD_GetState(pSDHandle) == SD_STATE_READY)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/****************************************************************************************
 *  @fn                - SD_GetBuffAddr
 *
 * 	@brief			   - Function to get the working buffer memory address
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *
 *  @return            -  memory location
 *
 *  @note              -
 */
uint8_t* SD_GetBuffAddr(SD_Handle_t* pSDHandle)
{
    return pSDHandle->bufferInfo.pCurrBuf;
}

/****************************************************************************************
 *  @fn                - SD_SetCurrBuff
 *
 *  @brief             - Set Current Buffer 
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *
 *  @return            -  
 *
 *  @note              - 
 */
void SD_ToggleCurrBuff(SD_Handle_t* pSDHandle)
{
    if (pSDHandle->bufferInfo.pCurrBuf == pSDHandle->bufferInfo.pBufA)
    {
        pSDHandle->bufferInfo.pCurrBuf = pSDHandle->bufferInfo.pBufB;
    }
    else
    {
        pSDHandle->bufferInfo.pCurrBuf = pSDHandle->bufferInfo.pBufA;
    }
}

/****************************************************************************************
 *	@fn 			     - SD_GetBuffSize
 *
 * 	@brief			     - Function to get buffer size
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 *
 * 	@return			     - buffer size
 *
 * 	@note				 -
 */
uint32_t SD_GetBuffSize(SD_Handle_t* pSDHandle)
{
    return pSDHandle->bufferInfo.Size;
}

/*******************       I/O Functions      *******************/

/****************************************************************************************
 *	@fn 			     - SD_ChipSelectControl
 *
 * 	@brief			     - Function to control SD chip select pin
 *
 * 	@param[pSDHandle]	 - Handler structure for SD Card
 * 	@param[HiOrLow]	 	 - Command Argument
 *
 * 	@return			     - none
 *
 * 	@note
 */
static void SD_ChipSelectControl(SD_Handle_t* pSDHandle, gpio_pin_state_t state)
{
    GPIO_Handle_t cs = pSDHandle->ChipSelHandle;

    // Wait until SPI peripheral is not busy
    while (SPI_GetFlagStatus(pSDHandle->pSPIx, SPI_FLAG_BSY) == FLAG_SET)
    {
    }

    GPIO_WriteToOutputPin(cs.pGPIOx, cs.GPIO_PinConfig.GPIO_PinNumber, state);
}

/****************************************************************************************
 *  @fn                - SD_GetCDStatus
 *
 *  @brief             - Read Card Detect Input Pin
 *
 *  @param[pSDHandle]  -  Handler structure for SD Card
 *
 *  @return            -  CD_DETECTED or CD_REMOVED
 *
 *  @note              - 
 */
card_detect_t SD_GetCDStatus(SD_Handle_t* pSDHandle)
{
    GPIO_Handle_t cd = pSDHandle->CardDetHandle;

    // Active High Switch
    if (GPIO_ReadFromInputPin(cd.pGPIOx, cd.GPIO_PinConfig.GPIO_PinNumber) == pSDHandle->CardDetPol)
    {
        return CD_DETECTED;
    }
    else
    {
        return CD_REMOVED;
    }
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
    if (SD_GetCDStatus(pSDHandle) == CD_REMOVED)
    {
        pSDHandle->SD_CardState = SD_STATE_NO_CARD;
    }
}
