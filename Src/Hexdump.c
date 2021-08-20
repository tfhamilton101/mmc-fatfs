/*
 * Hexdump.c
 *
 *  Created on: Aril 30, 2021
 *      Author: thomashamilton
 */
#include "Hexdump.h"

uint8_t DumpBuf[DUMP_BUF_SIZE] = {0};

/****************************************************************************************
 *	@fn 			     - Hexdump
 *
 * 	@brief			     - Function to print blocks of memory to the console
 *
 * 	@param[pUSARTHandle] - Handler structure for USART
 * 	@param[pReadHandle]	 - Generic driver Handler for reading data
 * 	@param[blockRead]	 - Function pointer to sub read block function
 * 	@param[addr]		 - Memory address to print
 * 	@param[blocks]		 - Number of sectors to print
 *
 * 	@return			     - none
 *
 * 	@note
 */
void Hexdump(USART_Handle_t* pUSARTHandle, void* pReadHandle, dumpInfo_t (*readBlock)(void*, uint32_t, uint32_t), uint32_t addr, uint32_t blocks)
{
    char print_buf[24];

    memset(DumpBuf, 0, DUMP_BUF_SIZE);

    // Get the block size and address unit
    dumpInfo_t dumpInfo = readBlock(pReadHandle, addr, 0);
    uint32_t blocksPerBuffer = dumpInfo.buffSize / dumpInfo.blockSize;

    // Print the block address
    sprintf(print_buf, "\nBlock %ld (0x%08lX) \n", addr / dumpInfo.addrUnit, addr);
    USART_PrintString(pUSARTHandle, print_buf);

    uint32_t n = 0;

    while (n < blocks)
    {
        uint32_t blocksRead = 0;

        if ((n + blocksPerBuffer) < blocks)
        {
            // Determine how many blocks to read
            blocksRead = blocksPerBuffer;

            // Read full buffer
            addr += readBlock(pReadHandle, addr, blocksRead).addrUnit;
        }
        else
        {
            // Determine how many blocks to read
            blocksRead = blocks - n;

            // Read the next blocks
            addr += readBlock(pReadHandle, addr, blocksRead).addrUnit;
        }

        n += blocksRead;

        HexdumpBuffer(pUSARTHandle, dumpInfo.pbuff, blocksRead * dumpInfo.blockSize);
    }

    USART_PrintString(pUSARTHandle, "\n");
}

/****************************************************************************************
 *	@fn 			     - HexdumpBuffer
 *
 * 	@brief			     - Function to print blocks of memory to the console
 *
 * 	@param[pUSARTHandle] - Handler structure for USART
 * 	@param[buf]		     - Buffer address to dump
 * 	@param[bufSize]		 - Size of buffer address
 *
 * 	@return			     - none
 *
 * 	@note
 */
void HexdumpBuffer(USART_Handle_t* pUSARTHandle, uint8_t* buf, uint32_t bufSize)
{
    uint32_t byte_index;
    uint32_t offset_count = 0;
    char print_buf[24];
    uint8_t* currLine;
    uint8_t* pdata = buf;

    // Print the Block to the Console
    for (byte_index = 0; byte_index < bufSize; byte_index++)
    {
        // Every 16 bytes print the offset address
        if ((byte_index % 16) == 0)
        {
            // Convert the address to hex
            bytesToHex((uint8_t*)print_buf, 16 * offset_count, DATA_SIZE_WORD);
            strcat(print_buf, "  ");

            // Print address with 8 hex characters
            USART_PrintString(pUSARTHandle, "\n");
            USART_PrintString(pUSARTHandle, print_buf);

            // Increment the offset count
            offset_count++;
            currLine = pdata;
        }

        // Every 8 bytes send an extra space
        if ((byte_index % 16) == 8)
        {
            USART_PrintString(pUSARTHandle, " ");
        }

        // Print each byte in hex
        bytesToHex((uint8_t*)print_buf, *pdata, DATA_SIZE_BYTE);
        strcat(print_buf, " ");

        // Print data with 2 hex characters
        USART_PrintString(pUSARTHandle, print_buf);

        // bump the data pointer
        pdata++;

        // If we are on the last byte of the line then print in readable text.
        if ((byte_index % 16) == 15)
        {
            uint8_t bytecount;
            USART_PrintString(pUSARTHandle, " |");

            for (bytecount = 0; bytecount < 16; bytecount++)
            {
                // Print non readable characters as a dot
                if ((*currLine < ' ') || (*currLine > '~'))
                {
                    USART_PrintString(pUSARTHandle, ".");
                }
                else
                {
                    USART_SendData(pUSARTHandle, currLine, 1);
                }

                currLine++;
            }

            USART_PrintString(pUSARTHandle, "|");
        }
    }

    USART_PrintString(pUSARTHandle, "\n");
}

/***********************************************************
 *                     Sub Functions                       *
 ***********************************************************/

dumpInfo_t FATdump(void* handle, uint32_t addr, uint32_t blocks)
{
    FAT_Handle_t* pFAT = (FAT_Handle_t*)handle;
    dumpInfo_t info = {0};

    // First Grab the dump info
    info.blockSize = pFAT->SystemInfo.BytesPerSector;
    info.buffSize = SD_GetBuffSize(pFAT->pSDHandle);
    info.pbuff = SD_GetBuffAddr(pFAT->pSDHandle);

    if (blocks == 0)
    {
        info.addrUnit = getFatAddrUnit(pFAT);
    }
    else
    {
        info.addrUnit = getFatAddrUnit(pFAT) * blocks;
    }

    SD_ReadBlock(pFAT->pSDHandle, addr, blocks);

    return info;
}

dumpInfo_t CodecDump(void* handle, uint32_t addr, uint32_t blocks)
{
    stereo_handle_t* StereoHandle = (stereo_handle_t*)handle;
    dumpInfo_t info = {0};

    // First Grab the dump info
    info.blockSize = 64;
    info.addrUnit = 64;
    info.buffSize = 256;
    info.pbuff = DumpBuf;

    CodecRegRead(StereoHandle, addr, info.pbuff, 52);

    return info;
}
