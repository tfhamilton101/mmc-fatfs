/*
 * Utils.c
 *
 *  Created on: Nov 4th, 2020
 *      Author: thomashamilton
 */

#include "Utils.h"

/***********************************************************
 * 					Static Functions			   		   *
 ***********************************************************/
static uint8_t nibToHex(uint8_t num);

/****************************************************************************************
 *  @fn                - ToLittleEndian
 *
 *  @brief             - Function to convert data into Little Endian
 *
 *  @param[data]       - memory location of data buffer
 *  @param[size]       - size of conversion
 *
 *  @return            - Converted value 
 *
 *  @note              - 
 */
uint32_t ToLittleEndian(uint8_t* data, DataSize_t size)
{
    uint32_t temp = 0;
    uint8_t n;

    for (n = 0; n < size; n++, data++)
    {
        temp += ((uint32_t)(*data) << (8 * n));
    }

    return temp;
}

/****************************************************************************************
 *  @fn                - ToBigEndian
 *
 *  @brief             - Function to convert data into Big Endian
 *
 *  @param[data]       - memory location of data buffer
 *  @param[size]       - size of conversion
 *
 *  @return            - Converted value 
 *
 *  @note              - 
 */
uint32_t ToBigEndian(uint8_t* data, DataSize_t size)
{
    uint32_t temp = 0;
    uint8_t n;

    for (n = 0; n < size; n++, data++)
    {
        temp += ((uint32_t)(*data) << (8 * (size - n - 1)));
    }

    return temp;
}

/****************************************************************************************
 *	@fn              - ToEndianBuf
 *
 * 	@brief			 - Function to convert data into Little Endian
 *
 *  @param[data]	 - memory location of data buffer
 *  @param[value]	 - value to convert in
 * 	@param[size]	 - size of conversion
 *
 * 	@return			 - none
 * 	@note
 */
void ToEndianBuf(uint8_t* data, uint32_t value, DataSize_t size)
{
    uint8_t n;

    for (n = 0; n < size; n++, data++)
    {
        *data = (uint8_t)(value >> (8 * n));
    }
}

/****************************************************************************************
 *	@fn 			     - bytesToHex
 *
 * 	@brief			     - Convert a byte into ASCII hex
 *
 * 	@param[num]	 - Handler structure for FAT
 * 	@param[buf]  - Memory location to store result
 * 	@param[size] - Size of variable to convert
 *
 * 	@return			     - none
 *
 * 	@note
 */
void bytesToHex(uint8_t* buf, uint32_t num, DataSize_t size)
{
    int32_t n;
    // Loop through the nibbles and convert to hex
    for (n = 2 * size; n > 0; n--, buf++)
    {
        *buf = nibToHex(num >> (4 * (n - 1)));
    }
    // Terminate the string
    *buf = '\0';
}

/****************************************************************************************
 *	@fn 			     - nibToHex
 *
 * 	@brief			     - Convert a byte into ASCII hex
 *
 * 	@param[num]	 - Handler structure for FAT
 *
 * 	@return			     - converted hex nibble
 *
 * 	@note
 */
static uint8_t nibToHex(uint8_t num)
{
    // Remove upper nibble
    num &= 0x0F;

    // Translate into hex
    if (num < 10)
    {
        return (uint8_t)'0' + num;
    }
    else
    {
        return (uint8_t)'A' + num - 10;
    }
}

/****************************************************************************************
 *	@fn 			 - strncpylower
 *
 * 	@brief			 - Convert a byte into ASCII hex
 *
 * 	@param[dest]	 - Destination buffer
 * 	@param[source]	 - Input source
 * 	@param[n]	     - max number of iterations
 *
 * 	@return			 - none
 *
 * 	@note
 */
void strncpylower(uint8_t* dest, uint8_t* source, uint8_t n)
{
    // Loop until the string is terminated or n iterations
    for (; (*source != '\0') && (n > 0); source++, dest++, n--)
    {
        *dest = tolower(*source);
    }
    // Terminate the string
    *dest = '\0';
}

/****************************************************************************************
 *	@fn 			 - strncpyUpper
 *
 * 	@brief			 - Convert a byte into ASCII hex
 *
 * 	@param[dest]	 - Destination buffer
 * 	@param[source]	 - Input source
 * 	@param[n]	     - max number of iterations
 *
 * 	@return			 - none
 *
 * 	@note
 */
void strncpyUpper(uint8_t* dest, uint8_t* source, uint8_t n)
{
    // Loop until the string is terminated or n iterations
    for (; (*source != '\0') && (n > 0); source++, dest++, n--)
    {
        *dest = toupper(*source);
    }
    // Terminate the string
    *dest = '\0';
}