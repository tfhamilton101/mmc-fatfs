/****************************************
 * Fat.c
 *
 *  Created on: Nov 22th, 2020
 *      Author: thomashamilton
 ****************************************/

#include "FAT.h"
#include "stm32f4xx_dma_driver.h"
#include "Stack.h"
#include "Utils.h"
#include <string.h>

/******  FAT DEBUG CONFIGS  ******/

// #define FAT_DEBUG_GENERIC
// #define FAT_DEBUG_TABLE

#if defined(FAT_DEBUG_GENERIC)
#include "Hexdump.h"
#endif


/************************************************************************************
 *							 File Specific Macros    								*
 ************************************************************************************/

#define DIR_BYTES_PER_ENTRY 32
#define MAX_DIRECTORIES 32
#define MAX_ENTIRES_PER_DIRECTORY 256

typedef enum
{
    FILENAME = 0,
    FILE_EXTENSION = 0x08,
    FILE_ATTRIBUTE = 0x0B,
    FILE_START_CLUSTER_HI = 0x14,
    FILE_TIME_MODIFIED = 0x16,
    FILE_DATE_MODIFIED = 0x18,
    FILE_START_CLUSTER_LO = 0x1A,
    FILE_SIZE = 0x1C,
    // Long Filename offsets
    LFN_ORDINAL = 0x00,
    LFN_CHECKSUM = 0x0D,
} FAT_file_entry_offset_t;

/*
 * Special values for first byte of the filename
 */
typedef enum
{
    // End of a directory
    END_OF_DIRECTORY = 0x00,
    // Filename has been used, now deleted
    FILENAME_DELETED = 0xe5,
    // First character of the filename is actually 0xe5 (Japan)
    FILENAME_E5h = 0x05,
    // First two entries of a subdirectory
    FILENAME_PARENT = 0x2E
} FAT_entry_type_t;

/*
 *  Flags held in the file attribute byte
 *  Note: bits 6 & 7 are reserved
 */
typedef enum
{
    FILE_FLAG_READ_ONLY = BIT0,
    FILE_FLAG_HIDDEN = BIT1,
    FILE_FLAG_SYSTEM = BIT2,
    FILE_FLAG_VOLUME = BIT3,
    FILE_FLAG_DIRECTORY = BIT4,
    FILE_FLAG_ARCHIVED = BIT5,
    FILE_FLAG_LFN = 0x0F
} FAT_file_flags_t;

/**************************************************
 *           Traverse FAT Table Define            *
 **************************************************/
typedef enum
{
    FAT_TRAVERSE_LOAD_QUEUE = 0,
    FAT_TRAVERSE_FIND_LAST_ID,
} fat_traverse_mode_t;

/************************************************************************************
 *							FAT Offset Macros										*
 ************************************************************************************/

/*** Partition Type Code Macros  ***/
typedef enum
{
    MBR_TYPE_CODE_FAT32_A = 0x0B,
    MBR_TYPE_CODE_FAT32_B = 0x0C,
    MBR_TYPE_CODE_FAT16 = 0x0E,
} mbr_type_code_t;


/**** 	Offset Macros for Boot Sector 	****/
typedef enum
{
    /*** FAT16 Specific ***/
    BOOT_BLK_BYTES_PER_SEC = 0x0B,
    BOOT_BLK_SEC_PER_CLUSTER = 0x0D,
    BOOT_BLK_RESERVED_SECT = 0x0E,
    BOOT_BLK_NUM_FATS = 0x10,
    BOOT_BLK_NUM_ROOT_ENTRIES = 0x11,
    BOOT_BLK_SECTORS_PER_TABLE_FAT16 = 0x16,
    BOOT_BLK_TOTAL_VOLUME_BLOCKS = 0x20,
    BOOT_BLK_FILE_SYS_TYPE_FAT16 = 0x36,
    /*** FAT32 Specific ***/
    BOOT_BLK_ROOT_DIR_CLUSTER_FAT32 = 0x2C,
    BOOT_BLK_SECTORS_PER_TABLE_FAT32 = 0x24,
    BOOT_BLK_FILE_SYS_TYPE_FAT32 = 0x52,
} boot_block_offsets_t;

#define BOOT_SIGNATURE 0xAA55

/************************************************************************************
 *						File Allocation Table Macros								*
 ************************************************************************************/

/**************************************************************************************
 * An queue is used to hold the clusterIDs that make a file. This is read from the FAT.
 * The size of the queue should be chosen such that most files can fit fully into the queue.
 * This will prevent the program from going back and reading the FAT mid way through a file.
 *
 * 						FAT16 								FAT32
 * 	  ------------------------------------------------------------------------------
 * 	  | Volume Size	  |	 Windows Default	|| Volume Size	 |	Windows Default	   |
 * 	  ------------------------------------------------------------------------------
 * 	  | 512 MB – 1 GB | 16KB ( 32 sectors)  || 256 MB – 8GB  |	 4KB (  8 sectors) |
 * 	  |   1 GB – 2 GB | 32KB ( 64 sectors)  ||    8GB – 16GB |	 8KB ( 16 sectors) |
 * 	  |   2 GB – 4 GB | 64KB ( 128 sectors) ||   16GB – 32GB |	16KB ( 32 sectors) |
 * 	  ------------------------------------------------------------------------------
 *
 *   Using the smallest cluster size (8 sectors) a queue size of 4096
 *   will allow a 16MB ( 4096 * 8 * 512 ) minimum file to fit in the queue.
 *
 * 	  -----------------------------------------------
 * 	  |  File Size  |	 FAT16	    |	 FAT32	    |
 * 	  -----------------------------------------------
 * 	  |     Min   	|	  67MB      |	  16MB      |
 * 	  |     Max  	|	 134MB      |	  67MB      |
 * 	  -----------------------------------------------
 *
 *************************************************************************************/

/*
 *  Structure to define FAT queue
 */
typedef enum
{
    FAT_QUEUE_MAX_CLUSTERS = 4096,
    NODES_QUEUE_TAIL_INIT = 0,
} FAT_Queue_t;

/*
 *  Structure to for FAT processing
 */
typedef struct
{
    uint32_t ClusterID;
    uint32_t ContBlocks;
} FAT_Queue_data_t;

typedef enum
{
    FAT_FREE_ID_MARKER = 0x0000,
    FAT_EOF_MARKER_FAT16 = 0xFFF8,
    // FAT32 Max cluster: 2^28
    FAT_EOF_MARKER_FAT32 = 0xFFFFFF8,
    // Used for both FAT16 & FAT32 Table Updates
    FAT_EOF_MARKER_GENERIC = 0xFFFFFFF,
} fat_table_markers_t;


// Fload Return type
typedef enum
{
    FLOAD_FAIL = 0,
    FLOAD_NOP,
    FLOAD_EOF_NOT_FOUND,
    FLOAD_EOF_FOUND,
} fat_fload_t;


/**********************************
 *  Macros to get Time and Date   *
 **********************************/

// Bit field Structure for time conversion
typedef struct
{
    uint16_t Seconds : 5;
    uint16_t Minutes : 6;
    uint16_t Hours : 5;
} Time_format_t, *pTimeformat;

// Bit field Structure for date conversion
typedef struct
{
    uint16_t Day : 5;
    uint16_t Month : 4;
    uint16_t Year : 7;
} Date_format_t, *pDateFormat;

#define START_YEAR 1980

// Time Conversion Macros
#define GetFileHours(x) (((pTimeformat)&x)->Hours)
#define GetFileMinutes(x) (((pTimeformat)&x)->Minutes)
#define GetFileSeconds(x) (((pTimeformat)&x)->Seconds)
// Date Conversion Macros
#define GetFileYear(x) (((pDateFormat)&x)->Year + START_YEAR)
#define GetFileMonth(x) (((pDateFormat)&x)->Month)
#define GetFileDay(x) (((pDateFormat)&x)->Day)


/**** Macros for Master Boot Record ****/
typedef enum
{
    MBR_TYPE_CODE = 0x01C2,
    MBR_LBA = 0x01C6,
    MBR_END_MARKER = 0x01FE,
} mbr_offsets_t;


/***************************************************
 * 				Global Variables    			   *
 ***************************************************/
FAT_Handle_t FAT = {0};

// Buffer to keep File Construction Stack
static uint32_t FileNodesBuf[FAT_QUEUE_MAX_CLUSTERS];

/***************************************************
 * 				Static Functions    			   *
 ***************************************************/
static void getSystemInfo(FAT_Handle_t* pFAT);

// Remove the padded spaces from a file entry name & entension
static void removeSpacePadding(uint8_t* text, uint8_t fieldSize);
// Parse File name and extension
static uint8_t parseFilename(char* FullName, uint8_t* Filename, uint8_t* FileExt);
static uint8_t getShortFilename(file_entry_t* file, uint8_t* filename);

static fat_fload_t loadFileNodesQueue(FAT_Handle_t* pFAT, file_entry_t* file, file_mode_t mode);
static fat_fload_t loadFreeClusterIDs(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster);

static void setCurFile(FAT_Handle_t* pFAT, file_entry_t* file);
static bool isEndofFatEntry(FAT_Handle_t* pFAT, uint32_t nextCluster);
static WorkingAddr_t getTableBlockAddr(FAT_Handle_t* pFAT, uint32_t ClusterID);
static uint32_t getNextClusterID(FAT_Handle_t* pFAT, uint32_t offset);
static fat_fwrite_t updateClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t nextID);
static uint32_t findNextFreeClusterID(FAT_Handle_t* pFAT, uint32_t clusterID);
static fat_fload_t traverseTable(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster, fat_traverse_mode_t mode);
static WorkingAddr_t getWorkingAddr(FAT_Handle_t* pFAT);

static bool isEndOfDir(file_entry_t* file);
static bool isLongFile(file_entry_t* file);
static bool hasFileFlag(file_entry_t* file, FAT_file_flags_t flag);

// Create new file
static fat_open_t createFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file);
static Search_Status_t findFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode);
static Search_Status_t findDirectory(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode);
static Search_Status_t searchPath(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file);
static fat_fwrite_t updateDirEntry(FAT_Handle_t* pFAT, file_entry_t* file);



/****************************************************************************************
 *	@fn 			     - InitFAT
 *
 * 	@brief			     - Initialize the FAT
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[pSDHandle]	 - Handler structure for SD
 *
 * 	@return			     - none
 *
 * 	@note
 */
fat_status_t InitFAT(FAT_Handle_t* pFAT, SD_Handle_t* pSDHandle)
{
    // Link SD handler to FAT handler
    pFAT->pSDHandle = pSDHandle;

    // Init SD Card
    SD_States_t status = SD_Init(pSDHandle);

    if (status != SD_STATE_READY)
    {
        pFAT->FAT_Stat = INIT_FAT_FAIL;
        return pFAT->FAT_Stat;
    }

    // Read FAT system parameters from Boot sector
    getSystemInfo(pFAT);

    // Set Root directory as working directory
    if (FAT_getStat(pFAT) == INIT_FAT_FAIL)
    {
        FAT_SetWorkingAddr(pFAT, 0, 0);
    }

    return pFAT->FAT_Stat;
}

/****************************************************************************************
 *	@fn 			     - getSystemInfo
 *
 * 	@brief			     - Get information for FAT
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 *
 * 	@return			     - none
 *
 * 	@note
 */
static void getSystemInfo(FAT_Handle_t* pFAT)
{
    if (SD_IsReady(pFAT->pSDHandle) == false)
    {
        pFAT->FAT_Stat = INIT_FAT_FAIL;
        return;
    }

    System_info_t* SystemInfo = &pFAT->SystemInfo;
    uint8_t* buff = SD_GetBuffAddr(pFAT->pSDHandle);

    /******************		  Read Master Boot Record (MBR) 	   **************/
    /* The MBR is the first sector of the drive. This sector contains boot code *
        * and a partition table. The contents of a FAT file system are located in  *
        * the first partition. In the partition description, We only care about    *
        * the logical block address (LBA) and the	partition type 					*/

    sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, 0, 1);

    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return;
    }

    /* First find the start of the the File system */
    uint32_t LogicalBlockAddress = ToLittleEndian(&buff[MBR_LBA], DATA_SIZE_WORD);

    uint8_t typeCode = buff[MBR_TYPE_CODE];

    /* Check Partition Type */
    if (typeCode == MBR_TYPE_CODE_FAT16)
    {
        SystemInfo->FAT_Type = FAT_TYPE_FAT16;
    }
    else if (typeCode == MBR_TYPE_CODE_FAT32_A || typeCode == MBR_TYPE_CODE_FAT32_B)
    {
        SystemInfo->FAT_Type = FAT_TYPE_FAT32;
    }
    else
    {
        pFAT->FAT_Stat = INIT_NO_FAT;
        return;
    }

    if (ToLittleEndian(&buff[MBR_END_MARKER], DATA_SIZE_HALF_WORD) != BOOT_SIGNATURE)
    {
        pFAT->FAT_Stat = INIT_BAD_END_MARKER;
        return;
    }

    /******************	 Parse out parameters from Boot Block	 ****************/
    uint32_t reservedSectors;
    uint32_t bootBlockAddr;

    // FAT16 uses a byte address scheme, while FAT32 block address
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        bootBlockAddr = LogicalBlockAddress * SD_DEFAULT_BLOCK_SIZE;
    }
    else if (getFatType(pFAT) == FAT_TYPE_FAT32)
    {
        bootBlockAddr = LogicalBlockAddress;
    }

    cmdStatus = SD_ReadBlock(pFAT->pSDHandle, bootBlockAddr, 1);

    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return;
    }

    SystemInfo->BytesPerSector = ToLittleEndian(&buff[BOOT_BLK_BYTES_PER_SEC], DATA_SIZE_HALF_WORD);
    reservedSectors = ToLittleEndian(&buff[BOOT_BLK_RESERVED_SECT], DATA_SIZE_HALF_WORD);
    SystemInfo->SectorsPerCluster = (uint32_t)buff[BOOT_BLK_SEC_PER_CLUSTER];
    SystemInfo->FAT_Copies = (uint32_t)buff[BOOT_BLK_NUM_FATS];
    SystemInfo->RootDirEntries = ToLittleEndian(&buff[BOOT_BLK_NUM_ROOT_ENTRIES], DATA_SIZE_HALF_WORD);
    SystemInfo->SectorsPerVolume = ToLittleEndian(&buff[BOOT_BLK_TOTAL_VOLUME_BLOCKS], DATA_SIZE_WORD);

    // Sectors Per FAT
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        SystemInfo->SectorsPerFAT = ToLittleEndian(&buff[BOOT_BLK_SECTORS_PER_TABLE_FAT16], DATA_SIZE_HALF_WORD);
    }
    else if (getFatType(pFAT) == FAT_TYPE_FAT32)
    {
        SystemInfo->SectorsPerFAT = ToLittleEndian(&buff[BOOT_BLK_SECTORS_PER_TABLE_FAT32], DATA_SIZE_HALF_WORD);
    }

    uint32_t addrUnit = getFatAddrUnit(pFAT);

    /**********   Calculate the addresses of FATs and Root directory   **********/
    SystemInfo->FAT1_Address = bootBlockAddr + (reservedSectors * addrUnit);
    SystemInfo->FAT2_Address = SystemInfo->FAT1_Address + (SystemInfo->SectorsPerFAT * addrUnit);
    SystemInfo->RootDirAddress = bootBlockAddr + (reservedSectors * addrUnit + SystemInfo->SectorsPerFAT * addrUnit * SystemInfo->FAT_Copies);
    /* Data Starting address (Cluster 2) */
    SystemInfo->DataStartAddress = SystemInfo->RootDirAddress + (SystemInfo->RootDirEntries * DIR_BYTES_PER_ENTRY);

    /*  The boot sector has an entry reserved for the Volume label. However, Microsoft ignored their own recommendation and
        *  leaves the field set to the string “NO NAME ”, which is the default for when the volume label has not been set.*/
    cmdStatus = SD_ReadBlock(pFAT->pSDHandle, SystemInfo->RootDirAddress, 1);

    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return;
    }

    /* Instead we will get the volume label from the first in the root directory. This is a special item just for this. */
    strncpy((char*)SystemInfo->VolumeLabel, (char*)(buff + FILENAME), VOLUME_LABEL_SIZE);
    removeSpacePadding(SystemInfo->VolumeLabel, VOLUME_LABEL_SIZE);

    /* Update the FAT Handler Status */
    pFAT->FAT_Stat = INIT_FAT_SUCCESS;

    /* Re-init the FAT to the root dir and set working dir */
    FAT_GoToRootDir(pFAT);
}

/****************************************************************************************
 *	@fn 			     - FAT_ScanDir
 *
 * 	@brief			     - Parse a file information from a directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[file]		 - Memory location to store file details
 *
 * 	@return			     - False if we reached the end of directory. True otherwise
 *
 */
bool FAT_ScanDir(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // These local variables are used to make function statements shorter
    uint32_t rxBufferSize = SD_GetBuffSize(pFAT->pSDHandle);
    uint8_t* entryAddr = SD_GetBuffAddr(pFAT->pSDHandle);
    WorkingAddr_t currAddr = getWorkingAddr(pFAT);
    sd_read_write_t cmdStatus = SD_READ_WRITE_FAIL;

    // Read new block if necessary
    if (currAddr.offset >= rxBufferSize)
    {
        // Increment the base address to the next sector
        if (getFatType(pFAT) == FAT_TYPE_FAT16)
        {
            currAddr.baseAddr += rxBufferSize;
        }
        else
        {
            currAddr.baseAddr++;
        }

        currAddr.offset = 0;

        cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currAddr.baseAddr, rxBufferSize / pFAT->SystemInfo.BytesPerSector);

        // Read Error Handling
        if (cmdStatus == SD_READ_WRITE_FAIL)
        {
            // Report the end of directory was found
            return false;
        }
    }

    // The first byte of the filename have special value.
    // Skip over any deleted files.
    while (*(entryAddr + currAddr.offset + FILENAME) == FILENAME_DELETED)
    {
        // Bump the pointer for the new entry
        currAddr.offset += DIR_BYTES_PER_ENTRY;
    }

    // Get File Attribute
    file->FileAttribute = *(entryAddr + currAddr.offset + FILE_ATTRIBUTE);

    // Processing a short file name
    if (!isLongFile(file))
    {
        // Clear the file name incase of previous scans
        memset(file->Filename, 0, FILENAME_SIZE + 1);

        // Copy in Filename
        strncpy((char*)file->Filename, (char*)(entryAddr + currAddr.offset + FILENAME), FILENAME_SIZE);
        removeSpacePadding(file->Filename, FILENAME_SIZE);

        // Copy in File Extension
        strncpylower(file->FileExt, (entryAddr + currAddr.offset + FILE_EXTENSION), FILE_EXT_SHORT_SIZE);
        removeSpacePadding(file->FileExt, FILE_EXT_SHORT_SIZE);
    }
    // Otherwise we need to process a long file name (VFAT)
    else
    {

        /********************* 	 Process entries of the LFN 	**********************/

        /* Create a temporary stack to hold the long file name entries */
        typedef struct
        {
            uint8_t longFilename[FILENAME_LF_SIZE + 1];
        } LFN_t;

        LFN_t filenameBuf[MAX_LFN_ENTRIES];
        LFN_t temp;

        // Init LFN entry stack
        StackInfo nameStack = initStack(filenameBuf, MAX_LFN_ENTRIES, sizeof(LFN_t));

        // The Ordinal field of the first LFN entry will inform us how many entries we need to process
        uint8_t entriesToProcess = LowerNibble(*(entryAddr + currAddr.offset));

        // Keep the LFN Unicode character offsets in an array for quick access
        uint8_t entryCharOffsets[FILENAME_LF_SIZE] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1C, 0x1E};
        uint16_t unicodeChar;
        uint8_t n, k;

        /* The LFN is read in groups of 13 unicode characters. The entries are read in reverse order, so the first entry
		 * will be the last 13 bytes of the name. The filename is terminated by 0x0000 then followed by 0xFFFFs          */
        for (n = 0; (n < entriesToProcess && n < MAX_LFN_ENTRIES); n++)
        {
            //  Copy the LFN entry temp storage
            for (k = 0; k < FILENAME_LF_SIZE; k++)
            {
                unicodeChar = ToLittleEndian(entryAddr + currAddr.offset + entryCharOffsets[k], DATA_SIZE_HALF_WORD);
                temp.longFilename[k] = LOBYTE(unicodeChar);
            }
            // Terminate the end of the string
            temp.longFilename[k] = '\0';

            // Add the LFN entry to the stack
            pushStack(&nameStack, &temp);

            // Bump the pointer for the new entry
            currAddr.offset += DIR_BYTES_PER_ENTRY;

            // Read new block if necessary
            if (currAddr.offset >= rxBufferSize)
            {
                // Increment the base address to the next sector
                if (getFatType(pFAT) == FAT_TYPE_FAT16)
                {
                    currAddr.baseAddr += rxBufferSize;
                }
                else
                {
                    currAddr.baseAddr++;
                }

                currAddr.offset = 0;

                cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currAddr.baseAddr, rxBufferSize / pFAT->SystemInfo.BytesPerSector);

                // Read Error Handling
                if (cmdStatus == SD_READ_WRITE_FAIL)
                {
                    // Report the end of directory was found
                    return false;
                }
            }
        }

        // Terminate the first character so the string functions work properly
        file->Filename[0] = '\0';
        while (!isStackEmpty(&nameStack))
        {
            popStack(&nameStack, &temp);
            // Copy Full Name Back into the file structure
            strncat((char*)file->Filename, (char*)temp.longFilename, FILENAME_LF_SIZE + 1);
        }

        /********************* 	 Get File Extension from the LFN 	**********************/
        uint8_t* pstart = file->Filename + (strlen((char*)file->Filename) - FILE_EXT_LONG_SIZE - 1);
        bool periodFound = false;
        uint8_t extLen = 0;

        file->FileExt[0] = '\0';

        /* Search through the last few characters for a period. If we find it assume *
		 * the following characters are the file extension. 					     */
        for (n = 0; (n < FILE_EXT_LONG_SIZE && !periodFound); n++, pstart++)
        {
            if (*pstart == '.')
            {
                // Set the found period flag
                periodFound = true;
                // Remove the file extension from the filename
                *pstart = '\0';
                // Bump the pointer to the next char
                pstart++;
                extLen = strlen((char*)pstart);
                strncpylower(file->FileExt, pstart, extLen);
                // Terminate the the end of the file extension
                file->FileExt[extLen] = '\0';
            }
        }

        /*********************************
		 ***  TODO: Process checksum  ****
		 *********************************/
    }

    /***************	Parse File Meta data	***************/

    /* Get File Attribute. This must be obtained again for the actual file
	 * metadata because the previous one could have been for the LFN entry	*/
    file->FileAttribute = *(entryAddr + currAddr.offset + FILE_ATTRIBUTE);
    // Get Time Created or Last Updated
    file->TimeModified = ToLittleEndian(entryAddr + currAddr.offset + FILE_TIME_MODIFIED, FILE_TIME_MODIFIED_SIZE);
    // Get Date Created or Last Updated
    file->DateModified = ToLittleEndian(entryAddr + currAddr.offset + FILE_DATE_MODIFIED, FILE_DATE_MODIFIED_SIZE);
    // Get File Size
    file->FileSize = ToLittleEndian(entryAddr + currAddr.offset + FILE_SIZE, FILE_SIZE_SIZE);

    /***************	Save Directory  Block address and offset	***************/
    // Note: These are calculated this way since its possible to read with multi-block buffer

    // Offset into directory block
    file->DirEntryOffset = currAddr.offset % SD_DEFAULT_BLOCK_SIZE;

    // Base address of block
    file->DirEntryBaseAddr = currAddr.baseAddr + ((currAddr.offset / SD_DEFAULT_BLOCK_SIZE) * getFatAddrUnit(pFAT));

    // Get File Starting Cluster
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        file->StartingCluster = ToLittleEndian(entryAddr + currAddr.offset + FILE_START_CLUSTER_LO, FILE_START_CLUSTER_SIZE);
    }
    else if (getFatType(pFAT) == FAT_TYPE_FAT32)
    {
        uint32_t startClusterHi = ToLittleEndian(entryAddr + currAddr.offset + FILE_START_CLUSTER_HI, FILE_START_CLUSTER_SIZE);
        uint32_t startClusterLo = ToLittleEndian(entryAddr + currAddr.offset + FILE_START_CLUSTER_LO, FILE_START_CLUSTER_SIZE);

        file->StartingCluster = MAKEWORD(startClusterHi, startClusterLo);
    }

    // Bump the pointer for the new entry
    currAddr.offset += DIR_BYTES_PER_ENTRY;

    // Update the working directory info
    FAT_SetWorkingAddr(pFAT, currAddr.baseAddr, currAddr.offset);

    return !isEndOfDir(file);
}

/****************************************************************************************
 *	@fn 			     - findFile
 *
 * 	@brief			     - Parse a file information from a directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[fileName]	 - name of the file
 * 	@param[file]		 - Memory location the file details
 * 	@param[mode]		 - search mode
 *
 * 	@return			     - Search status
 *
 */
static Search_Status_t findFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode)
{
    uint32_t addrQueueBuf[MAX_DIRECTORIES];
    uint16_t entriesPerDir = MAX_ENTIRES_PER_DIRECTORY;
    uint32_t workingDir = 0;
    uint32_t tempAddr = 0;
    sd_read_write_t cmdStatus = SD_READ_WRITE_FAIL;

    // Initialize a queue for the directory search
    QueueInfo addrQueue = initQueue(addrQueueBuf, MAX_DIRECTORIES, sizeof(uint32_t));

    // Set the Working address with the value on the top of the stack
    FAT_SetWorkingAddr(pFAT, FAT_GetWorkingDir(pFAT), 0);

    // Enqueue the starting address
    enqueue(&addrQueue, &pFAT->WorkingAddr.baseAddr);

    // temporary file to hold just the name and extension
    uint8_t Filename[FILENAME_MAX_SIZE + 1] = {0};
    uint8_t FileExt[FILE_EXT_LONG_SIZE + 1] = {0};

    // Parse the filename and extension from the user input
    parseFilename((char*)fileName, Filename, FileExt);

    // Loop until the queue is empty
    do
    {
        uint16_t offset = 0;

        // Reset entry count each directory
        uint32_t entryCount = 0;

        // Get the working address from the front of the queue
        peekQueue(&addrQueue, &workingDir);

        // Read the first blocks of the working directory
        cmdStatus = SD_ReadBlock(pFAT->pSDHandle, workingDir, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo.BytesPerSector);

        // Read Error Handling
        if (cmdStatus == SD_READ_WRITE_FAIL)
        {
            // Report Issue
            return FILESEARCH_FAIL;
        }

        // If we are in the root directory there is a limit to the number of entries
        if (getFatType(pFAT) == FAT_TYPE_FAT16)
        {
            entriesPerDir = (workingDir == pFAT->SystemInfo.RootDirAddress) ? pFAT->SystemInfo.RootDirEntries : MAX_ENTIRES_PER_DIRECTORY;
        }

        FAT_SetWorkingAddr(pFAT, workingDir, offset);

        // Scan Until we find the end of the directory
        while ((entryCount < entriesPerDir) && FAT_ScanDir(pFAT, file))
        {
            // Increment the entry count
            entryCount++;

            // Match the file names
            if (strcmp((char*)file->Filename, (char*)Filename) == 0 && strcmp((char*)file->FileExt, (char*)FileExt) == 0)
            {
                // Found the file, so exit the function
                return FILESEARCH_FOUND;
            }

            // If we have found a new directory, add it to the queue
            if (hasFileFlag(file, FILE_FLAG_DIRECTORY) && (mode == SEARCH_DIR_RECURSIVE || mode == SEARCH_FILE_RECURSIVE))
            {
                tempAddr = FAT_GetClusterAddr(pFAT, file->StartingCluster);
                enqueue(&addrQueue, &tempAddr);
            }
        }

        // queue the directory at the head of the queue
        dequeue(&addrQueue, &tempAddr);

    } while (!isQueueEmpty(&addrQueue));

    // Return file not found
    return FILESEARCH_NOT_FOUND;
}

/****************************************************************************************
 *	@fn 			     - findDirectory
 *
 * 	@brief			     - Parse a file information from a directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[fileName]	 - name of the file
 * 	@param[file]		 - Memory location the file details
 * 	@param[mode]		 - Search mode
 *
 * 	@return			     - Search status
 *
 */
Search_Status_t findDirectory(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode)
{
    // Search for file
    Search_Status_t searchStatus = findFile(pFAT, fileName, file, mode);

    if (searchStatus == FILESEARCH_FOUND && hasFileFlag(file, FILE_FLAG_DIRECTORY))
    {
        return FILESEARCH_FOUND;
    }

    if (searchStatus == FILESEARCH_FAIL)
    {
        return FILESEARCH_FAIL;
    }

    return FILESEARCH_NOT_FOUND;
}

/****************************************************************************************
 *	@fn                  - searchPath
 *
 * 	@brief               - Search for a file in a given path
 *
 * 	@param[pFAT]         - Handler structure for FAT
 * 	@param[fileName]	 - name of the file
 * 	@param[file]         - Memory location the file details
 *
 * 	@return	             - Search status
 *
 */
static Search_Status_t searchPath(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file)
{
    uint8_t n, delimeterLoc;
    uint8_t tempBuf[FILENAME_MAX_SIZE + 1];
    uint32_t currDir = FAT_GetWorkingDir(pFAT);
    uint8_t nameLen = strlen((char*)fileName);

    // If the name ends with a forward-slash remove it
    if (fileName[nameLen - 1] == '/')
    {
        fileName[nameLen - 1] = '\0';
    }

    if (strncmp((char*)fileName, "~/", 2) == 0)
    {
        fileName += 2;
        nameLen -= 2;
        FAT_GoToRootDir(pFAT);
    }

    // Loop through the input and search for sub directories
    for (n = 0, delimeterLoc = 0; n < nameLen; n++)
    {
        // Look for the delimiter
        if (fileName[n] == '/')
        {
            if (delimeterLoc != n)
            {
                strncpy((char*)tempBuf, (char*)&fileName[delimeterLoc], n - delimeterLoc);
                tempBuf[n - delimeterLoc] = '\0';

                // Search for the file or directory
                if (findDirectory(pFAT, tempBuf, file, SEARCH_DIR_LOCAL) == FILESEARCH_FOUND)
                {
                    // Change Working Directory to found sub-directory
                    FAT_SetWorkingDir(pFAT, FAT_GetClusterAddr(pFAT, file->StartingCluster));
                }
                else
                {
                    // Fail, go back to original directory
                    FAT_SetWorkingDir(pFAT, currDir);
                    return FILESEARCH_DIR_NOT_FOUND;
                }
            }

            // Update delimiter location
            delimeterLoc = n + 1;
        }
    }

    // Handle the case when the string ends in a filename
    // Ex: Recipes/Baked Beans.txt
    if (delimeterLoc != n)
    {
        strncpy((char*)tempBuf, (char*)&fileName[delimeterLoc], n - delimeterLoc);
        tempBuf[n - delimeterLoc] = '\0';

        // Search for the file
        if (findFile(pFAT, tempBuf, file, SEARCH_FILE_LOCAL) != FILESEARCH_FOUND)
        {
            // Fail, go back to original directory
            FAT_SetWorkingDir(pFAT, currDir);
            return FILESEARCH_NOT_FOUND;
        }
    }

    // Go back to original directory
    FAT_SetWorkingDir(pFAT, currDir);
    return FILESEARCH_FOUND;
}

/****************************************************************************************
 *  @fn                  - updateDirEntry
 *
 *  @brief               - Update a directory entry after a fwrite
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 *  @return	             - Search status
 *                 * // TODO: updateFileSize
 *
 */
static fat_fwrite_t updateDirEntry(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // Confirm the current file is being closed
    if ((pFAT->CurrFile != file) || (file->mode != FILE_MODE_WRITE && file->mode != FILE_MODE_WRITE_NEW))
    {
        return FWRITE_FAIL;
    }

    sd_read_write_t cmdStatus = SD_READ_WRITE_FAIL;
    uint8_t* entryBlock = SD_GetBuffAddr(pFAT->pSDHandle);
    uint8_t* entryfileSize = entryBlock + file->DirEntryOffset + FILE_SIZE;

    // First read the block that holds the file Directory Entry
    cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, 1);

    // Read Error Handling
    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        // Report Issue
        return FWRITE_FAIL;
    }

    // TODO: Remove these debug
#if defined(FAT_DEBUG_GENERIC)
    HexdumpBuffer(entryBlock + file->DirEntryOffset, DIR_BYTES_PER_ENTRY);
#endif

    if (file->FileSize == ToLittleEndian(entryfileSize, FILE_SIZE_SIZE))
    {
        // Nothing has changed
        return FWRITE_NOP;
    }

    ToEndianBuf(entryfileSize, file->FileSize, DATA_SIZE_WORD);

#if defined(FAT_DEBUG_GENERIC)
    HexdumpBuffer(entryBlock + file->DirEntryOffset, DIR_BYTES_PER_ENTRY);
#endif

    // Write updates to the Directory block
    cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, 1);

    return (cmdStatus == SD_READ_WRITE_FAIL) ? FWRITE_FAIL : FWRITE_SUCCESS;
}

/****************************************************************************************
 *  @fn                  - createFile
 *
 *  @brief               - Create a file
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 *  @return              - Search status
 * 
 *                       -
 */
static fat_open_t createFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file)
{
    // Search for the file
    Search_Status_t searchStat = searchPath(pFAT, fileName, file);

    // Report error if the file already exists or the directory does not
    if ((searchStat == FILESEARCH_FOUND) || (searchStat == FILESEARCH_DIR_NOT_FOUND))
    {
        return FOPEN_NOP;
    }

    /***********  Create a new entry in the FAT table  ***********/

    // First Find the next free ClusterID. Start at the first valid clusterID
    file->StartingCluster = findNextFreeClusterID(pFAT, 2);

#ifndef FAT_DEBUG_GENERIC
    // Create the new entry in the FAT
    if (updateClusterID(pFAT, file->StartingCluster, file->StartingCluster) == FWRITE_FAIL)
    {
        return FOPEN_FAIL;
    }
#endif

    /***********  Create new directory entry  ***********/

    // Grab the current buffer for later
    uint8_t* entryAddr = SD_GetBuffAddr(pFAT->pSDHandle) + file->DirEntryOffset;

    // Parse the filename and extension from the user input and remove path from fileName
    fileName += parseFilename((char*)fileName, file->Filename, file->FileExt);

    // Create the 8.3 filename from the input
    uint8_t shortFilename[FILENAME_SIZE + FILE_EXT_SHORT_SIZE + 1] = {0};
    uint8_t checksum = getShortFilename(file, shortFilename);

    /***********  Write Long Filename Entries ***********/

    // Read the Directory block
    sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, 1);

    // Keep the LFN Unicode character offsets in an array for quick access
    uint8_t entryCharOffsets[FILENAME_LF_SIZE] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1C, 0x1E};

    int8_t nameLen = strlen((char*)fileName);

    uint8_t lfnEnties = (nameLen + FILENAME_LF_SIZE - 1) / FILENAME_LF_SIZE;

    char temp[FILENAME_LF_SIZE + 1];

    // The Ordinal field of the first LFN entry must have 0x4 in the upper nibble
    *(entryAddr + LFN_ORDINAL) = 0x40;

    // Loop until the whole name has been processed
    while (nameLen > 0)
    {
        uint8_t lenMod = nameLen % FILENAME_LF_SIZE;

        // Determine how many bytes need to be copied into the LFN
        uint8_t entryLen = (lenMod == 0) ? FILENAME_LF_SIZE : lenMod;

        // Decrement the name length
        nameLen -= entryLen;

        // Copy the entry to the temp buffer
        strncpy(temp, (char*)fileName + nameLen, entryLen);

        uint8_t n;

        // Copy in Entry contents
        for (n = 0; n < FILENAME_LF_SIZE; n++)
        {
            if (n < entryLen)
            {
                // Copy in current character
                *(entryAddr + entryCharOffsets[n]) = *(temp + n);
            }
            else if (n == entryLen)
            {
                // End of String is terminated with 0x0000
                *((uint16_t*)(entryAddr + entryCharOffsets[n])) = 0x0000;
            }
            else
            {
                // The rest of the unicode chars are 0xFFFF
                *((uint16_t*)(entryAddr + entryCharOffsets[n])) = 0xFFFF;
            }
        }

        /** Copy in the checksum byte **/
        *(entryAddr + LFN_CHECKSUM) = checksum;

        /** Update the ordinal field **/
        *(entryAddr + LFN_ORDINAL) |= lfnEnties;

        /** Update the attribute to LFN flag **/
        *(entryAddr + FILE_ATTRIBUTE) = FILE_FLAG_LFN;

        // Update the current directory offset
        if (file->DirEntryOffset < (SD_GetBuffSize(pFAT->pSDHandle) - DIR_BYTES_PER_ENTRY))
        {
            // Increment the directory entry offset
            file->DirEntryOffset += DIR_BYTES_PER_ENTRY;
        }
        else
        {

#ifndef FAT_DEBUG_GENERIC
            // Write the block to memory
            cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, 1);
#else
            // Debug the new entry
            HexdumpBuffer(SD_GetBuffAddr(pFAT->pSDHandle), pFAT->SystemInfo.BytesPerSector);
#endif
            // Reset the directory entry
            file->DirEntryOffset = 0;

            // Increment the directory base address
            uint32_t sectorsPerBuffer = SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo.BytesPerSector;
            file->DirEntryBaseAddr += sectorsPerBuffer * getFatAddrUnit(pFAT);

            // Read next block of memory
            cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, 1);
        }

        // Grab the current buffer for later
        entryAddr = SD_GetBuffAddr(pFAT->pSDHandle) + file->DirEntryOffset;

        // Decrement the entry count
        lfnEnties--;
    }

    /***********  Short Filename Entry Information ***********/

    // Copy in the filename & extension in 8.3 format
    strncpy((char*)entryAddr + FILENAME, (char*)shortFilename, FILENAME_SIZE + FILE_EXT_SHORT_SIZE);

    // Fill in the starting cluster
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_LO, file->StartingCluster, FILE_START_CLUSTER_SIZE);
    }
    else
    {
        // TODO: Debug this.
        // Grab lower Portion of the word
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_LO, (file->StartingCluster & 0xFFFF), FILE_START_CLUSTER_SIZE);

        // Grab uower Portion of the word
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_HI, ((file->StartingCluster >> 16) & 0xFFFF), FILE_START_CLUSTER_SIZE);
    }

    // Attribute Byte set to archived.
    *(entryAddr + FILE_ATTRIBUTE) |= FILE_FLAG_ARCHIVED;

    // Starting Date for the file Jan 1, 1980
    Date_format_t date = {1, 1, 0};

    *((pDateFormat)(entryAddr + FILE_DATE_MODIFIED)) = date;

    // Calculate the number of blocks to write
    uint8_t writeCount = (file->DirEntryOffset < pFAT->SystemInfo.BytesPerSector) ? 1 : 2;

#ifndef FAT_DEBUG_GENERIC
    // Update the block
    cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->DirEntryBaseAddr, writeCount);
#else
    // Debug the new entry
    HexdumpBuffer(SD_GetBuffAddr(pFAT->pSDHandle), pFAT->SystemInfo.BytesPerSector * writeCount);
#endif

    return (cmdStatus == SD_READ_WRITE_FAIL) ? FOPEN_FAIL : FOPEN_SUCCESS;
}

/****************************************************************************************
 *  @fn                - getShortFilename
 *
 *  @brief             - Calculate filename checksum based on 8.3 filename
 *
 *  @param[file]       - Memory location the file details
 *  @param[filename]   - Filename in 8.3 format  
 *
 *  @return            - Checksum
 *
 *  @note              - 
 */
static uint8_t getShortFilename(file_entry_t* file, uint8_t* filename)
{
    // First copy the filename end extension into 8.3 format
    strncpy((char*)filename, (char*)file->Filename, FILENAME_SIZE);

    uint8_t nameLen = strlen((char*)file->Filename);

    // If the name exceeds the max characters
    if (nameLen > FILENAME_SIZE)
    {
        filename[6] = '~';
        filename[7] = '1';
    }
    else
    {
        // Pad in the rest of the name with spaces
        memset(filename + nameLen, ' ', FILENAME_SIZE - nameLen);
    }

    // Terminal for further string handling
    filename[FILENAME_SIZE] = '\0';

    // Copy on the extension
    strncat((char*)filename, (char*)file->FileExt, FILENAME_SIZE + FILE_EXT_SHORT_SIZE);

    // Update name to upper case
    strntoUpper(filename, FILENAME_SIZE + FILE_EXT_SHORT_SIZE);

    //uint8_t len = strlen((char*)filename);

    // TODO: Confirm checksum of short file like foobar.txt
    uint8_t n, sum, rotatedDec;

    for (n = 0, rotatedDec = 0, sum = 0; n < FILENAME_SIZE + FILE_EXT_SHORT_SIZE; n++, filename++)
    {
        // Calculate the rolling sum
        sum = *filename + rotatedDec;

        // Shift Evertthing down by one and wrap around the LSB
        rotatedDec = (sum << 7) + (sum >> 1);
    }

    return sum;
}

/***************************************************************************************
 *                              File Application Functions                             *      
 ***************************************************************************************/

/****************************************************************************************
 *	@fn                  - FAT_fopen
 *
 * 	@brief               - Find a file and load the NodesQueue up
 *
 * 	@param[pFAT]         - Handler structure for FAT
 * 	@param[fileName]     - name of the file
 * 	@param[file]         - Memory location the file details
 * 	@param[mode]         - File Mode
 *
 * 	@return              - Search status
 *
 */
fat_open_t FAT_fopen(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, file_mode_t mode)
{
    // Set our current working file
    setCurFile(pFAT, file);

    // Error Checking
    if ((FAT_getStat(pFAT) != INIT_FAT_SUCCESS) || (mode != FILE_MODE_READ && mode != FILE_MODE_WRITE && mode != FILE_MODE_WRITE_NEW))
    {
        return FOPEN_NOP;
    }

    // This mode creates a new file if it does not exist
    if (mode == FILE_MODE_WRITE_NEW)
    {
        if (createFile(pFAT, fileName, file) != FOPEN_SUCCESS)
        {
            return FOPEN_FAIL;
        }
    }

    // Search for the file
    if (searchPath(pFAT, fileName, file) != FILESEARCH_FOUND)
    {
        return FOPEN_NOT_FOUND;
    }

    fat_fload_t loadStatus = FOPEN_FAIL;

    file->mode = mode;
    file->state = FILE_STATE_IDLE;

    /*******************************************
     *   Found the file, so init NodeQueue     *
     *******************************************/

    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->NodesQueue;

    // Init Queue to hold the ClustersIds of the file
    pNodesQueue->Info = initQueue(FileNodesBuf, FAT_QUEUE_MAX_CLUSTERS, sizeof(uint32_t));

    // Initialized the tail to Zero which is not an acceptable clusterID
    pNodesQueue->Tail = NODES_QUEUE_TAIL_INIT;

    // Load Up the NodesQueue
    loadStatus = loadFileNodesQueue(pFAT, file, mode);

    // Return Types
    if ((loadStatus == FLOAD_EOF_FOUND) || (loadStatus == FLOAD_EOF_NOT_FOUND))
    {
        return FOPEN_SUCCESS;
    }

    return FOPEN_FAIL;
}

/****************************************************************************************
 *	@fn 			     - loadFileNodesQueue
 *
 * 	@brief			     - Read from the FAT table and load up the nodes queue
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 * 	@return			     - Search status
 *
 */
fat_fload_t loadFileNodesQueue(FAT_Handle_t* pFAT, file_entry_t* file, file_mode_t mode)
{
    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->NodesQueue;

    // Error Checking
    // TODO: Determine if this error handling needs updates
    if (isEndofFatEntry(pFAT, pNodesQueue->Tail) || isQueueFull(&pNodesQueue->Info) || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS))
    {
        return FLOAD_NOP;
    }

    uint32_t currClusterID;

    // For now, we will set the working directory to the root directory.
    // getNextClusterID() will calculate which block needs to be read to get the next queue node
    FAT_SetWorkingAddr(pFAT, pFAT->SystemInfo.RootDirAddress, 0);

    // The file has just been opened, determine the starting ClusterID
    if (pNodesQueue->Tail == NODES_QUEUE_TAIL_INIT)
    {
        if (mode == FILE_MODE_WRITE || mode == FILE_MODE_WRITE_NEW)
        {
            // Find the last cluster ID in the file
            if (traverseTable(pFAT, pNodesQueue, file->StartingCluster, FAT_TRAVERSE_FIND_LAST_ID) == FLOAD_FAIL)
            {
                return FLOAD_FAIL;
            }

            // Starting from the End of the file
            currClusterID = file->EndingCluster;

            // Determine if the end of the file is mid-cluster
            // TODO: This may need some updates
            if (file->FileSize % (pFAT->SystemInfo.BytesPerSector * pFAT->SystemInfo.SectorsPerCluster))
            {
                enqueue(&pNodesQueue->Info, &currClusterID);
            }
        }
        else
        {
            // Starting from the begining of the file
            currClusterID = file->StartingCluster;
        }
    }
    else
    {
        // Starting mid-file
        currClusterID = getNextClusterID(pFAT, pNodesQueue->Tail);

        // Block Read Failure
        if (currClusterID == 0)
        {
            return FLOAD_FAIL;
        }
    }

    // Fill the NodesQueue up with the next free ClusterIDs
    if (mode == FILE_MODE_WRITE || mode == FILE_MODE_WRITE_NEW)
    {
        return loadFreeClusterIDs(pFAT, pNodesQueue, currClusterID);
    }
    else
    {
        // Loop until ether the queue is full or EOF is found and Report the status
        return traverseTable(pFAT, pNodesQueue, currClusterID, FAT_TRAVERSE_LOAD_QUEUE);
    }
}

/****************************************************************************************
 *	@fn 			     - FAT_fclose
 *
 * 	@brief			     - Empty the nodes queue
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 * 	@return			     - Search status
 *
 */
void FAT_fclose(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->NodesQueue;
    uint32_t tempNode;

    // Confirm the current file is being closed
    if (pFAT->CurrFile != file)
    {
        return;
    }

    // Close out the file state
    file->state = FILE_STATE_CLOSE;

    if (file->mode == FILE_MODE_WRITE || file->mode == FILE_MODE_WRITE_NEW)
    {
        // Update directory entry
        updateDirEntry(pFAT, file);
    }
    else
    {
        // Initialized the tail to Zero which is a non-acceptable clusterID
        pNodesQueue->Tail = NODES_QUEUE_TAIL_INIT;

        // Empty out the NodesQueue
        while (!isQueueEmpty(&pNodesQueue->Info))
        {
            // queue the directory at the head of the queue
            dequeue(&pNodesQueue->Info, &tempNode);
        }
    }
}

/****************************************************************************************
 *	@fn 			     - FAT_fread
 *
 * 	@brief			     - Read block(s) of data from a file
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *  @param[data]         - Output: Pointer set to SD buffer containing read data
 *  @param[size]         - Output: Number of valid bytes in buffer (full buffer or final block remainder)
 *
 * 	@return			     - FREAD_DONE, FREAD_EOF_FOUND, FREAD_FAIL, or FREAD_NOP
 *
 * 	@note			     - Buffer is automatically toggled. For DMA transfers, caller must 
 *                           wait for completion before next read. Size accounts for final 
 *                           block when EOF is reached.
 */
fat_fread_t FAT_fread(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t** data, uint32_t* size)
{
    NodesQueue* pNodesQueue = &file->NodesQueue;

    static uint32_t sectorsRead = 0;
    static uint32_t currSectorNum = 0;
    static uint32_t currBaseAddr = 0;

    // Exit if the NodesQueue is empty or the file argument is not the current file
    if (isQueueEmpty(&pNodesQueue->Info) || (pFAT->CurrFile != file) 
       || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS) 
       || (file->mode != FILE_MODE_READ))
    {
        return FREAD_NOP;
    }

    // Reset the Static variables on the first write
    if (file->state == FILE_STATE_IDLE)
    {
        // Update the file state
        file->state = FILE_STATE_READ;

        currSectorNum = 0;
        currBaseAddr = 0;
        sectorsRead = 0;
    }

    System_info_t* systemInfo = &pFAT->SystemInfo;
    uint32_t sectorsPerBuffer = SD_GetBuffSize(pFAT->pSDHandle) / systemInfo->BytesPerSector;

    // Only read up to a cluster at a time, even if the buffer can hold more. 
    uint32_t sectorsToRead = sectorsPerBuffer > systemInfo->SectorsPerCluster ? systemInfo->SectorsPerCluster : sectorsPerBuffer;

    // Calculate the number of sectors remaining in the file and adjust to not read past the end of the file
    uint32_t totalFileSectors = (file->FileSize + (systemInfo->BytesPerSector - 1)) / systemInfo->BytesPerSector;
    uint32_t sectorsRemaining = totalFileSectors - sectorsRead;
    sectorsToRead = (sectorsRemaining < sectorsToRead) ? sectorsRemaining : sectorsToRead;

    uint32_t tempNode;
    
    if (currSectorNum == 0)
    {
        // Peak the next node from the Queue, it'd be dequeued when its finished.
        peekQueue(&pNodesQueue->Info, &tempNode);
        
        // Update the current base address
        currBaseAddr = FAT_GetClusterAddr(pFAT, tempNode);
    }
    
    // Read the current data block
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsToRead);

    // Read Error Handling
    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        // Report Issue
        return FREAD_FAIL;
    }

    sectorsRead += sectorsToRead;

    // Found the End of File
    if ((sectorsRead * systemInfo->BytesPerSector) >= file->FileSize)
    {
        // Done processing the current cluster
        dequeue(&pNodesQueue->Info, &tempNode);

        // Reset the static variables
        currSectorNum = 0;
        sectorsRead = 0;
        currBaseAddr = 0;

        // Set output parameters
        uint32_t bufSize = SD_GetBuffSize(pFAT->pSDHandle);
        *data = SD_GetBuffAddr(pFAT->pSDHandle);
        *size = file->FileSize % bufSize;
        SD_ToggleCurrBuff(pFAT->pSDHandle);

        return FREAD_EOF_FOUND;
    }

    // Determine how the current sector needs to be incremented
    if (currSectorNum < systemInfo->SectorsPerCluster - sectorsToRead)
    {
        currSectorNum += sectorsToRead;
    }
    else
    {
        // Done processing the current cluster
        dequeue(&pNodesQueue->Info, &tempNode);

        // If the queue is empty but the EOF is not found, we need to reload the queue
        if (isQueueEmpty(&pNodesQueue->Info) && !isEndofFatEntry(pFAT, pNodesQueue->Tail))
        {
            fat_fload_t loadStatus = loadFileNodesQueue(pFAT, file, FILE_MODE_READ);

            // fload writes over the block buffer, so we need to re-read the current block.
            cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsPerBuffer);

            // Read Error Handling
            if (cmdStatus == SD_READ_WRITE_FAIL || loadStatus == FLOAD_FAIL)
            {
                // Failure Could have while loading the queue.
                while (!isQueueEmpty(&pNodesQueue->Info))
                {
                    dequeue(&pNodesQueue->Info, &tempNode);
                }

                // Report Issue
                return FREAD_FAIL;
            }
        }

        // Reset the current Sector number
        currSectorNum = 0;
    }

    // Set output parameters
    *data = SD_GetBuffAddr(pFAT->pSDHandle);
    *size = sectorsToRead * systemInfo->BytesPerSector;
    SD_ToggleCurrBuff(pFAT->pSDHandle);

    return FREAD_DONE;
}

/****************************************************************************************
 *	@fn 			     - FAT_fwrite
 *
 * 	@brief			     - Write Block(s) of data from a file
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 * 	@return			     - Search status
 *
 */
fat_fwrite_t FAT_fwrite(FAT_Handle_t* pFAT, file_entry_t* file)
{
    NodesQueue* pNodesQueue = &file->NodesQueue;

    static uint32_t currSectorNum = 0;
    static uint32_t currBaseAddr = 0;

    // Exit if the NodesQueue is empty or the file argument is not the current file
    if (isQueueEmpty(&pNodesQueue->Info) || (pFAT->CurrFile != file) || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS) || (file->mode != FILE_MODE_WRITE && file->mode != FILE_MODE_WRITE_NEW))
    {
        return FWRITE_NOP;
    }

    uint32_t tempNode;
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    
    System_info_t* systemInfo = &pFAT->SystemInfo;
    uint32_t sectorsPerBuffer = SD_GetBuffSize(pFAT->pSDHandle) / systemInfo->BytesPerSector;

    // Only write up to a cluster at a time, even if the buffer can hold more. 
    uint32_t sectorsToWrite = sectorsPerBuffer > systemInfo->SectorsPerCluster ? systemInfo->SectorsPerCluster : sectorsPerBuffer;

    // Reset the Static variables on the first write
    if (file->state == FILE_STATE_IDLE)
    {
        // Update the file state
        file->state = FILE_STATE_WRITE;

        /*** Determine where to start writing ***/
        
        // Calculate how many bytes are written to the final cluster
        uint32_t bytesPerFinalCluster = file->FileSize % (systemInfo->BytesPerSector * systemInfo->SectorsPerCluster);

        // Peak the next node from the Queue, it'd be dequeued when its finished.
        peekQueue(&pNodesQueue->Info, &tempNode);

        // Calculate current sector number and base address
        currSectorNum = bytesPerFinalCluster / systemInfo->BytesPerSector;
        currBaseAddr = FAT_GetClusterAddr(pFAT, tempNode) + (currSectorNum * addrUnit);
    }
    else if (file->state == FILE_STATE_WRITE)
    {
        if (currSectorNum == 0)
        {
            // Peak the next node from the Queue, it'd be dequeued when its finished.
            peekQueue(&pNodesQueue->Info, &tempNode);

            // Update the current base address
            currBaseAddr = FAT_GetClusterAddr(pFAT, tempNode);
        }
    }
    else
    {
        return FWRITE_NOP;
    }

    // Read the current data block
    if (SD_WriteBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsToWrite) == SD_READ_WRITE_FAIL)
    {
        // Report Issue
        return FWRITE_FAIL;
    }

    // Determine how File size needs to be updated
    if (file->EndingCluster == tempNode)
    {
        uint32_t finalBlockOffset = file->FileSize % SD_GetBuffSize(pFAT->pSDHandle);

        file->FileSize += SD_GetBuffSize(pFAT->pSDHandle) - finalBlockOffset;
    }
    else
    {
        file->FileSize += SD_GetBuffSize(pFAT->pSDHandle);
    }

    // Determine how the current sector needs to be incremented
    if (currSectorNum < pFAT->SystemInfo.SectorsPerCluster - sectorsToWrite)
    {
        currSectorNum += sectorsToWrite;
    }
    else
    {
        // Done processing the current cluster
        dequeue(&pNodesQueue->Info, &tempNode);

        // Update FAT Table Entry
        if (file->EndingCluster != tempNode)
        {
            if (updateClusterID(pFAT, file->EndingCluster, tempNode) == FWRITE_FAIL)
            {
                return FWRITE_FAIL;
            }
        }

        // Update NodesQueue Tail
        file->EndingCluster = tempNode;

        // Reset the current Sector number
        currSectorNum = 0;
    }

    return FWRITE_DONE;
}

/****************************************************************************************
 *	@fn 			     - FAT_feof
 *
 * 	@brief			     - Has the end of file been reached
 *
 * 	@param[file]		 - Memory location the file details
 *
 * 	@return			     - whether the NodesQueue is empty
 *
 */
bool FAT_feof(file_entry_t* file)
{
    NodesQueue* pNodesQueue = &file->NodesQueue;

    return isQueueEmpty(&pNodesQueue->Info) || (file->state == FILE_STATE_FAIL);
}

/****************************************************************************************
 *  @fn                - FAT_readHeaderBlock
 *
 *  @brief             - Read header block from a file
 *
 *  @param[pFAT]       - Handler structure for FAT
 *  @param[file]       - Memory location the file details
 *
 *  @return            -  
 *
 *  @note              - 
 */
fat_fread_t FAT_readHeaderBlock(FAT_Handle_t* pFAT, file_entry_t* file)
{
    sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, FAT_GetClusterAddr(pFAT, file->StartingCluster), 1);

    // Return read status
    return (cmdStatus == SD_READ_WRITE_FAIL) ? FREAD_FAIL : FREAD_DONE;
}

/****************************************************************************************
 *	@fn 			     - FAT_GetClusterAddress
 *
 * 	@brief			     - Return the starting address of a given cluster
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[ClusterID]	 - Cluster of interest
 *
 * 	@return			     - Return base address for a clusterID
 *
 * 	@note   - There is cluster 0 or 1. Cluster’s IDs begin from 2
 */
uint32_t FAT_GetClusterAddr(FAT_Handle_t* pFAT, uint32_t ClusterID)
{
    uint32_t addrUnit = getFatAddrUnit(pFAT);

    if (ClusterID < 2)
    {
        return pFAT->SystemInfo.RootDirAddress;
    }
    else
    {
        return (pFAT->SystemInfo.DataStartAddress + (ClusterID - 2) * (pFAT->SystemInfo.SectorsPerCluster * addrUnit));
    }
}

/****************************************************************************************
 *	@fn 			     - hasFileFlag
 *
 * 	@brief			     - Get status flags from the file attribute byte
 *
 * 	@param[file]	     - Memory location of file structure
 * 	@param[flag]	     - Flag to check for
 *
 * 	@return			     - True or False
 *
 */
static bool hasFileFlag(file_entry_t *file, FAT_file_flags_t flag)
{
    return file->FileAttribute & flag;
}

/****************************************************************************************
 *	@fn 			     - isHiddenFile
 *
 * 	@brief			     - Determine whether file is valid
 *
 * 	@param[file]	     - Memory location of file structure
 *
 * 	@return			     - True or False
 *
 */
bool FAT_IsHiddenFile(file_entry_t* file)
{
    return hasFileFlag(file, FILE_FLAG_VOLUME | FILE_FLAG_SYSTEM | FILE_FLAG_HIDDEN);
}

/****************************************************************************************
 *	@fn 			     - FAT_IsDirectory
 *
 * 	@brief			     - Determine whether file is a directory
 *
 * 	@param[file]	     - Memory location of file structure
 *
 * 	@return			     - True or False
 *
 */
bool FAT_IsDirectory(file_entry_t *file)
{
    return hasFileFlag(file, FILE_FLAG_DIRECTORY);
}

/****************************************************************************************
 *  @fn                  - isEndOfDir
 *
 *  @brief               - Determine whether file is valid
 *
 *  @param[file]         - Memory location of file structure
 *
 *  @return              - True or False
 *    
 */
static bool isEndOfDir(file_entry_t* file)
{
    return file->Filename[0] == END_OF_DIRECTORY;
}

/****************************************************************************************
 *	@fn 			     - isLongFile
 *
 * 	@brief			     - Determine whether file is valid
 *
 * 	@param[file]	     - Memory location of file structure
 *
 * 	@return			     - True or False
 *
 */
static bool isLongFile(file_entry_t* file)
{
    return file->FileAttribute == FILE_FLAG_LFN;
}

/****************************************************************************************
 *	@fn 			     - FAT_SetWorkingAddr
 *
 * 	@brief			     - Set the FAT working address
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[WorkingAddr]	 - Handler structure for FAT
 * 	@param[offset]	 	 - Handler structure for FAT
 *
 * 	@return			     - none
 *
 * 	@note
 */
void FAT_SetWorkingAddr(FAT_Handle_t* pFAT, uint32_t WorkingAddr, uint32_t offset)
{
    pFAT->WorkingAddr.baseAddr = WorkingAddr;
    pFAT->WorkingAddr.offset = offset;
}

/****************************************************************************************
 *	@fn 			     - FAT_GetWorkingAddress
 *
 * 	@brief			     - Get the FAT working address
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 *
 * 	@return			     - Current working address structure
 *
 * 	@note
 */
static WorkingAddr_t getWorkingAddr(FAT_Handle_t* pFAT)
{
    return pFAT->WorkingAddr;
}

/****************************************************************************************
 *	@fn 			     - FAT_SetWorkingDir
 *
 * 	@brief			     - Set the FAT working directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[WorkingAddr]	 - Handler structure for FAT
 *
 * 	@return			     - none
 *
 * 	@note
 */
void FAT_SetWorkingDir(FAT_Handle_t* pFAT, uint32_t WorkingDir)
{
    pFAT->WorkingAddr.workingDir = WorkingDir;
}

/****************************************************************************************
 *	@fn 			     - FAT_GetWorkingDir
 *
 * 	@brief			     - Get the FAT working directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 *
 * 	@return			     - Top of the directory stack
 *
 * 	@note
 */
uint32_t FAT_GetWorkingDir(FAT_Handle_t* pFAT)
{
    return pFAT->WorkingAddr.workingDir;
}

/****************************************************************************************
 *	@fn 			     - FAT_GoToRootDir
 *
 * 	@brief			     - Set the FAT working directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 *
 * 	@return			     - none
 *
 * 	@note
 */
void FAT_GoToRootDir(FAT_Handle_t* pFAT)
{
    // Set the Working Directory
    FAT_SetWorkingDir(pFAT, pFAT->SystemInfo.RootDirAddress);
    // Set the Working Directory to the root dir
    FAT_SetWorkingAddr(pFAT, pFAT->SystemInfo.RootDirAddress, 0);
}

/*************************************************************************************************
 *                                      Static Functions                                        *
 *************************************************************************************************/

/****************************************************************************************
 *	@fn 			     - setCurFile
 *
 * 	@brief			     - Set the pointer to the current file. 
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[file]	     - Memory location of file structure
 *
 * 	@return			     - None
 *
 */
static void setCurFile(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // The field is used to keep track of which file the application is currently accessing
    pFAT->CurrFile = file;
}

/****************************************************************************************
 *	@fn 			     - removeSpacePadding
 *
 * 	@brief			     - Remove padding at the end of Volume name, filename or file extension
 *
 * 	@param[text]	     - Pointer of text to process
 * 	@param[fieldSize]	 - Size of text to process
 *
 * 	@return			     - None
 *
 *
 */
static void removeSpacePadding(uint8_t* text, uint8_t fieldSize)
{
    uint8_t* endPtr = (text + fieldSize - 1);

    /* Short Volume name, filenames and file extensions are padded with space bytes (ASCII: 0x20).
	 * Loop from the end until we find an non space. */
    while (fieldSize > 0 && (*endPtr == ' '))
    {
        // Update the pad to a string terminator
        if (*endPtr == ' ')
        {
            *endPtr = '\0';
        }

        // Decrement the pointer and field size
        endPtr--;
        fieldSize--;
    }
}

/****************************************************************************************
 *	@fn 			     - parseFilename
 *
 * 	@brief			     - Function to parse filename & extension from full name
 *
 * 	@param[FullName]	 - full file name with extension
 * 	@param[Filename]	 - Location to store the file name
 * 	@param[FileExt]	 	 - Location to store the file extension
 *
 * 	@return			     - Offset into the file where the name begins (after the path)
 *
 * 	@note
 */
static uint8_t parseFilename(char* FullName, uint8_t* Filename, uint8_t* FileExt)
{
    uint8_t startIndex = 0;

    // Terminate file extension this is for a directory search
    *FileExt = '\0';

    if (strcmp(FullName, ".") == 0 || strcmp(FullName, "..") == 0)
    {
        strncpy((char*)Filename, FullName, FILENAME_MAX_SIZE + 1);
    }
    else
    {
        uint8_t n;
        uint8_t len = strlen(FullName);
        uint8_t dotIndex = FILENAME_MAX_SIZE + 1;

        // Loop to find the file extension
        for (n = 0, startIndex = 0; n < len; n++)
        {
            if (FullName[n] == '.')
            {
                // Copy in the extension
                strncpylower(FileExt, (uint8_t*)&FullName[n + 1], FILE_EXT_LONG_SIZE);
                dotIndex = n;
            }
            // Adjust the starting index incase the name is a path
            // i.e. ~/Recipes/MyRecipe.txt
            else if (FullName[n] == '/')
            {
                startIndex = n + 1;
            }
        }

        // Copy over the filename
        strncpy((char*)Filename, FullName + startIndex, FILENAME_MAX_SIZE + 1);
        Filename[dotIndex] = '\0';
    }

    return startIndex;
}

/****************************************************************************************
 *	@fn 			      - isEndofFatEntry
 *
 * 	@brief			      - Remove padding at the end of Volume name, filename or file extension
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[nextCluster]	  - Next ClusterID
 *
 * 	@return			      - true or false
 *
 */
static bool isEndofFatEntry(FAT_Handle_t* pFAT, uint32_t nextCluster)
{
    uint32_t EOF_Marker = (getFatType(pFAT) == FAT_TYPE_FAT16) ? FAT_EOF_MARKER_FAT16 : FAT_EOF_MARKER_FAT32;

    /* 
     * EOF ClusterIDs
     * FAT16:     0xFFF8 to 0xFFFF
     * FAT32: 0xFFFFFFF8 to 0xFFFFFFFFF
     */
    if (nextCluster >= EOF_Marker)
    {
        return true;
    }
    return false;
}

/****************************************************************************************
 *	@fn 			     - getTableBlockAddr
 *
 * 	@brief			     - Return the starting address of a given cluster
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[ClusterID]	 - Cluster of interest
 *
 * 	@return			     - Base Address and offset of block where FAT entry lives
 *
 * 	@note  - // TODO: Test filling up FAT table to stress this
 */
static WorkingAddr_t getTableBlockAddr(FAT_Handle_t* pFAT, uint32_t clusterID)
{
    uint32_t clusterIdSize, clusterIdBlock;
    WorkingAddr_t blockAddr = {0};

    // Determine the size of the ClusterIDs
    clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Determine which block the entry resides in
    clusterIdBlock = (clusterID * clusterIdSize) / pFAT->SystemInfo.BytesPerSector;

    // Calculate the base address of block that holds the cluster
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    blockAddr.baseAddr = pFAT->SystemInfo.FAT1_Address + clusterIdBlock * addrUnit;

    // Calculate the offset into that block
    blockAddr.offset = (clusterID * clusterIdSize) % pFAT->SystemInfo.BytesPerSector;

    return blockAddr;
}

/****************************************************************************************
 *	@fn 			     - getNextClusterID
 *
 * 	@brief			     - Get the next cluster ID for a file
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[clusterID]	 - Cluster of interest
 *
 * 	@return			     - Next Cluster ID
 *
 * 	@note
 */
static uint32_t getNextClusterID(FAT_Handle_t* pFAT, uint32_t clusterID)
{
    WorkingAddr_t clusterLoc = getTableBlockAddr(pFAT, clusterID);

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Check if we need to update our working address
    if (getWorkingAddr(pFAT).baseAddr != clusterLoc.baseAddr)
    {
        // Update current working address
        FAT_SetWorkingAddr(pFAT, clusterLoc.baseAddr, 0);

        // Read new block
        // TODO: Should this just read single block for simplicity??
        sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, clusterLoc.baseAddr, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo.BytesPerSector);

        // If the command fails, send invalid clusterID
        return (cmdStatus == SD_READ_WRITE_FAIL) ? 0 : ToLittleEndian(SD_GetBuffAddr(pFAT->pSDHandle) + clusterLoc.offset, clusterIdSize);
    }

    return ToLittleEndian(SD_GetBuffAddr(pFAT->pSDHandle) + clusterLoc.offset, clusterIdSize);
}

/****************************************************************************************
 *	@fn                  - findNextFreeClusterID
 *
 * 	@brief               - Search for the next free clusterID
 *
 * 	@param[pFAT]         - Handler structure for FAT
 * 	@param[clusterID]    - Cluster to start search from (Last Cluster In File)
 *
 * 	@return              - Next Cluster ID
 *
 * 	@note   -
 */
static uint32_t findNextFreeClusterID(FAT_Handle_t* pFAT, uint32_t clusterID)
{
    WorkingAddr_t clusterLoc;

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    uint32_t ClusterIdValue = 0xFFFF;

    while (ClusterIdValue != FAT_FREE_ID_MARKER)
    {
        // Increment to the next clusterID
        clusterID++;

        // Look to the next ClusterID
        clusterLoc = getTableBlockAddr(pFAT, clusterID);

        // Check if we need to update our working address
        if (getWorkingAddr(pFAT).baseAddr != clusterLoc.baseAddr)
        {
            // Update current working address
            FAT_SetWorkingAddr(pFAT, clusterLoc.baseAddr, 0);

            // Read new block
            sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, clusterLoc.baseAddr, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo.BytesPerSector);

            // If the command fails, send invalid clusterID
            if (cmdStatus == SD_READ_WRITE_FAIL)
            {
                return 0;
            }
        }

        ClusterIdValue = ToLittleEndian(SD_GetBuffAddr(pFAT->pSDHandle) + clusterLoc.offset, clusterIdSize);
    }

    return clusterID;
}

/****************************************************************************************
 *  @fn                  - updateClusterID
 *
 *  @brief               - Updated the ClusterID linked list
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[clusterID]    - Cluster of interest
 *  @param[nextID]       - Next Cluster
 *
 *  @return              - Next Cluster ID
 *                       - // TODO: parameter to select which FAT copy
 *
 *  @note  - Inputs clusterID = nextID is used to create new table entry
 */
static fat_fwrite_t updateClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t nextID)
{

    WorkingAddr_t clusterLoc = getTableBlockAddr(pFAT, clusterID);
    WorkingAddr_t nextLoc = getTableBlockAddr(pFAT, nextID);
    uint8_t* fatBlock = SD_GetBuffAddr(pFAT->pSDHandle);

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Read the FAT block
    sd_read_write_t cmdStatus = SD_ReadBlock(pFAT->pSDHandle, clusterLoc.baseAddr, 1);

#if defined(FAT_DEBUG_TABLE)
    HexdumpBuffer(fatBlock, pFAT->SystemInfo.BytesPerSector);
#endif

    // Error Handling
    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        return FWRITE_FAIL;
    }

    // If both ClusterID are in the same block it can be updated in one write
    if (clusterLoc.baseAddr == nextLoc.baseAddr)
    {
        // Update the ClusterIDs
        ToEndianBuf(fatBlock + clusterLoc.offset, nextID, clusterIdSize);
        // NextID Terminated with EOF
        ToEndianBuf(fatBlock + nextLoc.offset, FAT_EOF_MARKER_GENERIC, clusterIdSize);

#ifndef FAT_DEBUG_TABLE
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, clusterLoc.baseAddr, 1);
#endif
    }
    // Otherwise it will be updated in two read/writes
    else
    {
#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo.BytesPerSector / 4);
#endif

        // Update first ClusterID
        ToEndianBuf(fatBlock + clusterLoc.offset, nextID, clusterIdSize);
#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo.BytesPerSector / 4);
#else
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, clusterLoc.baseAddr, 1);
#endif

        // Read Block and Update nextID
        cmdStatus = SD_ReadBlock(pFAT->pSDHandle, nextLoc.baseAddr, 1);
        // NextID Terminated with EOF
        ToEndianBuf(fatBlock + nextLoc.offset, FAT_EOF_MARKER_GENERIC, clusterIdSize);

#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo.BytesPerSector / 4);
#else
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, nextLoc.baseAddr, 1);
#endif
    }

#if defined(FAT_DEBUG_TABLE)
    HexdumpBuffer(fatBlock, pFAT->SystemInfo.BytesPerSector);
#endif

    // Error Handling
    if (cmdStatus == SD_READ_WRITE_FAIL)
    {
        return FWRITE_FAIL;
    }
    else
    {
        return FWRITE_SUCCESS;
    }
}

/****************************************************************************************
 *  @fn                   - loadFreeClusterIDs
 *
 *  @brief                - Function to fill NodesQueue with free clusterIDs
 * 
 * 	@param[pFAT]          - Handler structure for FAT
 *  @param[pNodesQueue]   - Queue to hold file nodes
 *  @param[startCluster]  - Starting ClusterID
 *
 *  @return               - None  
 *
 *  @note                 - 
 */
static fat_fload_t loadFreeClusterIDs(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster)
{
    uint32_t currClusterID = startCluster;

    // Loop until the NodesQueue is full
    while (!isQueueFull(&pNodesQueue->Info))
    {
        currClusterID = findNextFreeClusterID(pFAT, currClusterID);

        // Block Read Failure
        if (currClusterID == 0)
        {
            return FLOAD_FAIL;
        }
        else
        {
            enqueue(&pNodesQueue->Info, &currClusterID);
        }
    }

    return FLOAD_EOF_FOUND;
}

/****************************************************************************************
 *  @fn                - getFatType
 *
 *  @brief             - get File System Type 
 *
 * 	@param[pFAT]	   - Handler structure for FAT
 *
 *  @return            -  FAT_TYPE_FAT16 or FAT_TYPE_FAT32
 *
 *  @note              - 
 */
fat_types_t getFatType(FAT_Handle_t* pFAT)
{
    return pFAT->SystemInfo.FAT_Type;
}

/****************************************************************************************
 *  @fn                - getFatAddrUnit
 *
 *  @brief             - get Address Unit type based on FAT system
 *
 * 	@param[pFAT]	   - Handler structure for FAT
 *
 *  @return            -  Address unit
 *
 *  @note              - 
 */
uint32_t getFatAddrUnit(FAT_Handle_t* pFAT)
{
    return (getFatType(pFAT) == FAT_TYPE_FAT16) ? pFAT->SystemInfo.BytesPerSector : 1;
}

/****************************************************************************************
 *  @fn                - FAT_getStat
 *
 *  @brief             - get File System Type 
 *
 * 	@param[pFAT]       - Handler structure for FAT
 *
 *  @return            -  FAT_Stat
 *
 *  @note              - 
 */
fat_status_t FAT_getStat(FAT_Handle_t* pFAT)
{
    return pFAT->FAT_Stat;
}

/****************************************************************************************
 *  @fn                   - traverseTable
 *
 *  @brief                - Function to traverse through FAT Table
 * 
 * 	@param[pFAT]          - Handler structure for FAT
 *  @param[pNodesQueue]   - Queue to hold file nodes
 *  @param[startCluster]  - Starting ClusterID
 *  @param[mode]          - Mode
 *
 *  @return               - None  
 *
 *  @note                 - 
 */
static fat_fload_t traverseTable(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster, fat_traverse_mode_t mode)
{
    uint32_t currClusterID = startCluster;

    // Loop until ether the queue is full or EOF is found.
    do
    {
        if (mode == FAT_TRAVERSE_LOAD_QUEUE)
        {
            // Enqueue the clusterID
            enqueue(&pNodesQueue->Info, &currClusterID);
        }

        // Large files may need to be read in multiple passes
        // So we keep track of the queue tail
        pNodesQueue->Tail = currClusterID;

        // Get the next clusterID
        currClusterID = getNextClusterID(pFAT, currClusterID);

        // Block Read Failure
        if (currClusterID == 0)
        {
            // TODO: Empty Queue?
            return FLOAD_FAIL;
        }

    } while (!isQueueFull(&pNodesQueue->Info) && !isEndofFatEntry(pFAT, currClusterID));

    if (isEndofFatEntry(pFAT, currClusterID))
    {
        // Sent Ending Cluster
        pFAT->CurrFile->EndingCluster = pNodesQueue->Tail;

        // Update the Queue Tail to EOF
        pNodesQueue->Tail = currClusterID;

        return FLOAD_EOF_FOUND;
    }
    else
    {
        // Use Non-Valid value
        pFAT->CurrFile->EndingCluster = 0;

        return FLOAD_EOF_NOT_FOUND;
    }
}

/*******************       IRQ Handling and callback       *******************/

/****************************************************************************************
 *  @fn            - FAT_IRQHandling
 *
 *  @brief         - Handle FAT IRQ Events
 *
 *  @param[pFAT]   - Handler structure for FAT
 *
 *  @return        - None
 *
 *  @note          - 
 */
void FAT_IRQHandling(FAT_Handle_t* pFAT)
{
    if (SD_GetState(pFAT->pSDHandle) == SD_STATE_NO_CARD)
    {
        pFAT->FAT_Stat = FAT_UNINITIALIZED;
    }
}
