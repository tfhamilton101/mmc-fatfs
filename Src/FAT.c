/****************************************
 * Fat.c
 *
 *  Created on: Nov 22th, 2020
 *      Author: thomashamilton
 ****************************************/

#include "FAT.h"
#include "sd.h"
#include "sd_spec.h"
#include "Queue.h"
#include "Stack.h"
#include "stm32f4xx_dma_driver.h"
#include "Utils.h"
#include <errno.h>
#include <string.h>

/******  FAT DEBUG CONFIGS  ******/

// #define FAT_DEBUG_GENERIC
// #define FAT_DEBUG_TABLE

#if defined(FAT_DEBUG_GENERIC)
#include "Hexdump.h"
#endif

/************************************************************************************
 *							 FAT Macros										        *
 ************************************************************************************/

/*
 *  Structure for System information
 */
#define VOLUME_LABEL_SIZE 11

/**** Macros for Master Boot Record ****/
typedef enum
{
    MBR_TYPE_CODE = 0x01C2,
    MBR_LBA = 0x01C6,
    MBR_END_MARKER = 0x01FE,
} mbr_offsets_t;

/*
 *  @FAT_type
 *  Possible Modes for SD Communication
 */
typedef enum
{
    FAT_TYPE_FAT16 = 0,
    FAT_TYPE_FAT32,
} fat_types_t;

/*
 *  System Info Structure
 */
struct System_info_t
{
    fat_types_t FAT_Type;
    uint32_t FAT_Copies;
    uint32_t SectorsPerCluster;
    uint32_t BytesPerSector;
    uint32_t SectorsPerFAT;
    uint32_t RootDirEntries;
    uint32_t RootDirAddress;
    uint32_t FAT1_Address;
    uint32_t FAT2_Address;
    uint32_t DataStartAddress;
    uint32_t SectorsPerVolume;
    uint8_t VolumeLabel[VOLUME_LABEL_SIZE];
};

/************************************************************************************
 *							 File Specific Macros    								*
 ************************************************************************************/

#define DIR_BYTES_PER_ENTRY 32
#define MAX_DIRECTORIES 32
#define MAX_ENTIRES_PER_DIRECTORY 256

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


/**********************************
 *      File search enums		  *
 **********************************/

// Search Mode
typedef enum
{
    SEARCH_FILE_LOCAL = 0,
    SEARCH_FILE_RECURSIVE,
    SEARCH_DIR_LOCAL,
    SEARCH_DIR_RECURSIVE,
} Search_Mode_t;

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

typedef enum
{
    FILE_ATTRIBUTE_SIZE = 1,
    FILE_TIME_MODIFIED_SIZE = 2,
    FILE_DATE_MODIFIED_SIZE = 2,
    FILE_START_CLUSTER_SIZE = 2,
    FILE_SIZE_SIZE = 4
} FAT_file_entry_size_t;

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

typedef struct
{
    QueueInfo Info;
    uint32_t Tail;
} NodesQueue;

struct file_context_t
{
    uint32_t StartingCluster;
    uint32_t EndingCluster;
    uint32_t DirEntryBaseAddr;
    uint32_t DirEntryOffset;
    NodesQueue NodesQueue;
    uint32_t iterBaseAddr;
    uint32_t iterOffset;
};

/***************************************************
 * 				Global Variables    			   *
 ***************************************************/
FAT_Handle_t FAT = {0};

// Buffer to keep File Construction Stack
static uint32_t FileNodesBuf[FAT_QUEUE_MAX_CLUSTERS];
// Currently there is only one file context, so we can only process one file at a time.
static struct file_context_t fileContext;

/***************************************************
 * 				Static Functions    			   *
 ***************************************************/
static int getSystemInfo(FAT_Handle_t* pFAT);
static fat_types_t getFatType(FAT_Handle_t* pFAT);
static uint32_t getFatAddrUnit(FAT_Handle_t* pFAT);

// Remove the padded spaces from a file entry name & entension
static void removeSpacePadding(uint8_t* text, uint8_t fieldSize);
// Parse File name and extension
static uint8_t parseFilename(char* FullName, uint8_t* Filename, uint8_t* FileExt);
static uint8_t getShortFilename(file_entry_t* file, uint8_t* filename);

static int loadFileNodesQueue(FAT_Handle_t* pFAT, file_entry_t* file, file_mode_t mode);
static int loadFreeClusterIDs(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster);
static bool isEndofFatEntry(FAT_Handle_t* pFAT, uint32_t nextCluster);
static void getFatEntryAddr(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t* pBaseAddr, uint32_t* pOffset);
static uint32_t getClusterAddr(FAT_Handle_t* pFAT, uint32_t ClusterID);
static uint32_t getNextClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t* pLoadedBaseAddr);
static int updateClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t nextID);
static uint32_t findNextFreeClusterID(FAT_Handle_t* pFAT, uint32_t clusterID);
static uint32_t traverseTable(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster, fat_traverse_mode_t mode);
/* FAT Directory Functions */

static bool isEndOfDir(file_entry_t* file);

// Create new file
static int createFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file);
static int findFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode, uint32_t startAddr);
static int findDirectory(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode, uint32_t startAddr);
static int searchPath(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file);
static int updateDirEntry(FAT_Handle_t* pFAT, file_entry_t* file);


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
int InitFAT(FAT_Handle_t* pFAT, struct SD_Handle_t* pSDHandle)
{
    // Link SD handler to FAT handler
    pFAT->pSDHandle = pSDHandle;

    // Init SD Card
    int status = SD_Init(pSDHandle);

    if (status != 0)
    {
        // SD_Init already returned negative errno
        pFAT->FAT_Stat = INIT_FAT_FAIL;
        return status;
    }

    // Read FAT system parameters from Boot sector
    return getSystemInfo(pFAT);
}

/****************************************************************************************
 *	@fn 			     - getSystemInfo
 *
 * 	@brief			     - Get information for FAT
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 *
 * 	@return			     - 0 on success, negative errno on failure
 *
 * 	@note
 */
static int getSystemInfo(FAT_Handle_t* pFAT)
{
    // Allocate and link the system info structure
    static struct System_info_t SystemInfoData = {0};
    pFAT->SystemInfo = &SystemInfoData;
    
    System_info_t* SystemInfo = pFAT->SystemInfo;
    uint8_t* buff = SD_GetBuffAddr(pFAT->pSDHandle);

    /******************		  Read Master Boot Record (MBR) 	   **************/
    /* The MBR is the first sector of the drive. This sector contains boot code *
        * and a partition table. The contents of a FAT file system are located in  *
        * the first partition. In the partition description, We only care about    *
        * the logical block address (LBA) and the	partition type 					*/

    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, 0, 1);

    if (cmdStatus < 0)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return -EIO;
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
        return -EINVAL;
    }

    if (ToLittleEndian(&buff[MBR_END_MARKER], DATA_SIZE_HALF_WORD) != BOOT_SIGNATURE)
    {
        pFAT->FAT_Stat = INIT_BAD_END_MARKER;
        return -EINVAL;
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

    if (cmdStatus < 0)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return -EIO;
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

    if (cmdStatus < 0)
    {
        pFAT->FAT_Stat = INIT_CMD_FAIL;
        return -EIO;
    }

    /* Instead we will get the volume label from the first in the root directory. This is a special item just for this. */
    strncpy((char*)SystemInfo->VolumeLabel, (char*)(buff + FILENAME), VOLUME_LABEL_SIZE);
    removeSpacePadding(SystemInfo->VolumeLabel, VOLUME_LABEL_SIZE);

    /* Update the FAT Handler Status */
    pFAT->FAT_Stat = INIT_FAT_SUCCESS;
    return 0;
}

/****************************************************************************************
 *	@fn 			     - FAT_ReadDir
 *
 * 	@brief			     - Parse a file information from a directory
 *
 * 	@param[pFAT]	 	 - Handler structure for FAT
 * 	@param[file]		 - Memory location to store file details
 *
 * 	@return			     - 1 if entry found, 0 if end-of-directory, negative errno on error
 *
 */
int FAT_ReadDir(FAT_Handle_t* pFAT, file_entry_t* dir, file_entry_t* entry)
{
    // These local variables are used to make function statements shorter
    uint32_t rxBufferSize = SD_GetBuffSize(pFAT->pSDHandle);
    uint8_t* entryAddr = SD_GetBuffAddr(pFAT->pSDHandle);

    // Initialize scan position from directory state
    uint32_t currBaseAddr = dir->context->iterBaseAddr;
    uint32_t currOffset = dir->context->iterOffset;
    
    // Read new block if necessary
    if (currOffset >= rxBufferSize)
    {
        // Increment the base address to the next sector
        if (getFatType(pFAT) == FAT_TYPE_FAT16)
        {
            currBaseAddr += rxBufferSize;
        }
        else
        {
            currBaseAddr++;
        }

        currOffset = 0;

        int status = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr, rxBufferSize / pFAT->SystemInfo->BytesPerSector);

        // Read Error Handling
        if (status < 0)
        {
            // Report SD I/O error
            return -EIO;
        }
    }

    // The first byte of the filename have special value.
    // Skip over any deleted files.
    while (*(entryAddr + currOffset + FILENAME) == FILENAME_DELETED)
    {
        // Bump the pointer for the new entry
        currOffset += DIR_BYTES_PER_ENTRY;
    }

    // Get File Attribute
    uint8_t attribute = *(entryAddr + currOffset + FILE_ATTRIBUTE);

    // Processing a short file name
    if (attribute != FILE_FLAG_LFN)
    {
        // Clear the file name incase of previous scans
        memset(entry->name, 0, FILENAME_SIZE + 1);

        // Copy in Filename
        strncpy((char*)entry->name, (char*)(entryAddr + currOffset + FILENAME), FILENAME_SIZE);
        removeSpacePadding(entry->name, FILENAME_SIZE);

        // Copy in File Extension
        strncpylower(entry->ext, (entryAddr + currOffset + FILE_EXTENSION), FILE_EXT_SHORT_SIZE);
        removeSpacePadding(entry->ext, FILE_EXT_SHORT_SIZE);
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
        uint8_t entriesToProcess = LowerNibble(*(entryAddr + currOffset));

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
                unicodeChar = ToLittleEndian(entryAddr + currOffset + entryCharOffsets[k], DATA_SIZE_HALF_WORD);
                temp.longFilename[k] = LOBYTE(unicodeChar);
            }
            // Terminate the end of the string
            temp.longFilename[k] = '\0';

            // Add the LFN entry to the stack
            pushStack(&nameStack, &temp);

            // Bump the pointer for the new entry
            currOffset += DIR_BYTES_PER_ENTRY;

            // Read new block if necessary
            if (currOffset >= rxBufferSize)
            {
                // Increment the base address to the next sector
                if (getFatType(pFAT) == FAT_TYPE_FAT16)
                {
                    currBaseAddr += rxBufferSize;
                }
                else
                {
                    currBaseAddr++;
                }

                currOffset = 0;

                int status = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr, rxBufferSize / pFAT->SystemInfo->BytesPerSector);

                // Read Error Handling
                if (status < 0)
                {
                    // Report SD I/O error
                    return -EIO;
                }
            }
        }

        // Terminate the first character so the string functions work properly
        entry->name[0] = '\0';
        while (!isStackEmpty(&nameStack))
        {
            popStack(&nameStack, &temp);
            // Copy Full Name Back into the file structure
            strncat((char*)entry->name, (char*)temp.longFilename, FILENAME_LF_SIZE + 1);
        }

        /********************* 	 Get File Extension from the LFN 	**********************/
        uint8_t* pstart = entry->name + (strlen((char*)entry->name) - FILE_EXT_LONG_SIZE - 1);
        bool periodFound = false;
        uint8_t extLen = 0;

        entry->ext[0] = '\0';

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
                strncpylower(entry->ext, pstart, extLen);
                // Terminate the the end of the file extension
                entry->ext[extLen] = '\0';
            }
        }

        /*********************************
		 ***  TODO: Process checksum  ****
		 *********************************/
    }

    /***************	Parse File Meta data	***************/

    /* Get File Attribute. This must be obtained again for the actual file
	 * metadata because the previous one could have been for the LFN entry	*/
    attribute = *(entryAddr + currOffset + FILE_ATTRIBUTE);

    if (attribute & FILE_FLAG_DIRECTORY)
    {
        entry->type = ENTRY_TYPE_DIRECTORY;
    }
    else if (attribute & FILE_FLAG_VOLUME)
    {
        entry->type = ENTRY_TYPE_VOLUME;
    }
    else if (attribute & (FILE_FLAG_SYSTEM | FILE_FLAG_HIDDEN))
    {
        entry->type = ENTRY_TYPE_HIDDEN_FILE;
    }
    else
    {
        entry->type = ENTRY_TYPE_FILE;
    }

    // Get File Size
    entry->size = ToLittleEndian(entryAddr + currOffset + FILE_SIZE, FILE_SIZE_SIZE);

    /***************	Save Directory  Block address and offset	***************/
    // Note: These are calculated this way since its possible to read with multi-block buffer

    // Offset into directory block
    entry->context->DirEntryOffset = currOffset % SD_DEFAULT_BLOCK_SIZE;

    // Base address of block
    entry->context->DirEntryBaseAddr = currBaseAddr + ((currOffset / SD_DEFAULT_BLOCK_SIZE) * getFatAddrUnit(pFAT));

    // Get File Starting Cluster
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        entry->context->StartingCluster = ToLittleEndian(entryAddr + currOffset + FILE_START_CLUSTER_LO, FILE_START_CLUSTER_SIZE);
    }
    else if (getFatType(pFAT) == FAT_TYPE_FAT32)
    {
        uint32_t startClusterHi = ToLittleEndian(entryAddr + currOffset + FILE_START_CLUSTER_HI, FILE_START_CLUSTER_SIZE);
        uint32_t startClusterLo = ToLittleEndian(entryAddr + currOffset + FILE_START_CLUSTER_LO, FILE_START_CLUSTER_SIZE);

        entry->context->StartingCluster = MAKEWORD(startClusterHi, startClusterLo);
    }

    // Bump the pointer for the new entry
    currOffset += DIR_BYTES_PER_ENTRY;

    // Update the directory scan state
    dir->context->iterBaseAddr = currBaseAddr;
    dir->context->iterOffset = currOffset;

    // Return 1 if entry found, 0 if end-of-directory
    return isEndOfDir(entry) ? 0 : 1;
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
 * 	@return			     - 0 if found, negative errno on error
 *
 */
static int findFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode, uint32_t startAddr)
{
    uint32_t addrQueueBuf[MAX_DIRECTORIES];
    uint16_t entriesPerDir = MAX_ENTIRES_PER_DIRECTORY;
    uint32_t workingDir = 0;
    uint32_t tempAddr = 0;

    // Initialize a queue for the directory search
    QueueInfo addrQueue = initQueue(addrQueueBuf, MAX_DIRECTORIES, sizeof(uint32_t));

    // Enqueue the starting address
    enqueue(&addrQueue, &startAddr);

    // temporary file to hold just the name and extension
    uint8_t Filename[FILENAME_MAX_SIZE + 1] = {0};
    uint8_t FileExt[FILE_EXT_LONG_SIZE + 1] = {0};

    // Parse the filename and extension from the user input
    parseFilename((char*)fileName, Filename, FileExt);

    // Temporary directory entry for scanning
    struct file_context_t context;

    file_entry_t tempDir = {
        .context = &context,
    };

    // Loop until the queue is empty
    do
    {
        // Reset entry count each directory
        uint32_t entryCount = 0;

        // Get the working address from the front of the queue
        peekQueue(&addrQueue, &workingDir);

        // Read the first blocks of the working directory
        int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, workingDir, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo->BytesPerSector);

        // Read Error Handling
        if (cmdStatus < 0)
        {
            // Report Issue
            return -EIO;
        }

        // If we are in the root directory there is a limit to the number of entries
        if (getFatType(pFAT) == FAT_TYPE_FAT16)
        {
            entriesPerDir = (workingDir == pFAT->SystemInfo->RootDirAddress) ? pFAT->SystemInfo->RootDirEntries : MAX_ENTIRES_PER_DIRECTORY;
        }

        // Initialize temporary directory entry for scanning
        tempDir.context->iterBaseAddr = workingDir;
        tempDir.context->iterOffset = 0;

        // Scan Until we find the end of the directory
        int readDirResult;
        while ((entryCount < entriesPerDir) && (readDirResult = FAT_ReadDir(pFAT, &tempDir, file)) > 0)
        {
            // Increment the entry count
            entryCount++;

            // Match the file names
            if (strcmp((char*)file->name, (char*)Filename) == 0 && strcmp((char*)file->ext, (char*)FileExt) == 0)
            {
                // Found the file, so exit the function
                return 0;
            }

            // If we have found a new directory, add it to the queue
            if (file->type == ENTRY_TYPE_DIRECTORY && (mode == SEARCH_DIR_RECURSIVE || mode == SEARCH_FILE_RECURSIVE))
            {
                tempAddr = getClusterAddr(pFAT, file->context->StartingCluster);
                enqueue(&addrQueue, &tempAddr);
            }
        }

        // queue the directory at the head of the queue
        dequeue(&addrQueue, &tempAddr);

    } while (!isQueueEmpty(&addrQueue));

    // Return file not found
    return -ENOENT;
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
 * 	@return			     - 0 if found, negative errno on error
 *
 */
int findDirectory(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode, uint32_t startAddr)
{
    // Search for file
    int searchStatus = findFile(pFAT, fileName, file, mode, startAddr);

    if (searchStatus == 0 && file->type != ENTRY_TYPE_DIRECTORY)
    {
        // Found but not a directory
        return -ENOTDIR;
    }

    return searchStatus;
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
static int searchPath(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file)
{
    uint8_t n, delimeterLoc;
    uint8_t pathSegment[FILENAME_MAX_SIZE + 1];
    uint8_t pathBuf[FILENAME_MAX_SIZE + 1];

    // Create mutable copy of path to handle string constants
    strncpy((char*)pathBuf, (char*)path, FILENAME_MAX_SIZE);
    pathBuf[FILENAME_MAX_SIZE] = '\0';

    uint8_t* workingPath = pathBuf;
    uint8_t nameLen = strlen((char*)workingPath);

    // All paths are absolute: start from root directory
    uint32_t currSearchDir = pFAT->SystemInfo->RootDirAddress;

    if (strncmp((char*)workingPath, "/", 1) == 0)
    {
        if (strlen((char*)workingPath) == 1)
        {
            // If the path is just "/" we are looking for the volume label
            strcpy((char*)workingPath, (char*)pFAT->SystemInfo->VolumeLabel);
            nameLen = strlen((char*)workingPath);
        }
        else
        {
            workingPath += 1;
            nameLen -= 1;
        }
    }

    // Loop through the input and search for sub directories
    for (n = 0, delimeterLoc = 0; n < nameLen; n++)
    {
        // Look for the delimiter
        if (workingPath[n] == '/')
        {
            if (delimeterLoc != n)
            {
                strncpy((char*)pathSegment, (char*)&workingPath[delimeterLoc], n - delimeterLoc);
                pathSegment[n - delimeterLoc] = '\0';

                // Search for the directory segment starting from currSearchDir
                int dirResult = findDirectory(pFAT, pathSegment, file, SEARCH_DIR_LOCAL, currSearchDir);
                if (dirResult != 0)
                {
                    return dirResult;
                }

                // Descend into the found sub-directory
                currSearchDir = getClusterAddr(pFAT, file->context->StartingCluster);
            }

            // Update delimiter location
            delimeterLoc = n + 1;
        }
    }

    // Handle the case when the string ends in a filename
    // Ex: Recipes/Baked Beans.txt
    if (delimeterLoc != n)
    {
        strncpy((char*)pathSegment, (char*)&workingPath[delimeterLoc], n - delimeterLoc);
        pathSegment[n - delimeterLoc] = '\0';

        // Search for the file starting from currSearchDir
        int fileResult = findFile(pFAT, pathSegment, file, SEARCH_FILE_LOCAL, currSearchDir);
        if (fileResult != 0)
        {
            return fileResult;
        }
    }

    return 0;
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
static int updateDirEntry(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // Confirm the file mode is for writing
    if (file->mode != FILE_MODE_WRITE && file->mode != FILE_MODE_WRITE_NEW)
    {
        return -EINVAL;
    }

    uint8_t* entryBlock = SD_GetBuffAddr(pFAT->pSDHandle);
    uint8_t* entryfileSize = entryBlock + file->context->DirEntryOffset + FILE_SIZE;

    // First read the block that holds the file Directory Entry
    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, 1);

    // Read Error Handling
    if (cmdStatus < 0)
    {
        // Report Issue
        return -EIO;
    }

    // TODO: Remove these debug
#if defined(FAT_DEBUG_GENERIC)
    HexdumpBuffer(entryBlock + file->context->DirEntryOffset, DIR_BYTES_PER_ENTRY);
#endif

    if (file->size == ToLittleEndian(entryfileSize, FILE_SIZE_SIZE))
    {
        // Nothing has changed - success with no operation
        return 0;
    }

    ToEndianBuf(entryfileSize, file->size, DATA_SIZE_WORD);

#if defined(FAT_DEBUG_GENERIC)
    HexdumpBuffer(entryBlock + file->DirEntryOffset, DIR_BYTES_PER_ENTRY);
#endif

    // Write updates to the Directory block
    cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, 1);

    return (cmdStatus < 0) ? -EIO : 0;
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
static int createFile(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file)
{
    // Search for the file
    int searchStat = searchPath(pFAT, fileName, file);

    // If file was found (searchStat == 0), it already exists
    if (searchStat == 0)
    {
        return -EEXIST;
    }
    
    // If it's a legitimate "not found" error, we can proceed to create
    // Otherwise pass through other errors (I/O errors, etc.)
    if (searchStat != -ENOENT)
    {
        return searchStat;
    }

    /***********  Create a new entry in the FAT table  ***********/

    // First Find the next free ClusterID. Start at the first valid clusterID
    file->context->StartingCluster = findNextFreeClusterID(pFAT, 2);

#ifndef FAT_DEBUG_GENERIC
    // Create the new entry in the FAT
    if (updateClusterID(pFAT, file->context->StartingCluster, file->context->StartingCluster) != 0)
    {
        return -EIO;
    }
#endif

    /***********  Create new directory entry  ***********/

    // Grab the current buffer for later
    uint8_t* entryAddr = SD_GetBuffAddr(pFAT->pSDHandle) + file->context->DirEntryOffset;

    // Parse the filename and extension from the user input and remove path from fileName
    fileName += parseFilename((char*)fileName, file->name, file->ext);

    // Create the 8.3 filename from the input
    uint8_t shortFilename[FILENAME_SIZE + FILE_EXT_SHORT_SIZE + 1] = {0};
    uint8_t checksum = getShortFilename(file, shortFilename);

    /***********  Write Long Filename Entries ***********/

    // Read the Directory block
    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, 1);

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
        if (file->context->DirEntryOffset < (SD_GetBuffSize(pFAT->pSDHandle) - DIR_BYTES_PER_ENTRY))
        {
            // Increment the directory entry offset
            file->context->DirEntryOffset += DIR_BYTES_PER_ENTRY;
        }
        else
        {

#ifndef FAT_DEBUG_GENERIC
            // Write the block to memory
            cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, 1);
#else
            // Debug the new entry
            HexdumpBuffer(SD_GetBuffAddr(pFAT->pSDHandle), pFAT->SystemInfo->BytesPerSector);
#endif
            // Reset the directory entry
            file->context->DirEntryOffset = 0;

            // Increment the directory base address
            uint32_t sectorsPerBuffer = SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo->BytesPerSector;
            file->context->DirEntryBaseAddr += sectorsPerBuffer * getFatAddrUnit(pFAT);

            // Read next block of memory
            cmdStatus = SD_ReadBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, 1);
        }

        // Grab the current buffer for later
        entryAddr = SD_GetBuffAddr(pFAT->pSDHandle) + file->context->DirEntryOffset;

        // Decrement the entry count
        lfnEnties--;
    }

    /***********  Short Filename Entry Information ***********/

    // Copy in the filename & extension in 8.3 format
    strncpy((char*)entryAddr + FILENAME, (char*)shortFilename, FILENAME_SIZE + FILE_EXT_SHORT_SIZE);

    // Fill in the starting cluster
    if (getFatType(pFAT) == FAT_TYPE_FAT16)
    {
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_LO, file->context->StartingCluster, FILE_START_CLUSTER_SIZE);
    }
    else
    {
        // TODO: Debug this.
        // Grab lower Portion of the word
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_LO, (file->context->StartingCluster & 0xFFFF), FILE_START_CLUSTER_SIZE);

        // Grab uower Portion of the word
        ToEndianBuf(entryAddr + FILE_START_CLUSTER_HI, ((file->context->StartingCluster >> 16) & 0xFFFF), FILE_START_CLUSTER_SIZE);
    }

    // Attribute Byte set to archived.
    *(entryAddr + FILE_ATTRIBUTE) |= FILE_FLAG_ARCHIVED;

    // Starting Date for the file Jan 1, 1980
    Date_format_t date = {1, 1, 0};

    *((pDateFormat)(entryAddr + FILE_DATE_MODIFIED)) = date;

    // Calculate the number of blocks to write
    uint8_t writeCount = (file->context->DirEntryOffset < pFAT->SystemInfo->BytesPerSector) ? 1 : 2;

#ifndef FAT_DEBUG_GENERIC
    // Update the block
    cmdStatus = SD_WriteBlock(pFAT->pSDHandle, file->context->DirEntryBaseAddr, writeCount);
#else
    // Debug the new entry
    HexdumpBuffer(SD_GetBuffAddr(pFAT->pSDHandle), pFAT->SystemInfo->BytesPerSector * writeCount);
#endif

    return (cmdStatus < 0) ? -EIO : 0;
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
    strncpy((char*)filename, (char*)file->name, FILENAME_SIZE);

    uint8_t nameLen = strlen((char*)file->name);

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
    strncat((char*)filename, (char*)file->ext, FILENAME_SIZE + FILE_EXT_SHORT_SIZE);

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
 *	@fn              - FAT_fopen
 *
 * 	@brief           - Find a file and load the NodesQueue up
 *
 * 	@param[pFAT]     - Handler structure for FAT
 * 	@param[path]     - name of the file
 * 	@param[file]     - Memory location the file details
 * 	@param[mode]     - File Mode
 *
 * 	@return          - Search status
 *
 */
int FAT_fopen(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file, file_mode_t mode)
{
    // Error Checking
    if ((FAT_getStat(pFAT) != INIT_FAT_SUCCESS) || (mode != FILE_MODE_READ && mode != FILE_MODE_WRITE && mode != FILE_MODE_WRITE_NEW))
    {
        return -EINVAL;
    }

    // Point the file context to the single context instance.
    file->context = &fileContext;
    memset(file->context, 0, sizeof(struct file_context_t));

    // This mode creates a new file if it does not exist
    if (mode == FILE_MODE_WRITE_NEW)
    {
        int ret = createFile(pFAT, path, file);
        if (ret != 0)
        {
            return ret;  // Pass through error from createFile
        }
    }

    // Search for the file
    if (searchPath(pFAT, path, file) != 0)
    {
        return -ENOENT;
    }

    file->mode = mode;
    file->state = FILE_STATE_IDLE;

    /*******************************************
     *   Found the file, so init NodeQueue     *
     *******************************************/

    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->context->NodesQueue;

    // Init Queue to hold the ClustersIds of the file
    pNodesQueue->Info = initQueue(FileNodesBuf, FAT_QUEUE_MAX_CLUSTERS, sizeof(uint32_t));

    // Initialized the tail to Zero which is not an acceptable clusterID
    pNodesQueue->Tail = NODES_QUEUE_TAIL_INIT;

    // Initialize directory scan state if this is a directory
    if (file->type == ENTRY_TYPE_DIRECTORY
        || file->type == ENTRY_TYPE_VOLUME)
    {
        file->context->iterBaseAddr = getClusterAddr(pFAT, file->context->StartingCluster);
        file->context->iterOffset = 0;

        uint32_t rxBufferSize = SD_GetBuffSize(pFAT->pSDHandle);
        SD_ReadBlock(pFAT->pSDHandle, file->context->iterBaseAddr, rxBufferSize / pFAT->SystemInfo->BytesPerSector);

        // We don't need to load up the NodesQueue for a directory
        return 0;
    }

    // Load Up the NodesQueue
    int loadStatus = loadFileNodesQueue(pFAT, file, mode);

    // Return Types: 0 or 1 for success (EOF found or not found), negative errno on error
    if (loadStatus >= 0)
    {
        return 0;
    }

    return loadStatus;
}

/****************************************************************************************
 *	@fn 			     - loadFileNodesQueue
 *
 * 	@brief			     - Read from the FAT table and load up the nodes queue
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 * 	@return			     - 0 on success, negative errno on error
 *
 */
int loadFileNodesQueue(FAT_Handle_t* pFAT, file_entry_t* file, file_mode_t mode)
{
    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->context->NodesQueue;

    // Error Checking
    // TODO: Determine if this error handling needs updates
    if (isEndofFatEntry(pFAT, pNodesQueue->Tail) || isQueueFull(&pNodesQueue->Info) || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS))
    {
        return -EINVAL;
    }

    uint32_t currClusterID;

    // The file has just been opened, determine the starting ClusterID
    if (pNodesQueue->Tail == NODES_QUEUE_TAIL_INIT)
    {
        if (mode == FILE_MODE_WRITE || mode == FILE_MODE_WRITE_NEW)
        {
            // Find the last cluster ID in the file
            uint32_t lastClusterID = traverseTable(pFAT, pNodesQueue, file->context->StartingCluster, FAT_TRAVERSE_FIND_LAST_ID);
            
            if (lastClusterID == 0)
            {
                return -EIO;
            }
            
            // Store the ending cluster for write operations
            file->context->EndingCluster = lastClusterID;

            // Starting from the End of the file
            currClusterID = file->context->EndingCluster;

            // Determine if the end of the file is mid-cluster
            // TODO: This may need some updates
            if (file->size % (pFAT->SystemInfo->BytesPerSector * pFAT->SystemInfo->SectorsPerCluster))
            {
                enqueue(&pNodesQueue->Info, &currClusterID);
            }
        }
        else
        {
            // Starting from the begining of the file
            currClusterID = file->context->StartingCluster;
        }
    }
    else
    {
        // Starting mid-file
        uint32_t loadedBaseAddr = 0;
        currClusterID = getNextClusterID(pFAT, pNodesQueue->Tail, &loadedBaseAddr);

        // Block Read Failure
        if (currClusterID == 0)
        {
            return -EIO;
        }
    }

    /**  Fwrite - Fill NodesQueue up with the next free ClusterIDs ***/
    if (mode == FILE_MODE_WRITE || mode == FILE_MODE_WRITE_NEW)
    {
        int floadStatus = loadFreeClusterIDs(pFAT, pNodesQueue, currClusterID);
        if (floadStatus < 0)
        {
            return floadStatus;  // Pass through error
        }
        return 0;  // Success
    }

    /**  Read - Fill NodesQueue file ClusterIDs ***/

    // Loop until ether the queue is full or EOF is found and Report the status
    // Store the ending cluster
    file->context->EndingCluster = traverseTable(pFAT, pNodesQueue, currClusterID, FAT_TRAVERSE_LOAD_QUEUE);

    if (file->context->EndingCluster == 0)
    {
        return -EIO;
    }

    // Determine if EOF was reached by checking if pNodesQueue->Tail is an EOF marker
    if (isEndofFatEntry(pFAT, pNodesQueue->Tail))
    {
        return 0;  // EOF found
    }

    return 1;  // EOF not found
}

/****************************************************************************************
 *	@fn 			     - FAT_fclose
 *
 * 	@brief			     - Empty the nodes queue
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *
 * 	@return			     - 0 on success, negative errno on error
 *
 */
int FAT_fclose(FAT_Handle_t* pFAT, file_entry_t* file)
{
    // Defining a local variable so the following statements are shorter.
    NodesQueue* pNodesQueue = &file->context->NodesQueue;
    uint32_t tempNode;

    // Close out the file state
    file->state = FILE_STATE_CLOSE;

    if (file->mode == FILE_MODE_WRITE || file->mode == FILE_MODE_WRITE_NEW)
    {
        // Update directory entry and return error code
        return updateDirEntry(pFAT, file);
    }

    // Initialized the tail to Zero which is a non-acceptable clusterID
    pNodesQueue->Tail = NODES_QUEUE_TAIL_INIT;

    // Empty out the NodesQueue
    while (!isQueueEmpty(&pNodesQueue->Info))
    {
        // queue the directory at the head of the queue
        dequeue(&pNodesQueue->Info, &tempNode);
    }

    return 0;
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
int FAT_fread(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t** data, uint32_t* size)
{
    NodesQueue* pNodesQueue = &file->context->NodesQueue;

    static uint32_t sectorsRead = 0;
    static uint32_t currSectorNum = 0;
    static uint32_t currBaseAddr = 0;

    // Exit if the NodesQueue is empty or file mode is not read
    if (isQueueEmpty(&pNodesQueue->Info) 
       || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS) 
       || (file->mode != FILE_MODE_READ))
    {
        return -EINVAL;
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

    System_info_t* systemInfo = pFAT->SystemInfo;
    uint32_t sectorsPerBuffer = SD_GetBuffSize(pFAT->pSDHandle) / systemInfo->BytesPerSector;

    // Only read up to a cluster at a time, even if the buffer can hold more. 
    uint32_t sectorsToRead = sectorsPerBuffer > systemInfo->SectorsPerCluster ? systemInfo->SectorsPerCluster : sectorsPerBuffer;

    // Calculate the number of sectors remaining in the file and adjust to not read past the end of the file
    uint32_t totalFileSectors = (file->size + (systemInfo->BytesPerSector - 1)) / systemInfo->BytesPerSector;
    uint32_t sectorsRemaining = totalFileSectors - sectorsRead;
    sectorsToRead = (sectorsRemaining < sectorsToRead) ? sectorsRemaining : sectorsToRead;

    uint32_t tempNode;
    
    if (currSectorNum == 0)
    {
        // Peak the next node from the Queue, it'd be dequeued when its finished.
        peekQueue(&pNodesQueue->Info, &tempNode);
        
        // Update the current base address
        currBaseAddr = getClusterAddr(pFAT, tempNode);
    }
    
    // Read the current data block
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsToRead);

    // Read Error Handling
    if (cmdStatus < 0)
    {
        // Report Issue
        return -EIO;
    }

    sectorsRead += sectorsToRead;

    // Found the End of File
    if ((sectorsRead * systemInfo->BytesPerSector) >= file->size)
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
        *size = file->size % bufSize;
        SD_ToggleCurrBuff(pFAT->pSDHandle);

        return 0;
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
            int loadStatus = loadFileNodesQueue(pFAT, file, FILE_MODE_READ);

            // fload writes over the block buffer, so we need to re-read the current block.
            cmdStatus = SD_ReadBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsPerBuffer);

            // Read Error Handling
            if (cmdStatus < 0 || loadStatus != 0)
            {
                // Failure Could have while loading the queue.
                while (!isQueueEmpty(&pNodesQueue->Info))
                {
                    dequeue(&pNodesQueue->Info, &tempNode);
                }

                // Report Issue
                return -EIO;
            }
        }

        // Reset the current Sector number
        currSectorNum = 0;
    }

    // Set output parameters
    *data = SD_GetBuffAddr(pFAT->pSDHandle);
    *size = sectorsToRead * systemInfo->BytesPerSector;
    SD_ToggleCurrBuff(pFAT->pSDHandle);

    return 0;
}

/****************************************************************************************
 *	@fn 			     - FAT_fwrite
 *
 * 	@brief			     - Write Block(s) of data from a file
 *
 *  @param[pFAT]         - Handler structure for FAT
 *  @param[file]         - Memory location the file details
 *  @param[buffer]       - Pointer to data buffer to write
 *  @param[size]         - Number of bytes to write from buffer
 *
 * 	@return			     - 0 on success, negative error code on failure
 *
 */
int FAT_fwrite(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t* buffer, uint32_t size)
{
    NodesQueue* pNodesQueue = &file->context->NodesQueue;

    static uint32_t currSectorNum = 0;
    static uint32_t currBaseAddr = 0;

    // Parameter validation
    if (buffer == NULL)
    {
        return -EINVAL;
    }

    if (size == 0)
    {
        return -EINVAL;
    }

    uint32_t sdBufSize = SD_GetBuffSize(pFAT->pSDHandle);
    if (size > sdBufSize)
    {
        return -EINVAL;
    }

    // Exit if the NodesQueue is empty or file mode is not write
    if (isQueueEmpty(&pNodesQueue->Info) || (FAT_getStat(pFAT) != INIT_FAT_SUCCESS) || (file->mode != FILE_MODE_WRITE && file->mode != FILE_MODE_WRITE_NEW))
    {
        return -EINVAL;
    }

    // Copy user buffer to SD buffer and clear remainder
    uint8_t* sdBuf = SD_GetBuffAddr(pFAT->pSDHandle);
    memcpy(sdBuf, buffer, size);
    memset(sdBuf + size, 0, sdBufSize - size);

    uint32_t tempNode;
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    
    System_info_t* systemInfo = pFAT->SystemInfo;
    uint32_t sectorsPerBuffer = sdBufSize / systemInfo->BytesPerSector;

    // Only write up to a cluster at a time, even if the buffer can hold more. 
    uint32_t sectorsToWrite = sectorsPerBuffer > systemInfo->SectorsPerCluster ? systemInfo->SectorsPerCluster : sectorsPerBuffer;

    // Reset the Static variables on the first write
    if (file->state == FILE_STATE_IDLE)
    {
        // Update the file state
        file->state = FILE_STATE_WRITE;

        /*** Determine where to start writing ***/
        
        // Calculate how many bytes are written to the final cluster
        uint32_t bytesPerFinalCluster = file->size % (systemInfo->BytesPerSector * systemInfo->SectorsPerCluster);

        // Peak the next node from the Queue, it'd be dequeued when its finished.
        peekQueue(&pNodesQueue->Info, &tempNode);

        // Calculate current sector number and base address
        currSectorNum = bytesPerFinalCluster / systemInfo->BytesPerSector;
        currBaseAddr = getClusterAddr(pFAT, tempNode) + (currSectorNum * addrUnit);
    }
    else if (file->state == FILE_STATE_WRITE)
    {
        if (currSectorNum == 0)
        {
            // Peak the next node from the Queue, it'd be dequeued when its finished.
            peekQueue(&pNodesQueue->Info, &tempNode);

            // Update the current base address
            currBaseAddr = getClusterAddr(pFAT, tempNode);
        }
    }
    else
    {
        return -EINVAL;
    }

    // Write the current data block
    if (SD_WriteBlock(pFAT->pSDHandle, currBaseAddr + (currSectorNum * addrUnit), sectorsToWrite) < 0)
    {
        // Report Issue
        return -EIO;
    }

    // Determine how File size needs to be updated
    if (file->context->EndingCluster == tempNode)
    {
        uint32_t finalBlockOffset = file->size % size;

        file->size += size - finalBlockOffset;
    }
    else
    {
        file->size += size;
    }

    // Determine how the current sector needs to be incremented
    if (currSectorNum < pFAT->SystemInfo->SectorsPerCluster - sectorsToWrite)
    {
        currSectorNum += sectorsToWrite;
    }
    else
    {
        // Done processing the current cluster
        dequeue(&pNodesQueue->Info, &tempNode);

        // Update FAT Table Entry
        if (file->context->EndingCluster != tempNode)
        {
            int ret = updateClusterID(pFAT, file->context->EndingCluster, tempNode);
            if (ret != 0)
            {
                return ret;  // Pass through error
            }
        }

        // Update NodesQueue Tail
        file->context->EndingCluster = tempNode;

        // Reset the current Sector number
        currSectorNum = 0;
    }

    return 0;
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
    NodesQueue* pNodesQueue = &file->context->NodesQueue;

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
int FAT_readHeaderBlock(FAT_Handle_t* pFAT, file_entry_t* file)
{
    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, getClusterAddr(pFAT, file->context->StartingCluster), 1);

    // Return read status
    return (cmdStatus < 0) ? -EIO : 0;
}

/****************************************************************************************
 *	@fn 			     - getClusterAddress
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
uint32_t getClusterAddr(FAT_Handle_t* pFAT, uint32_t ClusterID)
{
    uint32_t addrUnit = getFatAddrUnit(pFAT);

    if (ClusterID < 2)
    {
        return pFAT->SystemInfo->RootDirAddress;
    }
    else
    {
        return (pFAT->SystemInfo->DataStartAddress + (ClusterID - 2) * (pFAT->SystemInfo->SectorsPerCluster * addrUnit));
    }
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
    return file->name[0] == END_OF_DIRECTORY;
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
static void getFatEntryAddr(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t* pBaseAddr, uint32_t* pOffset)
{
    uint32_t clusterIdSize, clusterIdBlock;

    // Determine the size of the ClusterIDs
    clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Determine which block the entry resides in
    clusterIdBlock = (clusterID * clusterIdSize) / pFAT->SystemInfo->BytesPerSector;

    // Calculate the base address of block that holds the cluster
    uint32_t addrUnit = getFatAddrUnit(pFAT);
    *pBaseAddr = pFAT->SystemInfo->FAT1_Address + clusterIdBlock * addrUnit;

    // Calculate the offset into that block
    *pOffset = (clusterID * clusterIdSize) % pFAT->SystemInfo->BytesPerSector;
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
static uint32_t getNextClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t* pLoadedBaseAddr)
{
    uint32_t clusterBaseAddr, clusterOffset;
    getFatEntryAddr(pFAT, clusterID, &clusterBaseAddr, &clusterOffset);

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Only read a new block when the cluster entry is in a different sector
    if (*pLoadedBaseAddr != clusterBaseAddr)
    {
        *pLoadedBaseAddr = clusterBaseAddr;

        int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, clusterBaseAddr, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo->BytesPerSector);

        // If the command fails, send invalid clusterID
        if (cmdStatus < 0)
        {
            return 0;
        }
    }

    return ToLittleEndian(SD_GetBuffAddr(pFAT->pSDHandle) + clusterOffset, clusterIdSize);
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
    uint32_t entryBaseAddr, entryOffset;

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Track which FAT block is currently loaded to avoid redundant SD reads
    uint32_t loadedBaseAddr = 0;

    uint32_t ClusterIdValue = 0xFFFF;

    while (ClusterIdValue != FAT_FREE_ID_MARKER)
    {
        // Increment to the next clusterID
        clusterID++;

        // Look to the next ClusterID
        getFatEntryAddr(pFAT, clusterID, &entryBaseAddr, &entryOffset);

        // Only read a new block when the cluster entry is in a different sector
        if (loadedBaseAddr != entryBaseAddr)
        {
            loadedBaseAddr = entryBaseAddr;

            // Read new block
            int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, entryBaseAddr, SD_GetBuffSize(pFAT->pSDHandle) / pFAT->SystemInfo->BytesPerSector);

            // If the command fails, send invalid clusterID
            if (cmdStatus < 0)
            {
                return 0;
            }
        }

        ClusterIdValue = ToLittleEndian(SD_GetBuffAddr(pFAT->pSDHandle) + entryOffset, clusterIdSize);
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
static int updateClusterID(FAT_Handle_t* pFAT, uint32_t clusterID, uint32_t nextID)
{

    uint32_t clusterBaseAddr, clusterOffset;
    uint32_t nextBaseAddr, nextOffset;
    getFatEntryAddr(pFAT, clusterID, &clusterBaseAddr, &clusterOffset);
    getFatEntryAddr(pFAT, nextID, &nextBaseAddr, &nextOffset);
    uint8_t* fatBlock = SD_GetBuffAddr(pFAT->pSDHandle);

    // Determine the size of the ClusterIDs
    DataSize_t clusterIdSize = (getFatType(pFAT) == FAT_TYPE_FAT16) ? DATA_SIZE_HALF_WORD : DATA_SIZE_WORD;

    // Read the FAT block
    int cmdStatus = SD_ReadBlock(pFAT->pSDHandle, clusterBaseAddr, 1);

#if defined(FAT_DEBUG_TABLE)
    HexdumpBuffer(fatBlock, pFAT->SystemInfo->BytesPerSector);
#endif

    // Error Handling
    if (cmdStatus < 0)
    {
        return -EIO;
    }

    // If both ClusterID are in the same block it can be updated in one write
    if (clusterBaseAddr == nextBaseAddr)
    {
        // Update the ClusterIDs
        ToEndianBuf(fatBlock + clusterOffset, nextID, clusterIdSize);
        // NextID Terminated with EOF
        ToEndianBuf(fatBlock + nextOffset, FAT_EOF_MARKER_GENERIC, clusterIdSize);

#ifndef FAT_DEBUG_TABLE
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, clusterBaseAddr, 1);
#endif
    }
    // Otherwise it will be updated in two read/writes
    else
    {
#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo->BytesPerSector / 4);
#endif

        // Update first ClusterID
        ToEndianBuf(fatBlock + clusterOffset, nextID, clusterIdSize);
#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo->BytesPerSector / 4);
#else
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, clusterBaseAddr, 1);
#endif

        // Read Block and Update nextID
        cmdStatus = SD_ReadBlock(pFAT->pSDHandle, nextBaseAddr, 1);
        // NextID Terminated with EOF
        ToEndianBuf(fatBlock + nextOffset, FAT_EOF_MARKER_GENERIC, clusterIdSize);

#if defined(FAT_DEBUG_TABLE)
        HexdumpBuffer(fatBlock, pFAT->SystemInfo->BytesPerSector / 4);
#else
        // Write the FAT Block
        cmdStatus = SD_WriteBlock(pFAT->pSDHandle, nextBaseAddr, 1);
#endif
    }

#if defined(FAT_DEBUG_TABLE)
    HexdumpBuffer(fatBlock, pFAT->SystemInfo->BytesPerSector);
#endif

    // Error Handling
    if (cmdStatus < 0)
    {
        return -EIO;
    }
    else
    {
        return 0;
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
 *  @return               - 0 if queue full, negative errno on error
 *
 *  @note                 - 
 */
static int loadFreeClusterIDs(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster)
{
    uint32_t currClusterID = startCluster;

    // Loop until the NodesQueue is full
    while (!isQueueFull(&pNodesQueue->Info))
    {
        currClusterID = findNextFreeClusterID(pFAT, currClusterID);

        // Block Read Failure
        if (currClusterID == 0)
        {
            return -EIO;
        }

        enqueue(&pNodesQueue->Info, &currClusterID);
    }

    return 0;  // Queue is full (EOF found)
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
    return pFAT->SystemInfo->FAT_Type;
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
    return (getFatType(pFAT) == FAT_TYPE_FAT16) ? pFAT->SystemInfo->BytesPerSector : 1;
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
static uint32_t traverseTable(FAT_Handle_t* pFAT, NodesQueue* pNodesQueue, uint32_t startCluster, fat_traverse_mode_t mode)
{
    uint32_t currClusterID = startCluster;
    uint32_t loadedBaseAddr = 0;

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
        currClusterID = getNextClusterID(pFAT, currClusterID, &loadedBaseAddr);

        // Block Read Failure
        if (currClusterID == 0)
        {
            // TODO: Empty Queue?
            return 0; // Return 0 on failure
        }

    } while (!isQueueFull(&pNodesQueue->Info) && !isEndofFatEntry(pFAT, currClusterID));

    // Save the last valid cluster ID before updating Tail to EOF marker
    uint32_t lastValidClusterID = pNodesQueue->Tail;

    // Update the Queue Tail to EOF marker
    pNodesQueue->Tail = currClusterID;

    // Return the last valid cluster ID (not the EOF marker)
    return lastValidClusterID;
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
