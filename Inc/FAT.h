/****************************************
 * FAT.h
 *
 *  Created on: Nov 22th, 2020
 *      Author: thomashamilton
 ****************************************/

#ifndef INC_FAT_H_
#define INC_FAT_H_

// Standard Libraries
#include <stdint.h>
#include "SD_Card.h"
#include "Queue.h"

/************************************************************************************
 *							 Directory Macros										*
 ************************************************************************************/
#define DIR_BYTES_PER_ENTRY 32

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

// Maximum number of entries supported by VFAT. (255 bytes / 13 bytes per entry)
#define VFAT_MAX_ENTRIES 20
// Maximum number of entries this driver supports. (Max LFN is 52 characters)
#define MAX_LFN_ENTRIES 4

/**********************************
 *    Directory File Macros 	  *
 **********************************/
typedef enum
{
    FILENAME_SIZE = 8,
    FILENAME_LF_SIZE = 13,
    FILENAME_MAX_SIZE = (FILENAME_LF_SIZE * MAX_LFN_ENTRIES),
    FILE_EXT_SHORT_SIZE = 3,
    FILE_EXT_LONG_SIZE = 4,
    FILE_FULL_SIZE = FILENAME_SIZE + FILE_EXT_SHORT_SIZE + 1,
    FILE_ATTRIBUTE_SIZE = 1,
    FILE_TIME_MODIFIED_SIZE = 2,
    FILE_DATE_MODIFIED_SIZE = 2,
    FILE_START_CLUSTER_SIZE = 2,
    FILE_SIZE_SIZE = 4
} FAT_file_entry_size_t;

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

typedef struct
{
    QueueInfo Info;
    uint32_t Tail;
} NodesQueue;

typedef enum
{
    FILE_MODE_READ,
    FILE_MODE_WRITE,
    FILE_MODE_WRITE_NEW,
} file_mode_t;

typedef enum
{
    FILE_STATE_FAIL,
    FILE_STATE_IDLE,
    FILE_STATE_WRITE,
    FILE_STATE_READ,
    FILE_STATE_CLOSE,
} file_state_t;

/*
 *  Structure to hold file information
 */
typedef struct
{
    uint8_t Filename[FILENAME_MAX_SIZE + 1];
    uint8_t FileExt[FILE_EXT_LONG_SIZE + 1];
    uint8_t FileAttribute;
    uint16_t TimeModified;
    uint16_t DateModified;
    uint32_t StartingCluster;
    uint32_t EndingCluster;
    uint32_t DirEntryBaseAddr;
    uint32_t DirEntryOffset;
    uint32_t FileSize;
    file_mode_t mode;
    file_state_t state;
    NodesQueue NodesQueue;
} file_entry_t;

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

/**********************************
 *      File search macros		  *
 **********************************/

// Search status
typedef enum
{
    FILESEARCH_FAIL = 0,
    FILESEARCH_NOT_FOUND,
    FILESEARCH_DIR_NOT_FOUND,
    FILESEARCH_FOUND,
} Search_Status_t;

// Search Mode
typedef enum
{
    SEARCH_FILE_LOCAL = 0,
    SEARCH_FILE_RECURSIVE,
    SEARCH_DIR_LOCAL,
    SEARCH_DIR_RECURSIVE,
} Search_Mode_t;

#define MAX_DIRECTORIES 32
#define MAX_ENTIRES_PER_DIRECTORY 256

/*
 *  Structure for System information
 */
#define VOLUME_LABEL_SIZE 11

/************************************************************************************
 *							 FAT Macros										        *
 ************************************************************************************/

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
typedef struct
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
} System_info_t;

/*
 *  Structure for FAT Directory Stack
 */
typedef struct
{
    uint32_t baseAddr;
    uint8_t dirLabel[VOLUME_LABEL_SIZE];
} FAT_Directory_t;

/*
 *  Structure to keep track of working Address
 */
typedef struct
{
    uint32_t workingDir;
    uint32_t baseAddr;
    uint32_t offset;
} WorkingAddr_t;

/************************************************************************************
 *							FAT States												*
 ************************************************************************************/

/**** Master Boot Record States ****/
typedef enum
{
    FAT_UNINITIALIZED = 0,
    INIT_NO_FAT,
    INIT_BAD_END_MARKER,
    INIT_FAT_FAIL,
    INIT_CMD_FAIL,
    INIT_FAT_SUCCESS,
} fat_init_states_t;

/**************************************************
 *           Handle structure for FAT             *
 **************************************************/
typedef struct
{
    fat_init_states_t FAT_Stat;
    SD_Handle_t* pSDHandle;
    System_info_t SystemInfo;
    WorkingAddr_t WorkingAddr;
    file_entry_t* CurrFile;
} FAT_Handle_t;

/**** Macros for Master Boot Record ****/
typedef enum
{
    MBR_TYPE_CODE = 0x01C2,
    MBR_LBA = 0x01C6,
    MBR_END_MARKER = 0x01FE,
} mbr_offsets_t;



/**************************************************
 *           FAT File I/O Return Types            *
 **************************************************/

// Fopen Return type
typedef enum
{
    FOPEN_FAIL = 0,
    FOPEN_NOP,
    FOPEN_NOT_FOUND,
    FOPEN_SUCCESS,
} fat_open_t;

// Fread Return type
typedef enum
{
    FREAD_FAIL = 0,
    FREAD_NOP,
    FREAD_EOF_FOUND,
    FREAD_DONE,
} fat_fread_t;

// Fwrite Return type
typedef enum
{
    FWRITE_FAIL = 0,
    FWRITE_NOP,
    FWRITE_SUCCESS,
    FWRITE_DONE,
} fat_fwrite_t;


/************************************************************************************
 *			        		APIs supported by this driver							*
 * 		   For more information about the APIs check the function definitions		*
 ************************************************************************************/

/* FAT Initialization Functions */
void InitFAT(FAT_Handle_t* pFAT, SD_Handle_t* pSDHandle);
void FAT_GetSystemInfo(FAT_Handle_t* pFAT);

/* File Read / Write functions */
fat_open_t FAT_fopen(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, file_mode_t mode);
fat_fread_t FAT_fread(FAT_Handle_t* pFAT, file_entry_t* file);
fat_fwrite_t FAT_fwrite(FAT_Handle_t* pFAT, file_entry_t* file);
void FAT_fclose(FAT_Handle_t* pFAT, file_entry_t* file);
bool FAT_feof(file_entry_t* file);

fat_fread_t FAT_readHeaderBlock(FAT_Handle_t* pFAT, file_entry_t* file);

/* FAT Directory Functions */
void FAT_SetWorkingDir(FAT_Handle_t* pFAT, uint32_t Dir);
void FAT_SetWorkingAddr(FAT_Handle_t* pFAT, uint32_t WorkingAddr, uint32_t offset);
uint32_t FAT_GetWorkingDir(FAT_Handle_t* pFAT);
void FAT_GoToRootDir(FAT_Handle_t* pFAT);

/* FAT Directory / File Searching Functions */
bool FAT_ScanDir(FAT_Handle_t* pFAT, file_entry_t* file);
Search_Status_t FAT_FindDir(FAT_Handle_t* pFAT, uint8_t* fileName, file_entry_t* file, Search_Mode_t mode);

/* FAT File Attribute Functions */
bool FAT_FileFlagStatus(file_entry_t* file, FAT_file_flags_t flag);
bool FAT_isHiddenFile(file_entry_t* file);

/* Other Functions */
uint32_t FAT_GetClusterAddr(FAT_Handle_t* pFAT, uint32_t ClusterID);
fat_types_t getFatType(FAT_Handle_t* pFAT);
uint32_t getFatAddrUnit(FAT_Handle_t* pFAT);

fat_init_states_t FAT_getStat(FAT_Handle_t* pFAT);
// TODO: CompressNodesQueue

/* IRQ Functions */
void FAT_IRQHandling(FAT_Handle_t* pFAT);

/************************************************************************************
 *			        		Externs for other files to use						*
 ************************************************************************************/

 extern FAT_Handle_t FAT;

#endif /* INC_FAT_H_ */
