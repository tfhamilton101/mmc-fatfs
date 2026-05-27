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


/**********************************
 *    Directory File Macros 	  *
 **********************************/

// Maximum number of entries supported by VFAT. (255 bytes / 13 bytes per entry)
#define VFAT_MAX_ENTRIES 20
// Maximum number of entries this driver supports. (Max LFN is 52 characters)
#define MAX_LFN_ENTRIES 4

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
    uint32_t ParentCluster;
    uint32_t DirEntryBaseAddr;
    uint32_t DirEntryOffset;
    uint32_t FileSize;
    file_mode_t mode;
    file_state_t state;
    NodesQueue NodesQueue;
    uint32_t iterBaseAddr;
    uint32_t iterOffset;
} file_entry_t;

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
} fat_status_t;

/**************************************************
 *           Handle structure for FAT             *
 **************************************************/
typedef struct
{
    fat_status_t FAT_Stat;
    SD_Handle_t* pSDHandle;
    System_info_t SystemInfo;
    file_entry_t* CurrFile;
} FAT_Handle_t;

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
fat_status_t InitFAT(FAT_Handle_t* pFAT, SD_Handle_t* pSDHandle);

/* File Read / Write functions */
fat_open_t FAT_fopen(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file, file_mode_t mode);
fat_open_t FAT_fopenDir(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file, file_mode_t mode);
fat_fread_t FAT_fread(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t** data, uint32_t* size);
fat_fwrite_t FAT_fwrite(FAT_Handle_t* pFAT, file_entry_t* file);
void FAT_fclose(FAT_Handle_t* pFAT, file_entry_t* file);
bool FAT_feof(file_entry_t* file);

fat_fread_t FAT_readHeaderBlock(FAT_Handle_t* pFAT, file_entry_t* file);

/* FAT Directory / File Searching Functions */
bool FAT_ScanDir(FAT_Handle_t* pFAT, file_entry_t* dir, file_entry_t* entry);

/* FAT File Attribute Functions */
bool FAT_IsHiddenFile(file_entry_t* file);
bool FAT_IsDirectory(file_entry_t* file);

/* Other Functions */
uint32_t FAT_GetClusterAddr(FAT_Handle_t* pFAT, uint32_t ClusterID);
fat_types_t getFatType(FAT_Handle_t* pFAT);
uint32_t getFatAddrUnit(FAT_Handle_t* pFAT);

fat_status_t FAT_getStat(FAT_Handle_t* pFAT);
// TODO: CompressNodesQueue

/* IRQ Functions */
void FAT_IRQHandling(FAT_Handle_t* pFAT);

/************************************************************************************
 *			        		Externs for other files to use						*
 ************************************************************************************/

 extern FAT_Handle_t FAT;

#endif /* INC_FAT_H_ */
