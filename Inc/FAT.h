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
#include <stdbool.h>

/**********************************
 *    Directory File Macros 	  *
 **********************************/

// Maximum number of entries this driver supports. (Max LFN is 52 characters)
#define MAX_LFN_ENTRIES 4

#define FILENAME_SIZE 8
#define FILENAME_LF_SIZE 13
#define FILENAME_MAX_SIZE (FILENAME_LF_SIZE * MAX_LFN_ENTRIES)
#define FILE_EXT_SHORT_SIZE 3
#define FILE_EXT_LONG_SIZE 4

#define FAT_DEFAULT_BLOCK_SIZE 512
#define FAT_BUFFER_SIZE (FAT_DEFAULT_BLOCK_SIZE * 2)

// Supported Allocation Unit (cluster) sizes in bytes.
// Note: Because this driver allocates a double buffer, most MCUs 
// cannot support cluster sizes greater than 32KB due to RAM constraints.
#define CLUSTER_SIZE_4KB (16 * FAT_DEFAULT_BLOCK_SIZE)
#define CLUSTER_SIZE_8KB (16 * FAT_DEFAULT_BLOCK_SIZE)
#define CLUSTER_SIZE_16KB (32 * FAT_DEFAULT_BLOCK_SIZE)
#define CLUSTER_SIZE_32KB (64 * FAT_DEFAULT_BLOCK_SIZE)


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

typedef enum 
{
    ENTRY_TYPE_FILE,
    ENTRY_TYPE_DIRECTORY,
    ENTRY_TYPE_VOLUME,
    ENTRY_TYPE_HIDDEN_FILE,
} file_entry_type_t;

// Forward declaration only 
struct file_context_t;

/*
 *  Structure to hold file information
 */
typedef struct
{
    uint8_t name[FILENAME_MAX_SIZE + 1];
    uint8_t ext[FILE_EXT_LONG_SIZE + 1];
    uint32_t size;
    file_mode_t mode;
    file_entry_type_t type;
    file_state_t state;
    struct file_context_t* context;
} file_entry_t;

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

// Forward declarations only - hide the struct definitions
typedef struct System_info_t System_info_t;
struct SD_Handle_t;

/**************************************************
 *           Handle structure for FAT             *
 **************************************************/
typedef struct
{
    fat_status_t FAT_Stat;
    struct SD_Handle_t* pSDHandle;
    System_info_t* SystemInfo;
    uint8_t sector_buf[FAT_BUFFER_SIZE];
} FAT_Handle_t;

/************************************************************************************
 *			        		APIs supported by this driver							*
 * 		   For more information about the APIs check the function definitions		*
 ************************************************************************************/

/* FAT Initialization Functions */
int InitFAT(FAT_Handle_t* pFAT, struct SD_Handle_t* pSDHandle);

/* File Read / Write functions */
int FAT_fopen(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file, file_mode_t mode);
int FAT_fopenDir(FAT_Handle_t* pFAT, uint8_t* path, file_entry_t* file, file_mode_t mode);
int FAT_fread(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t** data, uint32_t* size);
int FAT_fwrite(FAT_Handle_t* pFAT, file_entry_t* file, uint8_t* buffer, uint32_t size);
int FAT_fclose(FAT_Handle_t* pFAT, file_entry_t* file);
bool FAT_feof(file_entry_t* file);

int FAT_readHeaderBlock(FAT_Handle_t* pFAT, file_entry_t* file);

/* FAT Directory / File Searching Functions */
int FAT_ReadDir(FAT_Handle_t* pFAT, file_entry_t* dir, file_entry_t* entry);

fat_status_t FAT_getStat(FAT_Handle_t* pFAT);

/* IRQ Functions */
void FAT_IRQHandling(FAT_Handle_t* pFAT);

uint8_t* FAT_GetBuffAddr(void);
uint32_t FAT_GetBuffSize(void);

/************************************************************************************
 *			        		Externs for other files to use						*
 ************************************************************************************/

 extern FAT_Handle_t FAT;

#endif /* INC_FAT_H_ */
