/******************************************************************
*
* File: sd_card_file_mgr.h
*
* @brief: 
*/

#ifndef SD_CARD_FILE_MGR_H
#define	SD_CARD_FILE_MGR_H

#include "user_types.h"
#include "globdefs.h"

void FS_FileOperations(void) ;
void sd_card_file_mgr_init(void);
void sd_card_file_mgr_task(void);

#endif	/* SD_CARD_FILE_MGR_H */
