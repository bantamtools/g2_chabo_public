/* 
 * File:   ExceptionMgr.h
 * Author: sedwards
 *
 * Created on February 12, 2018, 8:07 PM
 */

#ifndef EXCEPTION_MGR_H
#define	EXCEPTION_MGR_H

#include "user_types.h"
#include "globdefs.h"

void log_error(int errcode);
/*  Enumerate errcodes */
typedef enum
{
  ERR_NONE,
  ERR_TMC2666_MTR_AXIS_ID_OUT_OF_RANGE,
  ERR_TMC2666_READBACK_SELECT_RANGE,
  ERR_TMC2666_UNDEFINED_CHOPPER_MODE,
  ERR_TMC2666_UNDEFINED_STALLGUARD_TUNE_STATE,
  ERR_SG_TUNE_MAXED_OUT_BEFORE_ACHIEVING_TARGET,
  ERR_SG_TUNE_MINNED_OUT_BEFORE_ACHIEVING_TARGET,
  ERR_RANGE_FAULT_JSON_CMD_PWM_CALC_PARAMS,
  ERR_UNDEFINED_STEPPER_CTRL_STATE,
  ERR_DIV_BY_ZERO_IN_FUNC_CALC_ABC,
  ERR_SPINDLE_SPEED_CHANGE_TOO_SMALL,
  ERR_SPINDLE_CALC_PWM_DIV_BY_ZERO,
  ERR_SPI_COMM_UNDEFINED_PWRBRD_SET_CMD,
  ERR_SPI_COMM_UNDEFINED_SSL_ID,
  ERR_MOTION_CTRL_TASK_UNDEFINED_STATE,
  ERR_MOTION_CTRL_UNDEF_STPR_SETUP_STATE,
  ERR_COMMS_MGR_START_GROUP_PRINT_ALREADY_STARTED,
  ERR_COMMS_MGR_END_GROUP_PRINT_NOT_STARTED,
  ERRCODE_COUNT
}ErrCodeE;        
  
/* Define a union for exception data */
#define MAX_EXCEPTION_32BIT_WORDS 2
typedef union
{
  UINT32 u32_data[MAX_EXCEPTION_32BIT_WORDS];
  UINT16 u16_data[MAX_EXCEPTION_32BIT_WORDS * sizeof(UINT16)];
  UINT8 byte_data[MAX_EXCEPTION_32BIT_WORDS* sizeof(UINT32)] ; 
}ExceptionDataU;

typedef struct
{
  ErrCodeE errcode;
  ExceptionDataU data;  
}ExceptionDataEntryS;

void ExcMgr_Init(void);
void ExcMgr_LogErr(ErrCodeE errcode, ExceptionDataU *data) ;
ErrCodeE ExcMgr_GetErrCode(void);
BOOL ExcMgr_CheckAnyNewErrCode(void);
#endif	/* EXCEPTION_MGR_H */

