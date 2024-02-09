/**************************************************************************
 * File:  user_types.h
 * Description: 
 * Author: sedwards
 *
 ****************************************************************************/

#ifndef USER_TYPES_H
#define	USER_TYPES_H

#define NEWLINE_CHAR '\n'
#define NULL_CHAR '\0'
#define CR_CHAR '\r'

typedef unsigned char   UINT8;
typedef unsigned char   uchar;
typedef unsigned short  UINT16;
typedef unsigned short  ushort;
typedef unsigned long   UINT32;
typedef char            SINT8;
typedef short           SINT16;
typedef long            SINT32;
typedef unsigned char   BOOL;
typedef uchar Boolean_t ;

#define true 1
#define false 0


typedef struct
{
  UINT8 bit0:1;
  UINT8 bit1:1;
  UINT8 bit2:1;
  UINT8 bit3:1;
  UINT8 bit4:1;
  UINT8 bit5:1;
  UINT8 bit6:1;
  UINT8 bit7:1; 
}U8BitFieldsS;

typedef struct
{
  UINT8 nibble_lsb:4;
  UINT8 nibble_msb:4; 
}U8NibbleFieldS;

typedef struct
{
  UINT8 lsb3:3;
  UINT8 msb1:1;
  UINT8 unused:4;
}U8FastDecayBitFieldS;

typedef union
{
  UINT8 u8_byte;
  U8FastDecayBitFieldS fast_decay_data;
  U8NibbleFieldS u4_data;
  U8BitFieldsS u8_bit_data;
}U8DataU;

typedef struct
{
    unsigned char low_byte;
    unsigned char high_byte;
} U16DataS;
typedef union
{
   UINT16 u16_data;
   U16DataS b_data;
   UINT8 bytes[sizeof(UINT16)];
}U16DataU;

typedef union
{
   UINT32 u32_data;
   UINT8 bytes[sizeof(UINT32)];
}U32DataU;

typedef union
{
    UINT32 u32_data;
    UINT16 u16_data[2];
    UINT8  u8_data[4];
    char   s8_data[4];
}FourByteDataU;
typedef union
{
    float  f32_data;
    UINT32 u32_data;
    UINT16 u16_data[2];
    UINT8  u8_data[4];
    char   s8_data[4];
}FourByteDataWithFloatU;
  /* Enumerate each spi slave select gpio line: */
  typedef enum
  {   
    SSL_X_MTRDRV,
    SSL_Y_MTRDRV,
    SSL_Z_MTRDRV,
    SSL_Y2_MTRDRV,
    SSL_ID_COUNT,
    SSL_LAST_MTRDRV=SSL_Y2_MTRDRV//SSL_Z_MTRDRV Note: adjust beyond Z if using A, B axes
  }SpiSlaveSelectIdE;
typedef enum
{
  MTR_AXIS_X,
  MTR_AXIS_Y,
  MTR_AXIS_Z,          
  MTR_AXIS_Y2,  
  MTR_AXIS_A=MTR_AXIS_Y2,
//obsolete  MTR_AXIS_B,//future
  MTR_AXIS_ID_COUNT,  
  MTR_AXES_COUNT =  MTR_AXIS_ID_COUNT//MTR_AXIS_Y2+1 
}MotorAxisIdE;
//302302923: to do: consolidate direction enum here with 
/* Enumerate stepper lead screw directionality */
typedef enum
{
  DIR_REV,
  DIR_CLK_WISE=DIR_REV,
  DIR_FWD,
  DIR_CNTR_CLK_WISE=DIR_FWD,               
  DIR_COUNT
}DirectionE;

typedef struct
  {
    UINT32 inval_op_c_flg:1;  
    UINT32 div_by_zero_c_flg:1;      
    UINT32 overflow_c_flg:1;
    UINT32 underflow_c_flg:1;    
    UINT32 inexact_c_flg:1;
    UINT32 reserved_6_to_5:2;
    UINT32 idc_flg:1;
    UINT32 reserved_21_to_8:14;
    UINT32 rounding_mode:2;
    UINT32 flush_to_zero_mode:1;
    UINT32 def_nan_mode:1;
    UINT32 reserved_1:2; 
    UINT32 overflow_flg:1;
    UINT32 carry_flg:1;
    UINT32 zero_flg:1;
    UINT32 neg_flg:1;    
  }FpScrS;
  
  typedef  union
  {
    FpScrS _struct;
    UINT32 _u32;
  }FpScrU;
/* GCode convention:
   1.  For Y (mcb+) and Z axis step counts: 
   forward from home, zero, is negative integer value.
   2. for X, forward is positive, reverse is negative integers   
  */
#endif	/* XP_TYPES_H */

