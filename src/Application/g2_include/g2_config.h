
/*
 * g2_config.h - configuration sub-system generic part (see config_app for application part)
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#ifndef CONFIG_H_ONCE
#define CONFIG_H_ONCE

/***** PLEASE NOTE *****
#include "config_app.h"    // is present at the end of this file
*/

/**** Config System Overview and Usage ***
 *
 *  --- Config objects and the config list ---
 *
 *  The config system provides a structured way to get, set and print configuration variables.
 *  The config system operates as a linked list of "NV objects" (nvObj name/value objects,
 *  OK, so they are not really objects) that encapsulate each variable.
 *
 *  The NV list is populated by the text_parser or the JSON_parser depending on the mode.
 *  This way the internals don't care about how the variable is represented or communicated
 *  externally as all internal operations occur on the nvObjs, not the wire form (text or JSON).
 */
/*  --- Config variables, tables and strings ---
 *
 *  Each configuration value is identified by a short mnemonic string (token).
 *  The token is resolved to an index into the cfgArray which is an array of structures
 *  with the static assignments for each variable. The cfgArray contains typed data in
 *  program memory.
 *
 *  Each cfgItem has:
 *   - group string identifying what group the variable is part of or "" if no group
 *   - token string - the token for that variable - pre-pended with the group (if present)
 *   - operations flags - e.g. if the value should be initialized and/or persisted to NVM
 *   - function pointer for formatted print() method for text-mode readouts
 *   - function pointer for get() method - gets value from memory
 *   - function pointer for set() method - sets value and runs functions
 *   - target - memory location that the value is written to / read from
 *   - default value - for cold initialization
 *
 *  The following rules apply to mnemonic tokens
 *   - are up to 6 alphanumeric characters and cannot contain whitespace or separators
 *   - must be unique (non colliding).
 *   - axis tokens start with the axis letter and are typically 3 characters including the axis letter
 *   - motor tokens start with the motor digit and are typically 3 characters including the motor digit
 *   - non-axis or non-motor tokens are 2-6 characters and by convention generally should not start
 *      with: xyzuvwabc0123456789 (but there can be exceptions)
 *
 *  "Groups" are collections of values that mimic REST resources. Groups include:
 *   - axis groups prefixed by "xyzuvwabc"
 *   - motor groups prefixed by "123456"    ("789" are reserved)
 *   - PWM groups prefixed by p1, p2         (p3 - p9 are reserved)
 *   - coordinate system groups prefixed by g54, g55, g56, g57, g59, g28, g30, g92
 *   - a system group is identified by "sys" and contains a collection of otherwise unrelated values
 *
 *  "Uber-groups" are groups of groups that are only used for text-mode printing - e.g.
 *   - group of all motor groups            (m)
 *   - group of all axes groups             (q)
 *   - group of all offset groups           (o)
 *   - group of all digital input groups    (di)
 *   - group of all digital output groups   (do)
 *   - group of all groups                  ($)
 */
/*  --- Making changes and adding new values
 *
 *  Adding a new value to config (or changing an existing one) involves touching the following places:
 *
 *   - Create a new record in cfgArray[]. Use existing ones for examples.
 *
 *   - Create functions for print, get, and set. You can often use existing generic functions for
 *     get and set, and sometimes print. If print requires any custom text it requires it's own function
 *     Look in the modules for examples - e.g. at the end of canoonical_machine.cpp
 *
 *   - The ordering of group displays is set by the order of items in cfgArray. None of the other
 *     orders matter but are generally kept sequenced for easier reading and code maintenance. Also,
 *     Items earlier in the array will resolve token searches faster than ones later in the array.
 *
 *     Note that matching will occur from the most specific to the least specific, meaning that
 *     if tokens overlap the longer one should be earlier in the array: "gco" should precede "gc".
 */
#include <g2core.h>
#include "main.h"

#ifdef CHABO_MINIMILL
#define CHABO_HARDWARE_PLATFORM           "MM"
#elif defined CHABO_MINIMILL_ESC
#define CHABO_HARDWARE_PLATFORM           "MM_ESC"
#elif defined CHABO_PLOTTER
#define CHABO_HARDWARE_PLATFORM           "PLTR"
#elif defined CHABO_LFP
#define CHABO_HARDWARE_PLATFORM           "LFP"
#elif defined CHABO_DCNC
#define CHABO_HARDWARE_PLATFORM           "DCNC"
#endif
#define CHABO_FIRMWARE_BUILD		        301.01   //assigned to: cs.fw_build
#define CHABO_FIRMWARE_BUILD_STRING         "301.01-chabo-dev-RevA"

/* NOT USED: */
#define CHABO_FIRMWARE_VERSION               0.8
#define CHABO_HARDWARE_VERSION              "A"//sme 

/**** nvObj lists ****
 *
 *  Commands and groups of commands are processed internally a doubly linked list of nvObj_t
 *  structures. This isolates the command and config internals from the details of communications,
 *  parsing and display in text mode and JSON mode.
 *
 *  The first element of the list is designated the response header element ("r") but the list
 *  can also be serialized as a simple object by skipping over the header
 *
 *  To use the nvObj list first reset it by calling nv_reset_nv_list(). This initializes the
 *  header, marks the the objects as TYPE_EMPTY (-1), resets the shared string, relinks all objects
 *  with NX and PV pointers, and makes the last element the terminating element by setting its NX
 *  pointer to NULL. The terminating element may carry data, and will be processed.
 *
 *  When you use the list you can terminate your own last element, or just leave the EMPTY elements
 *  to be skipped over during output serialization.
 *
 *  We don't use recursion so parent/child nesting relationships are captured in a 'depth' variable,
 *  This must remain consistent if the curlies are to work out. You should not have to track depth
 *  explicitly if you use nv_reset_nv_list() or the accessor functions like nv_add_integer() or
 *  nv_add_message(). If you see problems with curlies check the depth values in the lists.
 *
 *  Use the nv_print_list() dispatcher for all JSON and text output. Do not simply run through printf.
 */
/*  Token and Group Fields
 *
 *  The nvObject struct (nvObj_t) has strict rules on the use of the token and group fields.
 *  The following forms are legal which support the use cases listed:
 *
 *  Forms
 *    - group is NUL; token is full token including any group profix
 *    - group is populated; token is carried without the group prefix
 *    - group is populated; token is NUL - indicates a group operation
 *
 *  Use Cases
 *    - Lookup full token in cfgArray to get the index. Concatenates grp+token as key
 *    - Text-mode displays. Concatenates grp+token for display, may also use grp alone
 *    - JSON-mode display for single - element value e.g. xvm. Concatenate as above
 *    - JSON-mode display of a parent/child group. Parent is named grp, children nems are tokens
 */
/*  --- nv object string handling ---
 *
 *  It's very expensive to allocate sufficient string space to each nvObj, so nv uses a cheater's
 *  malloc. A single string of length NV_SHARED_STRING_LEN is shared by all nvObjs for all strings.
 *  The observation is that the total rendered output in JSON or text mode cannot exceed the size of
 *  the output buffer (typ 256 bytes), So some number less than that is sufficient for shared strings.
 *  This is all mediated through nv_copy_string(), nv_copy_string_P(), and nv_reset_nv_list().
 */
/*  --- Setting nvObj indexes ---
 *
 *  It's the responsibility of the object creator to set the index. Downstream functions
 *  all expect a valid index. Set the index by calling nv_get_index(). This also validates
 *  the token and group if no lookup exists. Setting the index is an expensive operation
 *  (linear table scan), so there are some exceptions where the index does not need to be set.
 *  These cases are put in the code, commented out, and explained.
 */
/*  --- Other Notes:---
 *
 *  NV_BODY_LEN needs to allow for one parent JSON object and enough children to complete the
 *  largest possible operation - usually the status report.
 */

/***********************************************************************************
 **** DEFINITIONS AND SETTINGS *****************************************************
 ***********************************************************************************/

// Sizing and footprints                // chose one based on # of elements in cfgArray
typedef uint32_t index_t;               // set/get_int is expecting an int32_t
// Stuff you probably don't want to change

#define NUL (char)0x00		//  ASCII NUL char (0) (not "NULL" which is a pointer)
#define STX (char)0x02		// ^b - STX
#define ETX (char)0x03		// ^c - ETX
#define ENQ (char)0x05		// ^e - ENQuire
#define BEL (char)0x07		// ^g - BEL
#define BS  (char)0x08		// ^h - backspace
#define TAB (char)0x09		// ^i - character
#define LF (char)0x0A		// ^j - line feed
#define VT (char)0x0B		// ^k - kill stop
#if 0
#define CR (char)0x0D		// ^m - carriage return
#else // CR is reserved for, defined in HAL for Clock Control register, and, global search shows no reference to CR for carriage return for bantam's usage of tinyG legacy code
#define _CR (char)0x0D		// ^m - carriage return
#endif
#define XON (char)0x11		// ^q - DC1, XON, resume

#define XOFF (char)0x13		// ^s - DC3, XOFF, pause
#define NAK (char)0x15		// ^u - Negative acknowledgement
#define xCAN (char)0x18		// ^x - Cancel, abort
#define ESC (char)0x1B		// ^[ - ESC(ape)
#define SPC (char)0x20		// ' '  Space character
#define DEL (char)0x7F		//  DEL(ete)
                                // defines allocated from stack (not-pre-allocated)
#define NV_FORMAT_LEN 128       // print formatting string max length
#define NV_MESSAGE_LEN 128      // sufficient space to contain end-user messages
                                 // pre-allocated defines (take RAM permanently)
#define NV_MAX_CHILDREN_COUNT  150//sme 11-9-2020 confi_array max members in a group lookup--must expand per large recent added group: vfd
#define NV_SHARED_STRING_LEN 1024 // shared string for string values

#define NV_BODY_LEN NV_MAX_CHILDREN_COUNT           //was 40 body elements - allow for 1 parent + N children
#define NV_EXEC_LEN 10           // elements reserved for exec, which won't directly respond


// (each body element takes about 30 bytes of RAM)

				// pre-allocated defines (take RAM permanently)
				// (each body element takes about 30 bytes of RAM)
// Stuff you probably don't want to change

//#define NV_STATUS_REPORT_LEN NV_MAX_OBJECTS // max number of status report elements - see cfgArray
#define TOKEN_LEN 10//5 sme make big!	// mnemonic token string: group prefix + short token
#define GROUP_LEN 6                     // max length of group prefix
#define NV_FOOTER_LEN 18                // sufficient space to contain a JSON footer array
#define NV_LIST_LEN (NV_BODY_LEN+2)     // +2 allows for a header and a footer
#define NV_EXEC_FIRST (NV_BODY_LEN+2)   // index of the first EXEC nv
#define NV_MAX_OBJECTS (NV_BODY_LEN-1)  // maximum number of objects in a body string
#define NO_MATCH (index_t)0xFFFF

typedef enum {
    TEXT_MODE = 0,                      // sticky text mode
    JSON_MODE,                          // sticky JSON mode
    AUTO_MODE,                          // auto-configure communications mode
    MARLIN_COMM_MODE,                   // sticky marlin-compatibility mode (if compiled in)
} commMode;

typedef enum {
    FLOW_CONTROL_OFF = 0,               // flow control disabled
    FLOW_CONTROL_XON,                   // flow control uses XON/XOFF
    FLOW_CONTROL_RTS                    // flow control uses RTS/CTS
} flowControl;

typedef enum {                      // value typing for config and JSON
    // exception types
    TYPE_SKIP = -2,                 // do not serialize this object (used for filtering)
    TYPE_EMPTY = -1,                // value struct is empty (which is not the same as "NULL")

    // fundamental JSON types       // must be 0-3. Do not change. Used by F_'s and JSON decoding
    TYPE_NULL = 0,                  // value is 'null' (meaning the JSON null value)
    TYPE_BOOLEAN = 1,               // value is "true" (1) or "false"(0)
    TYPE_INTEGER = 2,               // value is an 8 or 32 bit signed integer
    TYPE_STRING = 3,                // value is in string field

    // derived data types           // must be 4-7. Do not change.
    TYPE_FLOAT = 4,                 // value is a floating point number
    TYPE_UINT32 = 5,                // unsigned 32 bit integer (unused)
    TYPE_ARRAY = 6,                 // value is array element count, values are CSV ASCII in string field
    TYPE_DATA = 7,                  // value is blind cast to uint32_t (willbe removed)

    // transient types and types used during JSON processing
    TYPE_PARENT                     // object is a parent to a sub-object
//    TYPE_NULL_STRING,               // empty string - treated as null
//    TYPE_TYPE_ERROR                 // none of the above
} valueType;

/**** operations flags and shorthand ****/

#define F_TYPE_MASK     0x00000007  // 3 LSB's used for data type

#define F_INITIALIZE    0x08        // initialize this item (run set during initialization)
#define F_PERSIST       0x10        // persist this item when set is run
#define F_NOSTRIP       0x20        // do not strip the group prefix from the token
#define F_CONVERT       0x40        // set if unit conversion is required
#define F_ICONVERT      0x80        // set if unit conversion is required AND value is an inverse quantity

// Shorthand
// _n(ull) (used by commands or other case where there is no target data)
// _b(oolean)
// _s(tring)
// _i(nteger)
// _f(loat)
// _t(ext container)
// _d(ata)

#define _n0	    (TYPE_NULL)
#define _nn     (TYPE_NULL | F_NOSTRIP)

#define _s0     (TYPE_STRING)
#define _sip    (TYPE_STRING| F_PERSIST| F_INITIALIZE)
#define _sipn    (TYPE_STRING| F_PERSIST| F_INITIALIZE | F_NOSTRIP)

#define _sn     (TYPE_STRING | F_NOSTRIP)

#define _d0	(TYPE_DATA)
#define _dip    (TYPE_DATA | F_INITIALIZE | F_PERSIST)

#define _b0     (TYPE_BOOLEAN)      // boolean data types (only listing the ones we use)
#define _bip    (TYPE_BOOLEAN | F_INITIALIZE | F_PERSIST)
#define _bin    (TYPE_BOOLEAN | F_INITIALIZE | F_NOSTRIP)
#define _bipn   (TYPE_BOOLEAN | F_INITIALIZE | F_PERSIST | F_NOSTRIP)

#define _i0     (TYPE_INTEGER)      // integer data types (only listing the ones we use)
#define _ii     (TYPE_INTEGER | F_INITIALIZE)
#define _ip     (TYPE_INTEGER | F_PERSIST)
#define _in     (TYPE_INTEGER | F_NOSTRIP)
#define _iin    (TYPE_INTEGER | F_INITIALIZE |F_NOSTRIP)
#define _iip    (TYPE_INTEGER | F_INITIALIZE | F_PERSIST)
#define _iipn   (TYPE_INTEGER | F_INITIALIZE | F_PERSIST | F_NOSTRIP)

#define _f0      (TYPE_FLOAT)    // floating point data types (only listing the ones we use)
#define _fi     (TYPE_FLOAT | F_INITIALIZE)
#define _fp     (TYPE_FLOAT | F_PERSIST)
#define _fn     (TYPE_FLOAT | F_NOSTRIP)
#define _fip    (TYPE_FLOAT | F_INITIALIZE | F_PERSIST)
#define _fic    (TYPE_FLOAT | F_INITIALIZE | F_CONVERT)
#define _fin    (TYPE_FLOAT | F_INITIALIZE | F_NOSTRIP)
#define _fipc   (TYPE_FLOAT | F_INITIALIZE | F_PERSIST | F_CONVERT)
#define _fipn   (TYPE_FLOAT | F_INITIALIZE | F_PERSIST | F_NOSTRIP)
#define _fipi   (TYPE_FLOAT | F_INITIALIZE | F_PERSIST | F_ICONVERT)
#define _fipnc  (TYPE_FLOAT | F_INITIALIZE | F_PERSIST | F_NOSTRIP | F_CONVERT)

/**** Structures ****/

typedef struct nvString {               // shared string object
    uint16_t magic_start;
#if (NV_SHARED_STRING_LEN < 256)
    uint8_t wp;                         // use this string array index value if string len < 256 bytes
#else
    uint16_t wp;                        // use this string array index value is string len > 255 bytes
#endif
    char string[NV_SHARED_STRING_LEN];
    uint16_t magic_end;                 // guard to detect string buffer underruns
} nvStr_t;

typedef struct nvObject {               // depending on use, not all elements may be populated
    struct nvObject *pv;                // pointer to previous object or NULL if first object
    struct nvObject *nx;                // pointer to next object or NULL if last object
    int8_t depth;                       // depth of object in the tree. 0 is root (-1 is invalid)
    char group[GROUP_LEN+1];            // group prefix or NUL if not in a group
    char token[TOKEN_LEN+1];            // full mnemonic token for lookup
    index_t index;                      // index of tokenized name, or -1 if no token (optional)
    valueType valuetype;                // type of value that follows: see valueType enum
    int8_t precision;                   // decimal precision for reporting floating point numbers (JSON only)
    double/*float*/ value_flt;              // floating point values
    int32_t value_int;                  // signed integer values and booleans
    char (*stringp)[];                  // string value: pointer to array of characters from shared character array
} nvObj_t;                              // OK, so it's not REALLY an object

typedef uint8_t (*fptrCmd)(nvObj_t *nv);// required for cfg table access
typedef void (*fptrPrint)(nvObj_t *nv); // required for program memory access

typedef struct nvList {
    uint16_t magic_start;
    nvObj_t list[NV_LIST_LEN+NV_EXEC_LEN]; // list of nv objects, including space for a JSON header element
    uint16_t magic_end;
} nvList_t;

typedef struct cfgItem {
    char group[GROUP_LEN+1];            // group prefix (with NUL termination)
    char token[TOKEN_LEN+1];            // token - stripped of group prefix (w/NUL termination)
    uint8_t flags;                      // operations flags - see defines below
    int8_t precision;                   // decimal precision for display (JSON)
    fptrPrint print;                    // print binding: aka void (*print)(nvObj_t *nv);
    fptrCmd get;                        // GET binding aka uint8_t (*get)(nvObj_t *nv)
    fptrCmd set;                        // SET binding aka uint8_t (*set)(nvObj_t *nv)
    void *target;                       // target for writing config value
    float def_value;                    // default value for config item
} cfgItem_t;

/**** static allocation and definitions ****/

extern nvStr_t nvStr;
extern nvList_t nvl;
extern const cfgItem_t cfgArray[];

//#define nv_header nv.list
#define nv_header (&nvl.list[0])
#define nv_body   (&nvl.list[1])
#define nv_exec   (&nvl.list[NV_EXEC_FIRST])

/**** Prototypes for generic config functions - see individual modules for application-specific functions  ****/

void config_init(void);
stat_t set_defaults(nvObj_t *nv);       // reset config to default values
void config_init_assertions(void);
stat_t config_test_assertions(void);

// main entry points for core access functions
stat_t nv_get(nvObj_t *nv);             // main entry point for get value
stat_t nv_set(nvObj_t *nv);             // main entry point for set value
void nv_print(nvObj_t *nv);             // main entry point for print value
stat_t nv_persist(nvObj_t *nv);         // main entry point for persistence

// helpers
uint8_t nv_get_type(nvObj_t *nv);
void nv_coerce_types(nvObj_t *nv);
index_t nv_get_index(const char *group, const char *token);
index_t nv_index_max(void);             // (see config_app.c)
bool nv_index_is_single(index_t index); // (see config_app.c)
bool nv_index_is_group(index_t index);  // (see config_app.c)
bool nv_index_lt_groups(index_t index); // (see config_app.c)
bool nv_group_is_prefixed(char *group);

// generic internal functions and accessors
stat_t get_nul(nvObj_t *nv);            // get null value type
stat_t get_int32(nvObj_t *nv);          // get int32_t integer value
stat_t get_flt(nvObj_t *nv);            // get floating point value
stat_t get_data(nvObj_t *nv);           // get uint32_t integer value blind cast

stat_t set_noop(nvObj_t *nv);           // set nothing and return OK
stat_t set_nul(nvObj_t *nv);            // set nothing and return READ_ONLY error
stat_t set_ro(nvObj_t *nv);             // set nothing, return read-only error
stat_t set_int32_(nvObj_t *nv);          // set int32_t integer value
stat_t set_flt(nvObj_t *nv);            // set floating point value
stat_t set_data(nvObj_t *nv);           // set uint32_t integer value blind cast

stat_t set_grp(nvObj_t *nv);            // set data for a group
stat_t get_grp(nvObj_t *nv);            // get data for a group

// nvObj and list functions
void nv_get_nvObj(nvObj_t *nv);
nvObj_t *nv_reset_nv(nvObj_t *nv);
nvObj_t *nv_reset_nv_list(void);
nvObj_t *nv_reset_exec_nv_list();
stat_t nv_copy_string(nvObj_t *nv, const char *src);
nvObj_t *nv_add_object(const char *token);
nvObj_t *nv_add_integer(const char *token, const uint32_t value);
nvObj_t *nv_add_float(const char *token, const float value);
nvObj_t *nv_add_string(const char *token, const char *string);
nvObj_t *nv_add_conditional_message(const char *string);
void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags);

// application specific helpers and functions (config_app.c)

void convert_incoming_float(nvObj_t *nv);           // pre-process outgoing float values for units and illegal values
void convert_outgoing_float(nvObj_t *nv);           // pre-process incoming float values for canonical units

stat_t get_float(nvObj_t *nv, const float value);   // boilerplate for retrieving raw floating point value
stat_t get_integer(nvObj_t *nv, const int32_t value);   // boilerplate for retrieving 8 bit integer value
stat_t set_float(nvObj_t *nv, float &value);        // boilerplate for setting a floating point value w/conversion
stat_t set_float_range(nvObj_t *nv, float &value, float low, float high);
stat_t set_int_u8(nvObj_t *nv, uint8_t &value, uint8_t low, uint8_t high);
stat_t set_int32(nvObj_t *nv);//sme: this prototype was missing in chabo, with x-ide compiler faulting on that.
stat_t set_int32(nvObj_t *nv, int32_t  &value, int32_t low, int32_t high);
stat_t set_uint32(nvObj_t *nv, uint32_t &value, int32_t low, int32_t high);
stat_t get_ui16(nvObj_t *nv);

//edge
stat_t get_string(nvObj_t *nv, const char *str);
stat_t din_get_en(nvObj_t *nv);
stat_t din_get_po(nvObj_t *nv);
stat_t din_get_ac(nvObj_t *nv);
stat_t din_get_in(nvObj_t *nv);
stat_t din_get_input(nvObj_t *nv);
void din_print_state(nvObj_t *nv);

//V3
stat_t set_ui8(nvObj_t *nv);
stat_t get_ui8(nvObj_t *nv);
stat_t set_ui16(nvObj_t *nv);
stat_t get_ui16(nvObj_t *nv);
bool config_setting_defaults(void);
stat_t din_set_po(nvObj_t *nv);
stat_t _set_int_tests(nvObj_t *nv, int32_t low, int32_t high);


// diagnostics
void nv_dump_nv(nvObj_t *nv);
#ifdef RECONCILE_TUID_NO_STRIP
void config_allow_tuid_group_strip(bool allow);
#endif
#ifdef DEPLOY_INSTALLED_TOOL_PERSISTENCE
stat_t set_uda_data(nvObj_t *nv);
#endif
/*********************************************************************************************
 **** PLEASE NOTICE THAT CONFIG_APP.H IS HERE ************************************************
 *********************************************************************************************/
#include "g2_config_app.h"
 
#endif // End of include guard: CONFIG_H_ONCE
