/*
 * config.cpp - application independent configuration handling
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
/*
 *  See config.h for a Config system overview and a bunch of details.
 */
#include "main.h"
#include "bantam_hal.h"
#include "g2core.h"  // #1
#include "g2_config.h"  // #2
#include "g2_report.h"
#include "g2_controller.h"
#include "g2_canonical_machine.h"
#include "g2_json_parser.h"
#include "g2_text_parser.h"
#include "g2_persistence.h"
#include "g2_help.h"
#include "g2_util.h"
#include "g2_sd_persistence.h"   
#include "comms_mgr.h" 

bool config_startup_flag=false;
static void _set_defa(nvObj_t *nv, bool print);
void sme_pre_init_for_configs_sake(void);
/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

nvStr_t nvStr;
nvList_t nvl;
#ifdef RECONCILE_TUID_NO_STRIP
bool tuid_group_strip_needed=false;
void config_allow_tuid_group_strip(bool allow){tuid_group_strip_needed=allow;}
#endif
/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/* Primary access points to functions bound to text mode / JSON functions
 * These gatekeeper functions check index ranges so others don't have to
 *
 * nv_set()     - Write a value or invoke a function - operates on single valued elements or groups
 * nv_get()     - Build a nvObj with the values from the target & return the value
 *                Populate nv body with single valued elements or groups (iterates)
 * nv_print()   - Output a formatted string for the value.
 * nv_persist() - persist value to non-volatile storage. Takes special cases into account
 */
stat_t nv_set(nvObj_t *nv)
{
    stat_t result;
    if (nv->index >= nv_index_max()) 
    {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
    result=((fptrCmd)cfgArray[nv->index].set)(nv);
    return (result);
}

stat_t nv_get(nvObj_t *nv)
{
    if (nv->index >= nv_index_max()) {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
    return (((fptrCmd)cfgArray[nv->index].get)(nv));
}

void nv_print(nvObj_t *nv)
{
    if (nv->index >= nv_index_max()) {
        return;
    }
    ((fptrCmd)cfgArray[nv->index].print)(nv);
}

stat_t nv_persist(nvObj_t *nv)
{
  stat_t status = STAT_OK;
    if (nv_index_lt_groups(nv->index) == false) 
    {
        return(STAT_INTERNAL_RANGE_ERROR);
    }
    if (GET_TABLE_BYTE(flags) & F_PERSIST) 
    {
  
        status =write_persistent_value(nv);
        return(status );
     
    }
    return (STAT_OK);
}

/************************************************************************************
 * config_init() - called once on hard reset
 *
 * Performs one of 2 actions:
 *  (1) if persistence is set up or out-of-rev load RAM and NVM with settings.h defaults
 *  (2) if persistence is set up and at current config version use NVM data for config
 *
 *  You can assume the cfg struct has been zeroed by a hard reset.
 *  Do not clear it as the version and build numbers have already been set by tg_init()
 *
 * NOTE: Config assertions are handled from the controller
 */
void config_init()
{
  

#define DEBUG_NDX_NBR 555
    static int debug_ndx_nbr = DEBUG_NDX_NBR;

    nvObj_t *nv = nv_reset_nv_list();
    config_init_assertions();
    js.json_mode = JSON_MODE;                    // initial value until persistence is read
    /*12-7-2020: Confirmed this pre_init is essential in ST architecture, to pre-init st_cfg micro-step 
      and related parameters prior to calling set_sef() which applies microsteps and steps_per_unit 
      as denominators, whether they are initialized to non zero values ahead of time or not. 
      div by zero and NaN math errors in STM micro architectures results in a cpu reset, 
      whereas the synthetos micro tolerates these without a cpu reset. 
    */
    sme_pre_init_for_configs_sake();
  
    cm_set_units_mode(MILLIMETERS);             // must do inits in millimeter mode
    nv->index = 0;                              // this will read the first record in NVM
    config_startup_flag=true;
    read_persistent_value(nv);

    if (fp_NE(nv->value_flt, CHABO_FIRMWARE_BUILD))     
    {   // case (1) NVM is not setup or not in revision
        _set_defa(nv, false);
    } 
    else  
    {
        for (nv->index=0; nv_index_is_single(nv->index); nv->index++) 
        {
         
            if (nv->index==debug_ndx_nbr)
            {
             __NOP();// __no_operation();
            }
            /*sme: Non-persistent params end end up initialized with their compile time default*/
            if (GET_TABLE_BYTE(flags) & F_INITIALIZE) 
            {
                strncpy(nv->token, cfgArray[nv->index].token, TOKEN_LEN); // read the token from the array
                /* sme: Note: the read operation has no impact on non-persistent parameters*/
                read_persistent_value(nv);                
                nv_set(nv);        
            }
        }       
        config_startup_flag=false;
        sr_init_status_report();                    // reset status reports
    }
    rpt_print_loading_configs_message();
}

/*
 * set_defaults() - reset persistence with default values for machine profile
 * _set_defa() - helper function and called directly from config_init()
 */

static void _set_defa(nvObj_t *nv, bool print)
{
   static volatile int debug_ndx_value = 400;
    stat_t  status=STAT_OK;
    static  uint32_t dbg_ndx;  
   
#if 1//8-16-2022  flags debugging
    static volatile char* token_ptr=0;
    static volatile uint8_t flags[10];
    static volatile char temp_str[80];
    flags[0]= TYPE_INTEGER|F_INITIALIZE;
    flags[1]= _ii;
    flags[2]= _iin;
    flags[3]= flags[0]&F_INITIALIZE;
     flags[4]= flags[0]&(TYPE_INTEGER|F_INITIALIZE);
#endif 
    cm_set_units_mode(MILLIMETERS);             // must do inits in MM mode
    for (nv->index=0; nv_index_is_single(nv->index); nv->index++) 
    {
         
#if 1//debug use
      dbg_ndx=nv->index;
#define DEBUG_NDX_VAL 310         
      if(dbg_ndx==debug_ndx_value)//DEBUG_NDX_VAL)
      {
       __NOP();// __no_operation();
      }
#endif   

     flags[0] =(volatile uint8_t)cfgArray[nv->index].flags;  

flags[0] =(volatile uint8_t)cfgArray[nv->index].flags;  
#if 1//debug F_INITIALIZE in flags not working below. 
 strcpy(temp_str,cfgArray[nv->index].token);
 token_ptr=(volatile char*) strstr(cfgArray[nv->index].token,"wdog");
 if (token_ptr!=0)
 {
    __NOP();
 }
 flags[1]= flags[0]&F_INITIALIZE;
        if (flags[1] != 0)
#else
        if (cfgArray[nv->index].flags & F_INITIALIZE) 
#endif
        {
            auto type = cfgArray[nv->index].flags & F_TYPE_MASK;
            if ((type == TYPE_INTEGER) || (type == TYPE_DATA)) 
            {
                nv->valuetype = TYPE_INTEGER;
                nv->value_int = (int)cfgArray[nv->index].def_value;
            } 
            else if (type == TYPE_BOOLEAN) 
            {
                nv->valuetype = TYPE_BOOLEAN;
                nv->value_int = (int)cfgArray[nv->index].def_value;
            } 
            else if (type == TYPE_FLOAT) 
            {
                nv->valuetype = TYPE_FLOAT;
                nv->value_flt = cfgArray[nv->index].def_value;
            }
            else if (type == TYPE_STRING) //SME: ADDED 11-3-2021
            {
                nv->valuetype = TYPE_STRING;
                nv->stringp =  0;//cfgArray[nv->index].def_value;              
            }
            strncpy(nv->token, cfgArray[nv->index].token, TOKEN_LEN);
            status=cfgArray[nv->index].set(nv);        // run the set method, nv_set(nv);
            if (status ==STAT_OK)
            {
               if (nv->index == 0 || cfgArray[nv->index].flags & F_PERSIST) 
               {
                    nv_persist(nv);
               }
            }
            else
            {
              __NOP();//__no_operation();
            }
        }
    }/* End For */
  
    sr_init_status_report();                    // reset status reports
    if (print) 
    {
        rpt_print_initializing_message();       // don't start TX until all the NVM persistence is done
    }
}

stat_t set_defaults(nvObj_t *nv)
{
    // failsafe. nv->value_int must be true or no action occurs
    if (!nv->value_int) 
    {
        return(help_defa(nv));
    }
    _set_defa(nv, true);

    // The nvlist was used for the initialize message so the values are all garbage
    // Mark the nv as $defa so it displays nicely in the response
    nv_reset_nv_list();
    strncpy(nv->token, "defa", TOKEN_LEN);
//  nv->index = nv_get_index("", nv->token);    // correct, but not required
    nv->valuetype = TYPE_INTEGER;               // ++++ probably should be TYPE_BOOLEAN
    nv->value_int = true;
    return (STAT_OK);
}

/*
 * config_init_assertions()
 * config_test_assertions() - check memory integrity of config sub-system
 */

void config_init_assertions()
{
    cfg.magic_start = MAGICNUM;
    cfg.magic_end = MAGICNUM;
    nvl.magic_start = MAGICNUM;
    nvl.magic_end = MAGICNUM;
    nvStr.magic_start = MAGICNUM;
    nvStr.magic_end = MAGICNUM;
}

stat_t config_test_assertions()
{
    if ((BAD_MAGIC(cfg.magic_start)) ||
        (BAD_MAGIC(cfg.magic_end)) ||
        (BAD_MAGIC(nvl.magic_start)) ||
        (BAD_MAGIC(nvl.magic_end)) ||
        (BAD_MAGIC(nvStr.magic_start)) ||
        (BAD_MAGIC(nvStr.magic_end))) {
        return(cm_panic(STAT_CONFIG_ASSERTION_FAILURE, "config_test_assertions()"));
    }
    return (STAT_OK);
}

/***** Generic Internal Functions *********************************************/
/*
 *  Get input state given an nv object
 *  Note: if this is not forwarded to an input or the input is disabled,
 *  it returns NULL.
 */
stat_t din_get_input(nvObj_t *nv)
{
#if 1 //to do
   return 0;
#else
    return _ir(nv)->getState(nv);
#endif
}
/* Generic gets()
 *  get_nul()  - get nothing (returns STAT_NOOP)
 *  get_int32()  - get value as 32 bit integer
 *  get_data() - get value as 32 bit integer blind cast
 *  get_flt()  - get value as float
 */
stat_t get_nul(nvObj_t *nv)
{
    nv->valuetype = TYPE_NULL;
    return (STAT_NOOP);
}
static const char fmt_gpio_state[] = "Input %s state: %5ld\n";
void din_print_state(nvObj_t *nv) 
{
     sprintf(cs.out_buf, fmt_gpio_state, nv->token, nv->value_int);
      comms_mgr_write_msg(cs.out_buf) ;  
}
 
/**************************************************************************************
 *  get_ui16
 *
 *  
 **************************************************************************************/ 
stat_t get_ui16(nvObj_t *nv)
{
  nv->value_int = *((uint16_t *)GET_TABLE_WORD(target));
  nv->valuetype = TYPE_INTEGER;
  return (STAT_OK);
}/* End Function */
 
stat_t get_int32(nvObj_t *nv)
{
    nv->value_int = *((int32_t *)GET_TABLE_WORD(target));
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}
#ifdef DEPLOY_GET_CAUSE_OF_RESET 
stat_t get_uint32(nvObj_t *nv)
{
    nv->value_int = *((uint32_t *)GET_TABLE_WORD(target));
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}
#endif
stat_t get_flt(nvObj_t *nv)
{
    nv->value_flt= *((float *)GET_TABLE_WORD(target));
    nv->precision = (int8_t)GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return (STAT_OK);
}

stat_t get_data(nvObj_t *nv)
{
    nv->value_int = *((uint32_t *)GET_TABLE_WORD(target));
    nv->valuetype = TYPE_DATA;
    return (STAT_OK);
}

/* Generic sets()
 *  set_noop()  - set nothing and return OK
 *  set_nul()   - set nothing and return READ_ONLY error
 *  set_ro()    - set nothing, return read-only error
 *  set_int32() - set value as 32 bit unsigned integer
 *  set_flt()   - set value as float
 *  set_data()  - set value as 32 bit integer blind cast
 */

stat_t set_noop(nvObj_t *nv) {
    // we call this for msg to parrot back the string
    // check for string type and pass-through
    if (cfgArray[nv->index].flags & TYPE_STRING) 
    {
        nv->valuetype = TYPE_STRING;
    } 
    else 
    {
        nv->valuetype = TYPE_NULL;
    }
    return (STAT_OK);  // hack until JSON is refactored
}

stat_t set_nul(nvObj_t *nv) {
    nv->valuetype = TYPE_NULL;
    return (STAT_PARAMETER_IS_READ_ONLY);   // this is what it should be
}

stat_t set_ro(nvObj_t *nv) {
    if (strcmp(nv_body->token, "sr") == 0) { // hack. If setting an SR it doesn't fail
        return (STAT_OK);
    }
    nv->valuetype = TYPE_NULL;
    return (STAT_PARAMETER_IS_READ_ONLY);
}

stat_t set_int32(nvObj_t *nv)
{
     *((int32_t *)GET_TABLE_WORD(target)) = nv->value_int;
    nv->valuetype = TYPE_INTEGER;
    return(STAT_OK);
}

stat_t set_flt(nvObj_t *nv)
{
    *((float *)GET_TABLE_WORD(target)) = nv->value_flt;
    nv->precision = GET_TABLE_WORD(precision);
    nv->valuetype = TYPE_FLOAT;
    return(STAT_OK);
}

stat_t set_data(nvObj_t *nv)
{
    uint32_t *v = (uint32_t*)&nv->value_int;
    *((uint32_t *)GET_TABLE_WORD(target)) = *v;
    nv->valuetype = TYPE_DATA;
    return(STAT_OK);
}
#ifdef DEPLOY_INSTALLED_TOOL_PERSISTENCE
/****************************************************************************
 *
 * Function: set_uda_data
 *
 * Description: User Data A is reserved for storing the Tool UID 
 *    in four u32 components that are alternately represented
 *    as a string of 32 hex nibbles. 
 *    Each time the tool ID is updated from the desktop, 
 *    this wrapper function tracks the four consecutive
 *    calls required to update all four u32 UID components.
 *    Once all four are updated, the function: atc_uda_updated_from_config_app() 
 *    which sets a flag is set that informs the 
 *    motion_atc_move_cycle that a manual tool change event has
 *    occured, and the UID identity is ready to be decoded against
 *    the tool uids that are mapped to the carriage slots 1-8.
 *
 *     This function is also called when an ATC cycle has been performed
 *     either manually throught the debug console or from a gcode program,
 *     In such cases, the  selected tool slot id and it's related tuid id 
 *     string is already known, and the decoding described for manual tool
 *     changes is bypassed.
 *
 ****************************************************************************/
stat_t set_uda_data(nvObj_t *nv)
{
  static int update_counter=0;
#define UDA_UPDATE_COMPLETED_COUNT 4
  set_data(nv);
  return STAT_OK;
}
#endif

/************************************************************************************
 * Group operations
 *
 *  Group operations work on parent/child groups where the parent is one of:
 *    axis group             x,y,z,a,b,c
 *    motor group            1,2,3,4
 *    PWM group                p1
 *    coordinate group        g54,g55,g56,g57,g58,g59,g92
 *    system group            "sys" - a collection of otherwise unrelated variables
 *
 *  Text mode can only GET groups. For example:
 *    $x                    get all members of an axis group
 *    $1                    get all members of a motor group
 *    $<grp>                get any named group from the above lists
 *
 *  In JSON groups are carried as parent / child objects & can get and set elements:
 *    {"x":""}                      get all X axis parameters
 *    {"x":{"vm":""}}               get X axis velocity max
 *    {"x":{"vm":1000}}             set X axis velocity max
 *    {"x":{"vm":"","fr":""}}       get X axis velocity max and feed rate
 *    {"x":{"vm":1000,"fr";900}}    set X axis velocity max and feed rate
 *    {"x":{"am":1,"fr":800,....}}  set multiple or all X axis parameters
 */

/*
 * get_grp() - read data from axis, motor, system or other group
 *
 *  get_grp() is a group expansion function that expands the parent group and returns
 *  the values of all the children in that group. It expects the first nvObj in the
 *  nvBody to have a valid group name in the token field. This first object will be set
 *  to a TYPE_PARENT. The group field of the first nvOBJ is left nul - as the group
 *  field refers to a parent group, which this group has none.
 *
 *  All subsequent nvObjs in the body will be populated with their values.
 *  The token field will be populated as will the parent name in the group field.
 *
 *  The sys group is an exception where the children carry a blank group field, even though
 *  the sys parent is labeled as a TYPE_PARENT.
 */

stat_t get_grp(nvObj_t *nv)
{
    char group[GROUP_LEN+1];

    strcpy(group, nv->token);                       // save the group string
    nv_reset_nv_list();                             // start with a clean list
    strcpy(nv->token, group);                       // re-write the group string
    nv->valuetype = TYPE_PARENT;                    // make first object the parent
    for (index_t i=0; nv_index_is_single(i); i++) {
        if (strcmp(group, cfgArray[i].group) != 0) { continue; }
        (++nv)->index = i;
        nv_get_nvObj(nv);
    }
    return (STAT_OK);
}

/*
 * set_grp() - get or set one or more values in a group
 *
 *  This functions is called "_set_group()" but technically it's a getter and
 *  a setter. It iterates the group children and either gets the value or sets
 *  the value for each depending on the nv->valuetype.
 *
 *  This function serves JSON mode only as text mode shouldn't call it.
 */

stat_t set_grp(nvObj_t *nv)
{
    if (js.json_mode == TEXT_MODE) 
	{
        return (STAT_UNRECOGNIZED_NAME);
    }
    for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) 
	{
        if ((nv = nv->nx) == NULL) 
			break;
        if (nv->valuetype == TYPE_EMPTY) 
			break;
        else if (nv->valuetype == TYPE_NULL)        // NULL means GET the value
            nv_get(nv);
        else 
		{
            nv_set(nv);
            nv_persist(nv);
        }
    }
    return (STAT_OK);
}

/***********************************************************************************
 ***** nvObj functions ************************************************************
 ***********************************************************************************/

/***********************************************************************************
 * nvObj helper functions and other low-level nv helpers
 */

/* nv_get_index() - get index from mnenonic token + group
 *
 * nv_get_index() is the most expensive routine in the whole config. It does a
 * linear table scan of the strings, which of course could be further
 * optimized with indexes or hashing.
 */
index_t nv_get_index(const char *group, const char *token)
{
    char str[TOKEN_LEN + GROUP_LEN+1];    // should actually never be more than TOKEN_LEN+1
    index_t token_len=strlen(str);//sme 
    volatile index_t token_ptr_len;//sme    
    char *token_ptr=0;            //sme 

    strncpy(str, group, GROUP_LEN+1);
    strncat(str, token, TOKEN_LEN+1);

    index_t i;     
    index_t index_max = nv_index_max();
    //sme: this is refactored, simpler version tan the edge/legacy, which also had compile time literals for max assumed token length.
    for (i=0; i < index_max; i++) 
    {  
        token_ptr =  (char *)&cfgArray[i].token[0];
        token_ptr_len=strlen(token_ptr);
        if (strcmp(token_ptr, (char *)str)== 0)              
        {
           return i;
        }      
    }
    return (NO_MATCH);
}

/*
 * nv_get_type() - returns command type as a NV_TYPE enum
 *
 *  Note: Exception reports (er) do not go through this mechanism so they are
 *        not in the list below
 */
uint8_t nv_get_type(nvObj_t *nv)
{
    if (nv->token[0] == NUL) return (NV_TYPE_NULL);
    if (strcmp("gc", nv->token) == 0) { return (NV_TYPE_GCODE); }
    if (strcmp("n",  nv->token) == 0) { return (NV_TYPE_LINENUM); }
    if (strcmp("sr", nv->token) == 0) { return (NV_TYPE_REPORT); }
    if (strcmp("qr", nv->token) == 0) { return (NV_TYPE_REPORT); }
    if (strcmp("msg",nv->token) == 0) { return (NV_TYPE_MESSAGE); }
    if (strcmp("err",nv->token) == 0) { return (NV_TYPE_MESSAGE); } // errors are reported as messages
    return (NV_TYPE_CONFIG);
}

/*
 * nv_coerce_types() - change types based on type field in configApp table
 */

void nv_coerce_types(nvObj_t *nv)
{
    if (nv->valuetype == TYPE_NULL) {  // don't change type if it's a GET query
        return;
    }
    valueType type = (valueType)(cfgArray[nv->index].flags & F_TYPE_MASK);
    if (type == TYPE_INTEGER) 
	{
        if (nv->valuetype == TYPE_FLOAT) 
        {
            nv->value_int = std::floor(nv->value_flt+0.5f);
        }
        nv->valuetype = TYPE_INTEGER;   // will pay attention to the int value, not the float
    } 
	else if (type == TYPE_BOOLEAN) 
	{  // it may have been marked as a boolean, but if it's not...
        if (nv->valuetype == TYPE_INTEGER) 
		{
            nv->value_int = nv->value_int ? true : false;
        } 
        else if (nv->valuetype == TYPE_FLOAT) 
	    {
            nv->value_int = (fp_ZERO(nv->value_flt)) ? true : false;
        }
        nv->valuetype = TYPE_BOOLEAN;
    }
}

/******************************************************************************
 * nvObj low-level object and list operations
 * nv_get_nvObj()       - setup a nv object by providing the index
 * nv_reset_nv()        - quick clear for a new nv object
 * nv_reset_nv_list()   - clear entire header, body and footer for a new use
 * nv_copy_string()     - used to write a string to shared string storage and link it
 * nv_add_object()      - write contents of parameter to  first free object in the body
 * nv_add_integer()     - add an integer value to end of nv body (Note 1)
 * nv_add_float()       - add a floating point value to end of nv body
 * nv_add_string()      - add a string object to end of nv body
 * nv_add_conditional_message() - add a message to nv body if messages are enabled
 *
 *  Note: Functions that return a nv pointer point to the object that was modified or
 *  a NULL pointer if there was an error.
 *
 *  Note: Adding a really large integer (like a checksum value) may lose precision due
 *  to the cast to a float. Sometimes it's better to load an integer as a string if
 *  all you want to do is display it.
 *
 *  Note: A trick is to cast all string constants for nv_copy_string(), nv_add_object(),
 *  nv_add_string() and nv_add_conditional_message() to (const char *). Examples:
 *
 *    nv_add_string((const char *)"msg", string);
 */

void nv_get_nvObj(nvObj_t *nv)
{
    if (nv->index >= nv_index_max()) { return; }    // sanity

    index_t tmp = nv->index;
    nv_reset_nv(nv);
    nv->index = tmp;

    strcpy(nv->token, cfgArray[nv->index].token); // token field is always terminated
    strcpy(nv->group, cfgArray[nv->index].group); // group field is always terminated

    // special processing for system groups and stripping tokens for groups
    if (nv->group[0] != NUL) 
    {
        if ((cfgArray[nv->index].flags & F_NOSTRIP) &&(tuid_group_strip_needed!=true))
        {
            nv->group[0] = NUL;
        } 
        else 
        {
            strcpy(nv->token, &nv->token[strlen(nv->group)]); // strip group from the token
        }
    }
    ((fptrCmd)cfgArray[nv->index].get)(nv);     // populate the value
}

nvObj_t *nv_reset_nv(nvObj_t *nv)               // clear a single nvObj structure
{
    nv->valuetype = TYPE_EMPTY;                 // selective clear is much faster than calling memset
    nv->index = 0;
    nv->value_int = 0;
    nv->value_flt = 0;
    nv->precision = 0;
    nv->token[0] = NUL;
    nv->group[0] = NUL;
    nv->stringp = NULL;

    if (nv->pv == NULL) {                       // set depth correctly
        nv->depth = 0;
    } else {
        if (nv->pv->valuetype == TYPE_PARENT) {
            nv->depth = nv->pv->depth + 1;
        } else {
            nv->depth = nv->pv->depth;
        }
    }
    return (nv);                                // return pointer to nv as a convenience to callers
}

void _nv_reset_a_list(nvObj_t *nv, uint8_t length) // clear some nv list (called from below)
{
    for (uint8_t i=0; i<length; i++, nv++) {
        nv->pv = (nv-1);                        // the ends are bogus & corrected later
        nv->nx = (nv+1);
        nv->index = 0;
        nv->depth = 1;                          // header and footer are corrected later
        nv->precision = 0;
        nv->valuetype = TYPE_EMPTY;
        nv->token[0] = NUL;
    }
    (--nv)->nx = NULL;
}

nvObj_t *nv_reset_nv_list()                     // clear the header and response body
{
    nvStr.wp = 0;                               // reset the shared string
    nvObj_t *nv = nvl.list;                     // set up linked list and initialize elements

    _nv_reset_a_list(nv, NV_LIST_LEN);

    nv = nvl.list;                              // setup response header element ('r')
    nv->pv = NULL;
    nv->depth = 0;
    nv->valuetype = TYPE_PARENT;
    strcpy(nv->token, "r");

    return (nv_body);                           // this is a convenience for calling routines
}


nvObj_t *nv_reset_exec_nv_list()                // clear the exec body
{
    nvObj_t *nv = nv_exec;

    _nv_reset_a_list(nv, NV_EXEC_LEN);

    return (nv_exec);                           // this is a convenience for calling routines
}

stat_t nv_copy_string(nvObj_t *nv, const char *src)
{
    int src_len= strlen(src);//sme: 4-9-2021 debug reset, also on 5-25-2021 related to added cycle type: CYCLE_ATC_SEEK, ndx=5
    int combined_len = nvStr.wp + src_len;//sme: 4-9-2021 debug reset
    if ( combined_len > NV_SHARED_STRING_LEN) 
    {
        return (STAT_BUFFER_FULL);
    }
    char *dst = &nvStr.string[nvStr.wp];
    strcpy(dst, src);                           // copy string to current head position
                                                // string has already been tested for overflow, above
    nvStr.wp += strlen(src)+1;                  // advance head for next string
    nv->stringp = (char (*)[])dst;
    return (STAT_OK);
}

nvObj_t *nv_add_object(const char *token)       // add an object to the body using a token
{
    nvObj_t *nv = nv_body;
    for (uint8_t i=0; i<NV_BODY_LEN; i++) {
        if (nv->valuetype != TYPE_EMPTY) {
            if ((nv = nv->nx) == NULL) {        // not supposed to find a NULL; here for safety
                return(NULL);
            }
            continue;
        }
        // load the index from the token or die trying
        if ((nv->index = nv_get_index((const char *)"",token)) == NO_MATCH) { return (NULL);}
        nv_get_nvObj(nv);                       // populate the object from the index
        return (nv);
    }
    return (NULL);
}

nvObj_t *nv_add_integer(const char *token, const int32_t value) // add an integer object to the body
{
    nvObj_t *nv = nv_body;
    for (uint8_t i=0; i<NV_BODY_LEN; i++) {
        if (nv->valuetype != TYPE_EMPTY) {
            if ((nv = nv->nx) == NULL) {        // not supposed to find a NULL; here for safety
                return(NULL);
            }
            continue;
        }
        strncpy(nv->token, token, TOKEN_LEN);
        nv->value_int = value;
        nv->valuetype = TYPE_INTEGER;
        return (nv);
    }
    return (NULL);
}

nvObj_t *nv_add_data(const char *token, const uint32_t value)// add an data object to the body
{
    nvObj_t *nv = nv_body;
    for (uint8_t i=0; i<NV_BODY_LEN; i++) {
        if (nv->valuetype != TYPE_EMPTY) {
            if ((nv = nv->nx) == NULL) {        // not supposed to find a NULL; here for safety
                 return(NULL);
            }
            continue;
        }
        strcpy(nv->token, token);
        nv->value_int = value;
        nv->valuetype = TYPE_DATA;
        return (nv);
    }
    return (NULL);
}

nvObj_t *nv_add_float(const char *token, const float value)    // add a float object to the body
{
    nvObj_t *nv = nv_body;
    for (uint8_t i=0; i<NV_BODY_LEN; i++) {
        if (nv->valuetype != TYPE_EMPTY) {
            if ((nv = nv->nx) == NULL) {        // not supposed to find a NULL; here for safety
                return(NULL);
            }
            continue;
        }
        strncpy(nv->token, token, TOKEN_LEN);
        nv->value_flt = value;
        nv->valuetype = TYPE_FLOAT;
        return (nv);
    }
    return (NULL);
}

nvObj_t *nv_add_string(const char *token, const char *string) // add a string object to the body
{
    nvObj_t *nv = nv_body;
    for (uint8_t i=0; i<NV_BODY_LEN; i++) {
        if (nv->valuetype != TYPE_EMPTY) {
            if ((nv = nv->nx) == NULL) {        // not supposed to find a NULL; here for safety
                return(NULL);
            }
            continue;
        }
        strncpy(nv->token, token, TOKEN_LEN);
        if (nv_copy_string(nv, string) != STAT_OK) {
            return (NULL);
        }
        nv->index = nv_get_index((const char *)"", nv->token);
        nv->valuetype = TYPE_STRING;
        return (nv);
    }
    return (NULL);
}

/*
 * nv_add__conditional_message() - queue a RAM string as a message in the response (conditionally)
 */

nvObj_t *nv_add_conditional_message(const char *string)    // conditionally add a message object to the body
{
    if ((js.json_mode == JSON_MODE) 
	&& (js.echo_json_messages != true)) 
	{ 
		return (NULL); 
	}
    return(nv_add_string((const char *)"msg", string));
}

/**** nv_print_list() - print nv_array as JSON or text **********************
 *
 *   Generate and print the JSON and text mode output strings. Use this function
 *  for all text and JSON output that wants to be in a response header.
 *  Don't just printf stuff.
 *
 *  Inputs:
 *    json_flags = JSON_OBJECT_FORMAT - print just the body w/o header or footer
 *    json_flags = JSON_RESPONSE_FORMAT - print a full "r" object with footer
 *    json_flags = JSON_RESPONSE_TO_MUTED_FORMAT - JSON_RESPONSE_FORMAT, but only to muted channels
 *
 *    text_flags = TEXT_INLINE_PAIRS - print text as name/value pairs on a single line
 *    text_flags = TEXT_INLINE_VALUES - print text as comma separated values on a single line
 *    text_flags = TEXT_MULTILINE_FORMATTED - print text one value per line with formatting string
 */

void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags)
{
    if (js.json_mode == JSON_MODE) 
    {
        json_print_list(status, json_flags);
    } 
    else 
    {
        text_print_list(status, text_flags);
    }
}

/****************************************************************************
 ***** Diagnostics **********************************************************
 ****************************************************************************/
#if 0//sme is this used?No
void nv_dump_nv(nvObj_t *nv)
{
    sprintf (cs.out_buf, "i:%ld, d:%d, t:%d, p:%d, v:%f, g:%s, t:%s, s:%s\n",
            nv->index,
            nv->depth,
            nv->valuetype,
            nv->precision,
            (double)nv->value_flt,      // would need to add in value_int to be complete
            nv->group,
            nv->token,
            (char *)nv->stringp);
    
      comms_mgr_write_msg(cs.out_buf) ;   
}
#endif
void print_hardfault_data(char *msg)
{
  //sprintf(cs.out_buf,"Hard Fault Info: HFSR=%x08x,CFSR=%x08x\n",hfsr ,cfsr);
  comms_mgr_write_msg(msg) ;   
}
/**************************************************************************************
 *  set_ui8
 * assumes nv->index has a valid, in range value. If not, cpu reset is likely
 *  
 **************************************************************************************/ 
stat_t set_ui8(nvObj_t *nv)
{

  //edge changes to value_int instead of value which is float
  volatile  uint8_t tmp_val =(uint8_t) nv->value_int;
 
  *((uint8_t *)cfgArray[nv->index].target)= tmp_val;
       
	nv->valuetype = TYPE_INTEGER;
	return(STAT_OK);
}/* End Function */
/**************************************************************************************
 *  get_ui8
 *
 *  
 **************************************************************************************/ 
stat_t get_ui8(nvObj_t *nv)
{
  nv->value_int = *((uint8_t *) cfgArray[nv->index].target); 
  nv->value =(float)nv->value_int; 
  nv->valuetype = TYPE_INTEGER;
  return (STAT_OK);
}/* End Function */
/**************************************************************************************
 *  set_ui16
 *
 *  
 **************************************************************************************/ 
stat_t set_ui16(nvObj_t *nv)
{
  volatile  uint16_t tmp_val =(uint16_t) nv->value_int; 
  *((uint16_t *)cfgArray[nv->index].target)= tmp_val;        
  nv->valuetype = TYPE_INTEGER;
  return(STAT_OK);
}/* End Function */


bool config_setting_defaults(void){return config_startup_flag== true;}

