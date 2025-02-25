
/*
 * json_parser.cpp - JSON parser
 * This file is part of the g2core project
 *
 * Copyright (c) 2011 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Rob Giseburt
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
#include "main.h"
#include "bantam_hal.h"
#include "g2core.h"
#include "g2_config.h"  // JSON sits on top of the config system
#include "g2_controller.h"
#include "g2_json_parser.h"
#include "g2_text_parser.h"
#include "g2_canonical_machine.h"
#include "g2_report.h"
#include "g2_util.h"
#include "comms_mgr.h"  

/**** Allocation ****/

jsSingleton_t js;

/**** local scope stuff ****/
//sr_write_msg
static stat_t _json_parser_kernal(nvObj_t *nv, char *str);
static stat_t _json_parser_execute(nvObj_t *nv);
static stat_t _normalize_json_string(char *str, uint16_t size);
static stat_t _get_nv_pair(nvObj_t *nv, char **pstr, int8_t *depth);

/****************************************************************************
 * json_parser() - exposed part of JSON parser
 * _json_parser_kernal()
 * _normalize_json_string()
 * _get_nv_pair_strict()
 *
 *  This is a dumbed down JSON parser to fit in limited memory with no malloc
 *  or practical way to do recursion ("depth" tracks parent/child levels).
 *
 *  This function will parse the following forms up to the JSON_MAX limits:
 *    {"name":"value"}
 *    {"name":12345}
 *    {"name1":"value1", "n2":"v2", ... "nN":"vN"}
 *    {"parent_name":""}
 *    {"parent_name":{"name":"value"}}
 *    {"parent_name":{"name1":"value1", "n2":"v2", ... "nN":"vN"}}
 *
 *    "value" can be a string, number, true, false, or null (2 types)
 *
 *  Numbers
 *    - number values are not quoted and can start with a digit or -.
 *    - numbers cannot start with + or . (period)
 *    - exponentiated numbers are handled OK.
 *    - hexadecimal or other non-decimal number bases are not supported
 *
 *  The parser:
 *    - extracts an array of one or more JSON object structs from the input string
 *    - once the array is built it executes the object(s) in order in the array
 *    - passes the executed array to the response handler to generate the response string
 *    - returns the status and the JSON response string
 *
 *  Separation of concerns
 *    json_parser() is the only exposed part. It does parsing, display, and status reports.
 *    _get_nv_pair() only does parsing and syntax; no semantic validation or group handling
 *    _json_parser_kernal() does index validation and group handling
 *    _json_parser_execute() executes sets and gets in an application agnostic way. It should work for other apps than g2core
 */

stat_t json_parser(char *str, bool suppress_response) // suppress_response defaults to false, see decalaration in .h
{
    nvObj_t *nv = nv_reset_nv_list();               // get a fresh nvObj list
    volatile char str_copy[256];
    strcpy(str_copy,str);
    volatile char *str_ptr=str_copy;
    stat_t status = _json_parser_kernal(nv, str);
#ifdef RECONCILE_TUID_NO_STRIP
  char *tuid_str= strstr(str,"tuid");
   if(strncmp(tuid_str,(char*)"tuid",4)==0)  
   {
     config_allow_tuid_group_strip(true);
   }
#endif
    if (status == STAT_OK) 
    {                        // execute the command
        nv = nv_body;
        status = _json_parser_execute(nv);
    }
#ifdef RECONCILE_TUID_NO_STRIP                                        
    config_allow_tuid_group_strip(false);
#endif
    if (suppress_response || (status == STAT_COMPLETE)) 
    {  // skip the print if returning from something that already did it.
        return status;
    }
                                        
    nv_print_list(status, TEXT_MULTILINE_FORMATTED, JSON_RESPONSE_FORMAT);
    
    sr_request_status_report(SR_REQUEST_TIMED);     // generate incremental status report to show any changes
    return STAT_OK;
}

// This is almost the same as json_parser, except it doesn't *always* execute the parsed out list, and it never returns a reponse
void json_parse_for_exec(char *str, bool execute)
{
    nvObj_t *nv = nv_reset_exec_nv_list();          // get a fresh nvObj list
    stat_t status = _json_parser_kernal(nv, str);
    if ((status == STAT_OK) && (execute)) 
    {
        nv = nv_exec;
        status = _json_parser_execute(nv);          // execute the command
    }
    sr_request_status_report(SR_REQUEST_TIMED);     // generate incremental status report to show any changes
}

static stat_t _json_parser_execute(nvObj_t *nv) 
{
    do {
        if (nv->valuetype == TYPE_PARENT) 
        {   // added as partial fix for Issue #298  // Reading values with nested JSON changes values in inches mode
            if (strcmp(nv->token, "sr") == 0) 
            {     // Hack to execute Set Status Report (SR parent) See end note (*)
                return (nv_set(nv));
            }
        } 
        else if (nv->valuetype == TYPE_NULL) 
        {    // means run the GET function to get the value
            ritorno(nv_get(nv));                    // ritorno returns w/status on any errors
            if (nv->valuetype == TYPE_PARENT) 
            {     // This will be true if you read a group. Exit now
                return (STAT_OK);
            }
        } 
        else 
        {                                   // otherwise, run the SET function
            cm_parse_clear(*nv->stringp);   // parse Gcode and clear alarms if M30 or M2 is found
            ritorno(cm_is_alarmed());       // return error status if in alarm, shutdown or panic
            ritorno(nv_set(nv));            // run the SET function  to set value or execute something (e.g. gcode)
            nv_persist(nv);
        }
        if ((nv = nv->nx) == NULL) 
        {
            return (STAT_JSON_TOO_MANY_PAIRS);      // Not supposed to encounter a NULL
        }
    } while (nv->valuetype != TYPE_EMPTY);

    return (STAT_OK);                               // only successful commands exit through this point
}

// (*) Note: The JSON / token system is essentially flat, as it was derived from a command-line flat-ASCII approach
//     If the JSON objects had proper recursive descent handlers that just passed the remaining string (at that level)
//     off for further processing, we would not need to do this hack. A fix is in the works. For now, this is OK.

static stat_t _json_parser_kernal(nvObj_t *nv, char *str)
{
    stat_t status;
    int8_t depth;
    volatile int static bkpt_count=0;
    char group[GROUP_LEN+1] = {""};                 // group identifier - starts as NUL
    int16_t i = NV_BODY_LEN;
 
    status = _normalize_json_string(str, JSON_INPUT_STRING_MAX);
    if (status != STAT_OK) 
    {
        nv->valuetype = TYPE_NULL;
        return (status);
    }
   
    // parse the JSON command into the nv body
    do {
        if (--i == 0) 
        {
            return (STAT_JSON_TOO_MANY_PAIRS);      // length error
        }
        // Use relaxed parser. Will read either strict or relaxed mode. To use strict-only parser refer
        // to build earlier than 407.03. Substitute _get_nv_pair_strict() for _get_nv_pair()
        if ((status = _get_nv_pair(nv, &str, &depth)) > STAT_EAGAIN) 
        { // erred out
            nv->valuetype = TYPE_NULL;
            return (status);
        }
         
        // propagate the group from previous NV pair (if relevant)
        if (group[0] != NUL) 
        {
            strncpy(nv->group, group, GROUP_LEN);   // copy the parent's group to this child
        }
       
        // validate the token and get the index
        if ((nv->index = nv_get_index(nv->group, nv->token)) == NO_MATCH) 
        {
            nv->valuetype = TYPE_NULL;
            return (STAT_UNRECOGNIZED_NAME);
        }
        if ((nv_index_is_group(nv->index)) && (nv_group_is_prefixed(nv->token))) 
        {
            strncpy(group, nv->token, GROUP_LEN);   // record the group ID			
        }
       //12-21-2020: sme: text parser applies this, so we do it here, else the trinamics items won't have their group identified        
        strcpy(nv->group, cfgArray[nv->index].group); // capture the group string if there is one
        nv_coerce_types(nv);                        // adjust types based on type fields in configApp table
        
        if ((nv = nv->nx) == NULL) 
        {
            return (STAT_JSON_TOO_MANY_PAIRS);      // Not supposed to encounter a NULL
        }
    } while (status != STAT_OK);                    // breaks when parsing is complete

    return (STAT_OK);                  // only successful commands exit through this point
}

/*
 * _normalize_json_string - normalize a JSON string in place
 *
 *  Validate string size limits, remove all whitespace and convert
 *  to lower case, with the exception of gcode comments
 */

static stat_t _normalize_json_string(char *str, uint16_t max_size)
{
    char *wr;                                       // write pointer
    uint8_t in_comment = false;
    int len=strlen(str);
    if ( len> max_size) 
    { 
        return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
    }
    for (wr = str; *str != NUL; str++) {
        if (!in_comment) {                          // normal processing
            if (*str == '(') in_comment = true;
            if ((*str <= ' ') || (*str == DEL)) continue; // toss ctrls, WS & DEL
            *wr++ = tolower(*str);
        } else {                                    // Gcode comment processing
            if (*str == ')') in_comment = false;
            *wr++ = *str;
        }
    }
    *wr = NUL;
    return (STAT_OK);
}

/*
 * _get_nv_pair() - get the next name-value pair w/relaxed JSON rules. Also parses strict JSON.
 *
 *  Parse the next statement and populate the command object (nvObj).
 *
 *  Leaves string pointer (str) on the first character following the object.
 *  Which is the character just past the ',' separator if it's a multi-valued
 *  object or the terminating NUL if single object or the last in a multi.
 *
 *  Keeps track of tree depth and closing braces as much as it has to.
 *  If this were to be extended to track multiple parents or more than two
 *  levels deep it would have to track closing curlies - which it does not.
 *
 *  ASSUMES INPUT STRING HAS FIRST BEEN NORMALIZED BY _normalize_json_string()
 *
 *  If a group prefix is passed in it will be pre-pended to any name parsed
 *  to form a token string. For example, if "x" is provided as a group and
 *  "fr" is found in the name string the parser will search for "xfr" in the
 *  cfgArray.
 */
/*  RELAXED RULES
 *
 *  Quotes are accepted but not needed on names
 *  Quotes are required for string values
 *
 *  See build 406.xx or earlier for strict JSON parser - deleted in 407.03
 */

static stat_t _get_nv_pair(nvObj_t *nv, char **pstr, int8_t *depth)
{
    /*volatile static*/ uint8_t i, len;
    /*volatile static*/ char *strptr=0;
    char *tmp;
    char leaders[] = {"{,\""};      // open curly, quote and leading comma
    char separators[] = {":\""};    // colon and quote
    char terminators[] = {"},\""};  // close curly, comma and quote
    char value[] = {"{\".-+"};      // open curly, quote, period, minus and plus
    bool atc_tool_offset_hex_string=false;//added 10-18-2021
    float read_fval=0;//11-8-2022
 
#define MAX_FRACTIONAL_DIGITS 6
/*volatile static*/ int max_fractional_digits=MAX_FRACTIONAL_DIGITS;
#define MAX_VAL_STRLEN 80 //sme:arbitrary
/*volatile static*/ char value_string[MAX_VAL_STRLEN];
    nv_reset_nv(nv);                // wipes the object and sets the depth

    // --- Process name part ---
    // Find, terminate and set pointers for the name. Allow for leading and trailing name quotes.
    char * name = *pstr;
    for (i=0; true; i++, (*pstr)++) 
    {
        if (strchr(leaders, (int)**pstr) == NULL) 
        {     // find leading character of name
            name = (*pstr)++;
            break;
        }
        if (i == MAX_PAD_CHARS) {
            return (STAT_JSON_SYNTAX_ERROR);
        }
    }

    // Find the end of name, NUL terminate and copy token
    for (i=0; true; i++, (*pstr)++) 
    {
        if (strchr(separators, (int)**pstr) != NULL) 
        {
            *(*pstr)++ = NUL;
            strncpy(nv->token, name, TOKEN_LEN+1);      // copy the string to the token
            break;
        }
        if (i == TOKEN_LEN) 
        {
            return (STAT_INPUT_EXCEEDS_MAX_LENGTH);
        }
    }

    // --- Process value part ---  (organized from most to least frequently encountered)

    // Find the start of the value part
    for (i=0; true; i++, (*pstr)++) 
    {
        if (isalnum((int)**pstr)) 
          break;
        
        if (strchr(value, (int)**pstr) != NULL) 
          break;
        
        if (i == MAX_PAD_CHARS) 
        {
            return (STAT_JSON_SYNTAX_ERROR);
        }
    }

    // nulls (gets)
    if ((**pstr == 'n') || ((**pstr == '\"') && (*(*pstr+1) == '\"'))) 
    { // process null value
        nv->valuetype = TYPE_NULL;
        nv->value_int = TYPE_NULL;

 
    // numbers
    } 
    else if (isdigit(**pstr) || (**pstr == '-')) 
    {    // value is a number
        strptr=*pstr;
        nv->value_int = atol(*pstr); // get the number as an integer
        

      /* sme: debug "inf": Observe Gc++ compiler strtod, atof lib 
      * calls throw an "inf" for floating pont decimal fractions 
      * with more that 8+ digits  
      */
        
        strcpy(value_string,*pstr);
        char *ptr=strchr(value_string,'.');  
        if (ptr != 0)
        {
           ptr++;
           len=strlen(ptr);
           if (len>max_fractional_digits)
           {
               ptr[max_fractional_digits]=0;
           }
        }
        nv->value_flt = (float)strtod(value_string, &tmp);           
        if (ptr != 0)
        {
           nv->valuetype = TYPE_FLOAT; 
        }
        else
        {
#if 0//sme: 11-29-2022  alternate means to determine float type:
     //requires very expensive call: nv->index=nv_get_index("", nv->token);
        if( (cfgArray[nv->index].flags & TYPE_FLOAT)==TYPE_FLOAT)
        {
            __NOP();
        }
#endif                
            nv->valuetype = TYPE_INTEGER;
        }

        if ((tmp == *pstr) ||                           // if start pointer equals end the conversion failed
            (strchr(terminators, *tmp) == NULL)) 
        {      
           // terminators are the only legal chars at the end of a number
            nv->valuetype = TYPE_NULL;                  // report back an error
            return (STAT_BAD_NUMBER_FORMAT);
        }

        
    // object parent
    } 
    else if (**pstr == '{') 
    {
        nv->valuetype = TYPE_PARENT;
        (*pstr)++;
        return(STAT_EAGAIN);                           // signal that there is more to parse

    // strings
    } 
    else if (**pstr == '\"') 
    {                        // value is a string
        (*pstr)++;
        nv->valuetype = TYPE_STRING;
        if ((tmp = strchr(*pstr, '\"')) == NULL) 
        {
            return (STAT_JSON_SYNTAX_ERROR);            // find the end of the string
        }
        *tmp = NUL;

        // if string begins with 0x it might be data, needs to be at least 3 chars long
        if (strncmp(nv->token, "tuid",4)==0)
        {
              atc_tool_offset_hex_string=true;
        }
        if ( strlen(*pstr)>=3 && (*pstr)[0]=='0' && (*pstr)[1]=='x' && !atc_tool_offset_hex_string)          
        {
            uint32_t *v = (uint32_t*)&nv->value_int;
            *v = strtoul((const char *)*pstr, 0L, 0);
            nv->valuetype = TYPE_DATA;
        } 
        else 
        {
            ritorno(nv_copy_string(nv, *pstr));
        }
        *pstr = ++tmp;

    // boolean true/false
    } 
    else if (**pstr == 't') 
    {
        nv->valuetype = TYPE_BOOLEAN;
        nv->value_int = true;
    } 
    else if (**pstr == 'f') 
    {
        nv->valuetype = TYPE_BOOLEAN;
        nv->value_int = false;

    // arrays
    } 
    else if (**pstr == '[') 
    {
        nv->valuetype = TYPE_ARRAY;
        ritorno(nv_copy_string(nv, *pstr));     // copy array into string for error displays
        return (STAT_VALUE_TYPE_ERROR);         // return error as the parser doesn't do input arrays yet

    // general error condition
    } 
    else 
    {
        return (STAT_JSON_SYNTAX_ERROR);        // ill-formed JSON
    }

    // process comma separators and end curlies
    if ((*pstr = strpbrk(*pstr, terminators)) == NULL) 
    { // advance to terminator or err out
        return (STAT_JSON_SYNTAX_ERROR);
    }
    if (**pstr == '}') 
    {
        *depth -= 1;                            // pop up a nesting level
        (*pstr)++;                              // advance to comma or whatever follows
    }
    if (**pstr == ',') 
    {
        return (STAT_EAGAIN);                   // signal that there is more to parse
    }
    (*pstr)++;
    
    return (STAT_OK);                           // signal that parsing is complete
}

/****************************************************************************
 * json_serialize() - make a JSON object string from JSON object array
 *
 *  *nv is a pointer to the first element in the nv list to serialize
 *  *out_buf is a pointer to the output string - usually what was the input string
 *  Returns the character count of the resulting string
 *
 *   Operation:
 *    - The nvObj list is processed start to finish with no recursion
 *
 *    - Assume the first object is depth 0 or greater (the opening curly)
 *
 *    - Assume remaining depths have been set correctly; but might not achieve closure;
 *      e.g. list starts on 0, and ends on 3, in which case provide correct closing curlies
 *
 *    - Assume there can be multiple, independent, non-contiguous JSON objects at a
 *      given depth value. These are processed correctly - e.g. 0,1,1,0,1,1,0,1,1
 *
 *    - The list must have a terminating nvObj where nv->nx == NULL.
 *      The terminating object may or may not have data (empty or not empty).
 *
 *  Returns:
 *      Returns length of string, or -1 if there's been an error
 *
 *  Desired behaviors:
 *    - Allow self-referential elements that would otherwise cause a recursive loop
 *    - Skip over empty objects (TYPE_EMPTY)
 *    - If a JSON object is empty represent it as {}
 *      --- OR ---
 *    - If a JSON object is empty omit the object altogether (no curlies)
 */

int16_t json_serialize(nvObj_t *nv, char *out_buf, uint16_t size)
{
    char *str = out_buf;
    char *str_max = out_buf + size;
    int8_t initial_depth = nv->depth;
    int8_t prev_depth = 0;
    uint8_t need_a_comma = false;

    *str++ = '{';                                 // write opening curly

    while (true) {
        if (nv->valuetype != TYPE_EMPTY) 
        {
            if (need_a_comma) 
            { 
              *str++ = ',';
            }
            need_a_comma = true;
            strcpy(str++, "\"");
            strcpy(str, nv->token); str += strlen(nv->token);
            strcpy(str++, "\":"); str++;

            switch (nv->valuetype)  
            {
                case (TYPE_EMPTY):  {   break; }
                case (TYPE_NULL):   {   strcpy(str, "null");
                                        str += 4;
                                        break;
                                    }
                case (TYPE_PARENT): {   *str++ = '{';
                                        need_a_comma = false;
                                        prev_depth++; // make sure empty objects are closed
                                        
                                        break;
                                    }
                case (TYPE_FLOAT):  {   convert_outgoing_float(nv);
                                        str += floattoa(str, nv->value_flt, nv->precision);
                                        break;
                                    }
                case (TYPE_INTEGER):{   str += sprintf(str, "%d", (int)nv->value_int);
                                        break;
                                    }
                case (TYPE_STRING): {   *str++ = '"';
                                        strcpy(str, *nv->stringp);
                                        str += strlen(*nv->stringp);
                                        *str++ = '"';
                                        break;
                                    }
                case (TYPE_BOOLEAN):{   if (!nv->value_int) {
                                            strcpy(str, "false");
                                            str += 5;
                                        } else {
                                            strcpy(str, "true");
                                            str += 4;
                                        }
                                        break;
                                    }
                case (TYPE_DATA):   {   uint32_t *v = (uint32_t*)&nv->value_int;
                                        str += sprintf(str, "\"0x%lx\"", *v);
                                        break;
                                    }
                case (TYPE_ARRAY):  {   strcpy(str++, "[");
                                        strcpy(str, *nv->stringp);
                                        str += strlen(*nv->stringp);
                                        strcpy(str++, "]");
                                        break;
                                    }
                default: {}
            }
        }
        if (str >= str_max) { return (-1);}     // signal buffer overrun
        if ((nv = nv->nx) == NULL) { break;}    // end of the list

        while (nv->depth < prev_depth--) 
        {      // iterate the closing curlies
            need_a_comma = true;
            *str++ = '}';
        }
        prev_depth = nv->depth;
    }

    // closing curlies and NEWLINE
    while (prev_depth-- > initial_depth) {
        *str++ = '}';
    }
    str += sprintf((char *)str, "}\n");         // using sprintf for this last one ensures a NUL termination
    if (str > out_buf + size) {
        return (-1);
    }
    return (str - out_buf);
}

/*
 * json_print_object() - serialize and print the nvObj array directly (w/o header & footer)
 *
 *  Ignores JSON verbosity settings and everything else - just serializes the list & prints
 *  Useful for reports and other simple output.
 *  Object list should be terminated by nv->nx == NULL
 */
void json_print_object(nvObj_t *nv)
{
    json_serialize(nv,  cs.out_buf, sizeof(cs.out_buf));
    comms_mgr_write_msg ( cs.out_buf ) ;    
}

/*
 * json_print_list() - command to select and produce a JSON formatted output
 */

void json_print_list(stat_t status, uint8_t flags)
{
    switch (flags) 
    {
     case JSON_NO_PRINT: 
       break;
       
     case JSON_OBJECT_FORMAT: 
         json_print_object(nv_body);        
         break;
         
     case JSON_RESPONSE_FORMAT:
     case JSON_RESPONSE_TO_MUTED_FORMAT:
        json_print_response(status);             
        break; 
    }
}

/*
 * json_print_response() - JSON responses with headers, footers and observing JSON verbosity
 *
 *  A footer is returned for every setting except $jv=0
 *
 *  JV_SILENT = 0,      // no response is provided for any command
 *  JV_FOOTER,          // responses contain  footer only; no command echo, gcode blocks or messages
 *  JV_CONFIGS,         // echo configs; gcode blocks are not echoed; messages are not echoed
 *  JV_MESSAGES,        // echo configs; gcode messages only (if present); no block echo or line numbers
 *  JV_LINENUM,         // echo configs; gcode blocks return messages and line numbers as present
 *  JV_VERBOSE          // echoes all configs and gcode blocks, line numbers and messages
 *
 *  This gets a bit complicated. The first nvObj is the header, which must be set by reset_nv_list().
 *  The first object in the body will always have the gcode block or config command in it,
 *  which you may or may not want to display. This is followed by zero or more displayable objects.
 *  Then if you want a gcode line number you add that here to the end. Finally, a footer goes
 *  on all the (non-silent) responses.
 */

void json_print_response(uint8_t status)
{

    if ((js.json_verbosity == JV_SILENT) || (cs.responses_suppressed)) 
    {                   // silent means no responses
        return;
    }
    if (js.json_verbosity == JV_EXCEPTIONS)    
    {            // cutout for JV_EXCEPTIONS mode
        if (status == STAT_OK) 
        {
            if (cm->machine_state != MACHINE_INITIALIZING) 
            { // always do full echo during startup
                return;
            }
        }
    }

    // Body processing
    nvObj_t *nv = nv_body;
    if (status == STAT_JSON_SYNTAX_ERROR) 
    {
        nv_reset_nv_list();
        nv_add_string((const char *)"err", escape_string(cs.bufp, cs.saved_buf));

    }
    else if ((cm->machine_state != MACHINE_INITIALIZING) || (status == STAT_INITIALIZING)) 
    { // always do full echo during startup
        uint8_t nv_type;
        do 
        {
            if ((nv_type = nv_get_type(nv)) == NV_TYPE_NULL)               break;

            if (nv_type == NV_TYPE_GCODE) 
            {
                if (js.echo_json_gcode_block == false) 
                {    // kill command echo if not enabled
                    nv->valuetype = TYPE_EMPTY;
                }
            } 
            else if (nv_type == NV_TYPE_MESSAGE) 
            {        // kill message echo if not enabled
                if (js.echo_json_messages == false) 
                {
                    nv->valuetype = TYPE_EMPTY;
                }

            } 
            else if (nv_type == NV_TYPE_LINENUM) 
            {        // kill line number echo if not enabled
                if ((js.echo_json_linenum == false) || (fp_ZERO(nv->value_int))) 
                { // do not report line# 0
                    nv->valuetype = TYPE_EMPTY;
                }
            }
        } while ((nv = nv->nx) != NULL);                    // Emergency escape
    }

    // Footer processing - wind to the end of the populated blocks
    if (nv == NULL) 
    {                        // this can happen when processing a stale list
        return;              //...that already has a null-terminated footer
    }
    while(nv->valuetype != TYPE_EMPTY) 
    {                    // find a free nvObj at end of the list...
        if ((nv = nv->nx) == NULL) 
        {                        // oops! No free nvObj!
            rpt_exception(STAT_JSON_OUTPUT_TOO_LONG, (char*)"json_print_response() json too long"); // report this as an exception
            return;
        }
    }

    // in xio.cpp:xio.readline the CR || LF read from the host is not appended to the string.
    // to ensure that the correct number of bytes are reported back to the host we add a +1 to
    // cs.linelen so that the number of bytes received matches the number of bytes reported

    char footer_string[NV_FOOTER_LEN];
    char *str = footer_string;

    strcpy(str, "1,"); str += 2;                            // '1' is the footer revision hard coded
    str += inttoa(str, status);                             // nb: inttoa() works differently than itoa(). See util.cpp
    strcpy(str++, ",");
    str += inttoa(str, cs.linelen+1);
    cs.linelen = 0;                                         // reset linelen so it's only reported once

    nv_copy_string(nv, footer_string);                      // link string to nv object
    nv->depth = 0;                                          // footer 'f' is a peer to response 'r' (hard wired to 0)
    nv->valuetype = TYPE_ARRAY;                             // declare it as an array
    strcpy(nv->token, "f");                                 // set it to Footer
    nv->nx = NULL;                                          // terminate the list

    // serialize the JSON response and print it if there were no errors
    if (json_serialize(nv_header, cs.out_buf, sizeof(cs.out_buf)) >= 0) 
    {          
        comms_mgr_write_msg(cs.out_buf);         
    }
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * js_get_ej() - get JSON communications mode
 * js_set_ej() - set JSON communications mode
 *
 * This one is a bit different:
 *  - cs.comm_mode is the setting for the *communications mode* (persistent)
 *  - js.json_mode is the actual current mode
 *
 *  If comm_mode is set to TEXT_MORE (0) or JSON_MODE (1) then json_mode should also be changed
 *  If comm_mode is set to AUTO_MORE (0) then json_mode should not be changed
 */

stat_t js_get_ej(nvObj_t *nv) { return(get_integer(nv, cs.comm_mode)); }
stat_t js_set_ej(nvObj_t *nv)
{
    ritorno (set_int_u8(nv, (uint8_t &)cs.comm_mode, TEXT_MODE, AUTO_MODE));
    if (commMode(nv->value_int) < AUTO_MODE) {    // set json_mode to 0 or 1, but don't change it if comm_mode == 2
        js.json_mode = commMode(nv->value_int);
    }
    return (STAT_OK);
}

/*
 * js_get_jv() - get JSON verbosity
 * js_set_jv() - set JSON verbosity and related flags
 */

stat_t js_get_jv(nvObj_t *nv) { return(get_integer(nv, js.json_verbosity)); }
stat_t js_set_jv(nvObj_t *nv)
{
    ritorno (set_int_u8(nv, (uint8_t &)js.json_verbosity, JV_SILENT, JV_MAX_VALUE));

    js.echo_json_footer = false;
    js.echo_json_messages = false;
    js.echo_json_configs = false;
    js.echo_json_linenum = false;
    js.echo_json_gcode_block = false;

    if (js.json_verbosity == JV_EXCEPTIONS) {
        js.echo_json_footer = true;
        js.echo_json_messages = true;
        js.echo_json_configs = true;
    } else {
        if (nv->value_int >= JV_FOOTER)     js.echo_json_footer = true;
        if (nv->value_int >= JV_MESSAGES)   js.echo_json_messages = true;
        if (nv->value_int >= JV_CONFIGS)    js.echo_json_configs = true;
#if 1//sme: 10-26-2022 experiment--is this slowing down performance?
        if (nv->value_int >= JV_LINENUM)    js.echo_json_linenum = true;
#endif
        if (nv->value_int >= JV_VERBOSE)    js.echo_json_gcode_block = true;
    }
    return(STAT_OK);
}


/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

/*
 * js_print_ej()
 * js_print_jv()
 * js_print_js()
 * js_print_jf()
 */

static const char fmt_ej[] = "[ej]  enable json mode%13d [0=text,1=JSON,2=auto]\n";
static const char fmt_jv[] = "[jv]  json verbosity%15d [0=silent,1=footer,2=messages,3=configs,4=linenum,5=verbose]\n";
static const char fmt_js[] = "[js]  json serialize style%9d [0=relaxed,1=strict]\n";
static const char fmt_jf[] = "[jf]  json footer style%12d [1=checksum,2=window report]\n";

void js_print_ej(nvObj_t *nv) { text_print(nv, fmt_ej);}    // TYPE_INT
void js_print_jv(nvObj_t *nv) { text_print(nv, fmt_jv);}    // TYPE_INT
void js_print_js(nvObj_t *nv) { text_print(nv, fmt_js);}    // TYPE_INT
void js_print_jf(nvObj_t *nv) { text_print(nv, fmt_jf);}    // TYPE_INT

#endif // __TEXT_MODE
