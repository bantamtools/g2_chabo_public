
/*
 * gcode_parser.cpp - rs274/ngc Gcode parser
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Rob Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
#include "g2core.h"  // #1
#include "g2_config.h"  // #2
#include "g2_controller.h"
#include "g2_gcode.h"
#include "g2_canonical_machine.h"
#include "g2_settings.h"
#include "g2_spindle.h"
#include "g2_util.h"     
#include "g2_json_parser.h"
#include "g2_report.h"
#include "comms_mgr.h"

extern void debug_reset_rx_msg_counter(void);   
// Helpers

// Locally used enums

typedef enum {                          // Used for detecting gcode errors. See NIST section 3.4
    MODAL_GROUP_G0 = 0,                 // {G10,G28,G28.1,G92}  non-modal axis commands (note 1)
    MODAL_GROUP_G1,                     // {G0,G1,G2,G3,G80}    motion
    MODAL_GROUP_G2,                     // {G17,G18,G19}        plane selection
    MODAL_GROUP_G3,                     // {G90,G91}            distance mode
    MODAL_GROUP_G5,                     // {G93,G94}            feed rate mode
    MODAL_GROUP_G6,                     // {G20,G21}            units
    MODAL_GROUP_G7,                     // {G40,G41,G42}        cutter radius compensation
    MODAL_GROUP_G8,                     // {G43,G49}            tool length offset
    MODAL_GROUP_G9,                     // {G98,G99}            return mode in canned cycles
    MODAL_GROUP_G12,                    // {G54,G55,G56,G57,G58,G59} coordinate system selection
    MODAL_GROUP_G13,                    // {G61,G61.1,G64}      path control mode
    MODAL_GROUP_M4,                     // {M0,M1,M2,M30,M60}   stopping
    MODAL_GROUP_M6,                     // {M6}                 tool change
    MODAL_GROUP_M7,                     // {M3,M4,M5}           spindle turning
    MODAL_GROUP_M8,                     // {M7,M8,M9}           coolant (M7 & M8 may be active together)
    MODAL_GROUP_M9                      // {M48,M49}            speed/feed override switches
} cmModalGroup;
#define MODAL_GROUP_COUNT (MODAL_GROUP_M9+1)
// Note 1: Our G0 omits G4,G30,G53,G92.1,G92.2,G92.3 as these have no axis components to error check

/* The difference between NextAction and MotionMode (in canonical machine) is that
*  NextAction is used by the current block, and may carry non-modal commands, whereas
 * MotionMode persists across blocks (as G modal group 1)
 */

typedef enum {                                  // these are in order to optimized CASE statement
    NEXT_ACTION_DEFAULT = 0,                    // Must be zero (invokes motion modes)
    NEXT_ACTION_DWELL,                          // G4
    NEXT_ACTION_SET_G10_DATA,                   // G10
    NEXT_ACTION_GOTO_G28_POSITION,              // G28 go to machine position
    NEXT_ACTION_SET_G28_POSITION,               // G28.1 set position in abs coordinates
    NEXT_ACTION_SEARCH_HOME,                    // G28.2 homing cycle
    NEXT_ACTION_SET_ABSOLUTE_ORIGIN,            // G28.3 origin set
    NEXT_ACTION_HOMING_NO_SET,                  // G28.4 homing cycle with no coordinate setting
    NEXT_ACTION_GOTO_G30_POSITION,              // G30 go to machine position
    NEXT_ACTION_SET_G30_POSITION,               // G30.1 set position in abs coordinates
    NEXT_ACTION_STRAIGHT_PROBE_ERR,             // G38.2
    NEXT_ACTION_STRAIGHT_PROBE,                 // G38.3
    NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR,        // G38.4
    NEXT_ACTION_STRAIGHT_PROBE_AWAY,            // G38.5
    NEXT_ACTION_SET_TL_OFFSET,                  // G43
    NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET,       // G43.2
    NEXT_ACTION_CANCEL_TL_OFFSET,               // G49
    NEXT_ACTION_SET_G92_OFFSETS,                // G92
    NEXT_ACTION_RESET_G92_OFFSETS,              // G92.1
    NEXT_ACTION_SUSPEND_G92_OFFSETS,            // G92.2
    NEXT_ACTION_RESUME_G92_OFFSETS,             // G92.3
    NEXT_ACTION_JSON_COMMAND_SYNC,              // M100
    NEXT_ACTION_JSON_COMMAND_ASYNC,             // M100.1
    NEXT_ACTION_JSON_WAIT,                      // M101

} gpNextAction;

// Structures used by Gcode parser

typedef struct GCodeInputValue {    // Gcode inputs - meaning depends on context

    gpNextAction next_action;       // handles G modal group 1 moves & non-modals
    cmMotionMode motion_mode;       // Group1: G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89
    uint8_t program_flow;           // used only by the gcode_parser
    uint32_t linenum;               // gcode N word

    float target[AXES];             // XYZABC where the move should go
    float arc_offset[3];            // IJK - used by arc commands
    float arc_radius;               // R word - radius value in arc radius mode
    float F_word;                   // F word - feedrate as present in the F word (will be normalized later)
    float P_word;                   // P word - parameter used for dwell time in seconds, G10 commands
    float S_word;                   // S word - usually in RPM
    uint8_t H_word;                 // H word - used by G43s
    uint8_t L_word;                 // L word - used by G10s

    uint8_t feed_rate_mode;         // See cmFeedRateMode for settings
    uint8_t select_plane;           // G17,G18,G19 - values to set plane to
    uint8_t units_mode;             // G20,G21 - 0=inches (G20), 1 = mm (G21)
    uint8_t coord_system;           // G54-G59 - select coordinate system 1-9
    uint8_t path_control;           // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    uint8_t distance_mode;          // G91   0=use absolute coords(G90), 1=incremental movement
    uint8_t arc_distance_mode;      // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    uint8_t origin_offset_mode;     // G92...TRUE=in origin offset mode
    uint8_t absolute_override;      // G53 TRUE = move using machine coordinates - this block only (G53)

    uint8_t tool;                   // Tool after T and M6 (tool_select and tool_change)
    uint8_t tool_select;            // T value - T sets this value
    uint8_t tool_change;            // M6 tool change flag - moves "tool_select" to "tool"
    uint8_t coolant_mist;           // TRUE = mist on (M7)
    uint8_t coolant_flood;          // TRUE = flood on (M8)
    uint8_t coolant_off;            // TRUE = turn off all coolants (M9)
    uint8_t spindle_control;        // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)

    bool m48_enable;                // M48/M49 input (enables for feed and spindle)
    bool fro_control;               // M50 feedrate override control
    bool tro_control;               // M50.1 traverse override control
    bool spo_control;               // M51 spindle speed override control
} GCodeValue_t;

typedef struct GCodeFlags {         // Gcode input flags

    bool next_action;
    bool motion_mode;
    bool program_flow;
    bool linenum;

    bool target[AXES];
    bool arc_offset[3];
    bool arc_radius;

    bool F_word;
    bool P_word;
    bool S_word;
    bool H_word;
    bool L_word;

    bool feed_rate_mode;
    bool select_plane;
    bool units_mode;
    bool coord_system;
    bool path_control;
    bool distance_mode;
    bool arc_distance_mode;
    bool origin_offset_mode;
    bool absolute_override;

    bool tool;
    bool tool_select;
    bool tool_change;
    bool coolant_mist;
    bool coolant_flood;
    bool coolant_off;
    bool spindle_control;

    bool m48_enable;
    bool fro_control;
    bool tro_control;
    bool spo_control;
    bool checksum;
} GCodeFlag_t;

typedef struct GCodeParser 
{
    bool modals[MODAL_GROUP_COUNT];
} GCodeParser_t;

GCodeParser_t gp;   // main parser struct
GCodeValue_t gv;    // gcode input values
GCodeFlag_t gf;     // gcode input flags

// local helper functions and macros
static void _normalize_gcode_block(char *str, char **active_comment, bool *block_delete_flag);
static stat_t _get_next_gcode_word(char **pstr, char *letter, float *value, int32_t *value_int);
static stat_t _point(float value);
static stat_t _verify_checksum(char *str);
static stat_t _validate_gcode_block(char *active_comment);
static stat_t _parse_gcode_block(char *line, char *active_comment); // Parse the block into the GN/GF structs
static stat_t _execute_gcode_block(char *active_comment);           // Execute the gcode block

#define SET_MODAL(m,parm,val) ({gv.parm=val; gf.parm=true; gp.modals[m]=true; break;})
#define SET_NON_MODAL(parm,val) ({gv.parm=val; gf.parm=true; break;})
#define EXEC_FUNC(f,v) if(gf.v) { status=f(gv.v);}
float get_feedrate(void){return gv.F_word;}
/*
 * gcode_parser_init()
 */

void gcode_parser_init()
{
    memset(&gv, 0, sizeof(GCodeValue_t));
    memset(&gf, 0, sizeof(GCodeFlag_t));
}

/*
 * gcode_parser() - parse a block (line) of gcode
 *
 *  Top level of gcode parser. Normalizes block and looks for special cases
 */

stat_t gcode_parser(char *block)
{
    char *str = block;                      // gcode command or NUL string
    char none = NUL;
    char *active_comment = &none;           // gcode comment or NUL string
    bool block_delete_flag;

    stat_t check_ret = _verify_checksum(str);
    if (check_ret != STAT_OK) {
        return check_ret;
    }
   
    _normalize_gcode_block(str, &active_comment, &block_delete_flag);

    // TODO, now MSG is put in the active comment, handle that.

    if (str[0] == NUL) {                    // normalization returned null string
        if ((active_comment != nullptr) && (active_comment[0] == '{')) {
            json_parser(active_comment);
        }
        return (STAT_OK);                   // most likely a comment line
    }

    // Trap M30 and M2 as $clear conditions. This has no effect if not in ALARM or SHUTDOWN
    cm_parse_clear(str);                    // parse Gcode and clear alarms if M30 or M2 is found
    ritorno(cm_is_alarmed());               // return error status if in alarm, shutdown or panic

    // Block delete omits the line if a / char is present in the first space
    // For now this is unconditional and will always delete
//  if ((block_delete_flag == true) && (cm_get_block_delete_switch() == true)) {
    if (block_delete_flag == true) {
        return (STAT_NOOP);
    }
    return(_parse_gcode_block(block, active_comment));
}

/*
 * _verify_checksum() - ensure that, if there is a checksum, that it's valid
 *
 * Returns STAT_OK is it's valid.
 * Returns STAT_CHECKSUM_MATCH_FAILED if the checksum doesn't match.
 */
static stat_t _verify_checksum(char *str)
{
    bool has_line_number = false; // -1 means we don't have one
    if (*str == 'N') {
        has_line_number = true;
    }

    char checksum = 0;
    char c = *str++;
    while (c && (c != '*') && (c != '\n') && (c != '\r')) {
        checksum ^= c;
        c = *str++;
    }

    // c might be 0 here, in which case we didn't get a checksum and we return STAT_OK

    if (c == '*') {
        *(str-1) = 0; // null terminate, the parser won't like this * here!
        gf.checksum = true;
        if (strtol(str, NULL, 10) != checksum) {
            debug_trap("checksum failure");
            return STAT_CHECKSUM_MATCH_FAILED;
        }
        if (!has_line_number)  {
            debug_trap("line number missing with checksum");
            return STAT_MISSING_LINE_NUMBER_WITH_CHECKSUM;
        }
    }
    return STAT_OK;
}

/****************************************************************************************
 * _normalize_gcode_block() - normalize a block (line) of gcode in place
 *
 *  Baseline normalization functions:
 *   - Isolate comments. See below.
 *   The rest of this applies just to the GCODE string itself (not the comments):
 *   - Remove white space, control and other invalid characters
 *   - Convert all letters to upper case
 *   - Remove (erroneous) leading zeros that might be taken to mean Octal
 *   - Signal if a block-delete character (/) was encountered in the first space
 *   - NOTE: Assumes no leading whitespace as this was removed at the controller dispatch level
 *
 *  So this: "g1 x100 Y100 f400" becomes this: "G1X100Y100F400"
 *
 *  Comment, active comment and message handling:
 *   - Comment fields start with a '(' char or alternately a semicolon ';' or percent '%'
 *   - Semicolon ';' or percent '%' end the line. All characters past are discarded
 *   - Multiple embedded comments are acceptable if '(' form
 *   - Active comments start with exactly "({" and end with "})" (no relaxing, invalid is invalid)
 *   - Active comments are moved to the end of the string
 *   - Multiple active comments are merged and moved to the end of the string
 *   - Gcode message comments (MSG) are converted to ({msg:"blah"}) active comments
 *     - The 'MSG' specifier in comment can have mixed case but cannot cannot have embedded white spaces
 *     - Only ONE MSG comment will be accepted
 *   - Other "plain" comments are discarded
 *
 *  Returns:
 *   - com points to comment string or to NUL if no comment
 *   - msg points to message string or to NUL if no comment
 *   - block_delete_flag is set true if block delete encountered, false otherwise
 */
/* Active comment notes:
 *
 *   We will convert as follows:
 *   FROM: G0 ({blah: t}) x10 (comment)
 *   TO  : g0x10\0{blah:t}
 *   NOTES: Active comments moved to the end, stripped of (), everything lowercased, and plain comment removed.
 *
 *   FROM: M100 ({a:t}) (comment) ({b:f}) (comment)
 *   TO  : m100\0{a:t,b:f}
 *   NOTES: multiple active comments merged, stripped of (), and actual comments ignored.
 */

char _normalize_scratch[RX_BUFFER_SIZE];

void _normalize_gcode_block(char *str, char **active_comment, bool *block_delete_flag)
{
    _normalize_scratch[0] = 0;
    volatile int len=0;
    char *gc_rd = str;                  // read pointer
    char *gc_wr = _normalize_scratch;   // write pointer
    char *ac_rd = str;                  // Active Comment read pointer
    char *ac_wr = _normalize_scratch;   // Active Comment write pointer
    bool last_char_was_digit = false;   // used for octal stripping

    // Move the ac_wr point forward one for every non-AC character we KEEP (plus one for a NULL in between)
    ac_wr++;                            // account for the in-between NULL

    // mark block deletes
    if (*gc_rd == '/') {
        *block_delete_flag = true;
        gc_rd++;
    } else {
        *block_delete_flag = false;
    }

    while (*gc_rd != 0) {
        if ((*gc_rd == ';') || (*gc_rd == '%')) {   // check for ';' or '%' comments that end the line
            *gc_rd = 0;                             // go ahead and snap the string off cleanly here
            break;
        }

        // check for comment '('
        else if (*gc_rd == '(') {
            // We only care if it's a "({" in order to handle string-skipping properly
            gc_rd++;
            if ((*gc_rd == '{') || (((* gc_rd    == 'm') || (* gc_rd    == 'M')) &&
                                    ((*(gc_rd+1) == 's') || (*(gc_rd+1) == 'S')) &&
                                    ((*(gc_rd+2) == 'g') || (*(gc_rd+2) == 'G'))
                )) {
                if (ac_rd == nullptr) {
                    ac_rd = gc_rd;      // note the start of the first AC
                }

                // skip the comment, handling strings carefully
                bool in_string = false;
                while (*(++gc_rd) != 0) {
                    if (*gc_rd=='"') {
                        in_string = true;
                    } else if (in_string) {
                        if ((*gc_rd == '\\') && (*(gc_rd+1) != 0)) {
                            gc_rd++; // Skip it, it's escaped.
                        }
                    } else if ((*gc_rd == ')')) {
                        break;
                    }
                }
                if (*gc_rd == 0) {      // We don't want the rd++ later to skip the NULL if we're at one
                    break;
                }
            } else {
                *(gc_rd-1) = ' ';       // Change the '(' to a space to simplify the comment copy later
                while ((*gc_rd != 0) && (*gc_rd != ')')) {  // skip ahead until we find a ')' (or NULL)
                    gc_rd++;
                }
            }
        } else if (!isspace(*gc_rd)) {
            bool do_copy = false;

            // Perform Octal stripping - remove invalid leading zeros in number strings
            // Otherwise number conversions can fail, as Gcode does not support octal but C libs do
            // Change 0123.004 to 123.004, or -0234.003 to -234.003
            if (isdigit(*gc_rd) || (*gc_rd == '.')) { // treat '.' as a digit so we don't strip after one
                if (last_char_was_digit || (*gc_rd != '0') || !isdigit(*(gc_rd+1))) {
                    do_copy = true;
                }
                last_char_was_digit = true;
            }
            else if ((isalnum((char)*gc_rd)) || (strchr("-.", *gc_rd))) { // all valid characters
                last_char_was_digit = false;
                do_copy = true;
            }
            if (do_copy) {
                *(gc_wr++) = toupper(*gc_rd);
                ac_wr++; // move the ac start position
            }
        }
        gc_rd++;
    }

    // Enforce null termination
    *gc_wr = 0;
    char *comment_start = ac_wr;    // note the beginning of the comments
    if (ac_rd != nullptr) {

        // Now we'll copy the comments to the scratch
        while (*ac_rd != 0) {
            // check for comment '('
            // Remember: we're only "counting characters" at this point, no more.
            if (*ac_rd == '(') {
                // We only care if it's a "({" in order to handle string-skipping properly
                ac_rd++;

                bool do_copy = false;
                bool in_msg = false;
                if (((* ac_rd    == 'm') || (* ac_rd    == 'M')) &&
                    ((*(ac_rd+1) == 's') || (*(ac_rd+1) == 'S')) &&
                    ((*(ac_rd+2) == 'g') || (*(ac_rd+2) == 'G'))
                    ) {

                    ac_rd += 3;
                    if (*ac_rd == ' ') {
                        ac_rd++; // skip the first space.
                    }

                    if (*(ac_wr-1) == '}') {
                        *(ac_wr-1) = ',';
                    } else {
                        *(ac_wr++) = '{';
                    }
                    *(ac_wr++) = 'm';
                    *(ac_wr++) = 's';
                    *(ac_wr++) = 'g';
                    *(ac_wr++) = ':';
                    *(ac_wr++) = '"';

                    // TODO - FIX BUFFER OVERFLOW POTENTIAL
                    // "(msg)" is four characters. "{msg:" is five. If the write buffer is full, we'll overflow.
                    // Also " is MSG will be quoted, making one character into two.

                    in_msg = true;
                    do_copy = true;
                }

                else if (*ac_rd == '{') {
                    // merge json comments
                    if (*(ac_wr-1) == '}') {
                        *(ac_wr-1) = ',';

                        // don't copy the '{'
                        ac_rd++;
                    }

                    do_copy = true;
                }

                if (do_copy) {
                    // skip the comment, handling strings carefully
                    bool in_string = false;
                    bool escaped = false;
                    while (*ac_rd != 0) {
                        if (in_string && (*ac_rd == '\\')) {
                            escaped = true;
                        } else if (!escaped && (*ac_rd == '"')) {
                            // In msg comments, we have to escape "
                            if (in_msg) {
                                *(ac_wr++) = '\\';
                            } else {
                                in_string = !in_string;
                            }
                        } else if (!in_string && (*ac_rd == ')')) {
                            ac_rd++;
                            if (in_msg) {
                                *(ac_wr++) = '"';
                                *(ac_wr++) = '}';
                            }
                            break;
                        } else {
                            escaped = false;
                        }

                        // Skip spaces if we're not in a string or msg (implicit string)
                        if (in_string || in_msg || (*ac_rd != ' ')) {
                            *ac_wr = *ac_rd;
                            ac_wr++;
                        }

                        ac_rd++;
                    }
                }

                // We don't want the rd++ later to skip the NULL if we're at one
                if (*ac_rd == 0) {
                    break;
                }
            }
            ac_rd++;
        }
    }

    // Enforce null termination
    *ac_wr = 0;
len=(ac_wr-_normalize_scratch)+1;//sme: 6-2-2021 debug atc 
    // Now copy it all back
    memcpy(str, _normalize_scratch, len);//(ac_wr-_normalize_scratch)+1);

    *active_comment = str + (comment_start - _normalize_scratch);
}

/****************************************************************************************
 * _get_next_gcode_word() - get gcode word consisting of a letter and a value
 *
 *  This function requires the Gcode string to be normalized.
 *  Normalization must remove any leading zeros or they will be converted to Octal
 *  G0X... is not interpreted as hexadecimal. This is trapped.
 */

static stat_t _get_next_gcode_word(char **pstr, char *letter, float *value, int32_t *value_int)
{
    volatile static int bad_counter=0;
    if (**pstr == NUL) { return (STAT_COMPLETE); }    // no more words

    // get letter part
    if(isupper(**pstr) ==(int)false) 
    {
        bad_counter++;
#define BAD_COUNT_MAX 100//arbitray
        if (bad_counter>BAD_COUNT_MAX)
        {
            bad_counter=-1;
        }
        return (STAT_INVALID_OR_MALFORMED_COMMAND);
    }
    *letter = **pstr;
    (*pstr)++;

    // get-value general case
    char *end = *pstr;
    *value = c_atof(end);
    *value_int = atol(*pstr);                       // needed to get an accurate line number for N > 8,388,608

    if (end == *pstr) 
    {
        return(STAT_BAD_NUMBER_FORMAT);
    }    // more robust test then checking for value=0;
    *pstr = end;
    return (STAT_OK);                               // pointer points to next character after the word
}

/*
 * _point() - isolate the decimal point value as an integer
 */

static uint8_t _point(const float value)
{
    volatile static float copy_value = 0;//sme 8-2-2022
    volatile static float rounded_value = 0;//sme 8-2-2022
    volatile static float truncated_value =0;//sme 8-2-2022
    volatile static float result_value=0;//sme 8-2-2022
    copy_value = value;//sme 8-2-2022
    rounded_value = std::round(value*10.0);//sme 8-2-2022
    truncated_value =std::trunc(value)*10.0;//sme 8-2-2022
    result_value=rounded_value-truncated_value;//sme 8-2-2022
    return result_value;//sme 8-2-2022
    //sme: 8-2-2022 return((uint8_t)(std::round(value*10.0) - std::trunc(value)*10.0));    // isolate the decimal point as an int
}

/****************************************************************************************
 * _validate_gcode_block() - check for some gross Gcode block semantic violations
 */

static stat_t _validate_gcode_block(char *active_comment)
{
//  Check for modal group violations. From NIST, section 3.4 "It is an error to put
//  a G-code from group 1 and a G-code from group 0 on the same line if both of them
//  use axis words. If an axis word-using G-code from group 1 is implicitly in effect
//  on a line (by having been activated on an earlier line), and a group 0 G-code that
//  uses axis words appears on the line, the activity of the group 1 G-code is suspended
//  for that line. The axis word-using G-codes from group 0 are G10, G28, G30, and G92"

//  if ((gp.modals[MODAL_GROUP_G0] == true) && (gp.modals[MODAL_GROUP_G1] == true)) {
//     return (STAT_MODAL_GROUP_VIOLATION);
//  }

// look for commands that require an axis word to be present
//  if ((gp.modals[MODAL_GROUP_G0] == true) || (gp.modals[MODAL_GROUP_G1] == true)) {
//     if (_axis_changed() == false)
//     return (STAT_GCODE_AXIS_IS_MISSING);
//  }
    return (STAT_OK);
}

/****************************************************************************************
 * _parse_gcode_block() - parses one line of NULL terminated G-Code.
 *
 *  All the parser does is load the state values in gn (next model state) and set flags
 *  in gf (model state flags). The execute routine applies them. The buffer is assumed to
 *  contain only uppercase characters and signed floats (no whitespace).
 */

static stat_t _parse_gcode_block(char *buf, char *active_comment)
{
    char *pstr = (char *)buf;                   // persistent pointer into gcode block for parsing words
    char letter;                                // parsed letter, eg.g. G or X or Y
    float value = 0;                            // value parsed from letter (e.g. 2 for G2)
    int32_t value_int = 0;                      // integer value parsed from letter - needed for line numbers
    stat_t status = STAT_OK;

    // set initial state for new move
    memset(&gv, 0, sizeof(GCodeValue_t));       // clear all next-state values
    memset(&gf, 0, sizeof(GCodeFlag_t));        // clear all next-state flags
    gv.motion_mode = cm_get_motion_mode(MODEL); // get motion mode from previous block
  
    if (cm->gm.feed_rate_mode == INVERSE_TIME_MODE) 
    {// new feed rate required when in INV_TIME_MODE
        gv.F_word = 0;
        gf.F_word = true;
    }
   
//#define SET_MODAL(m,parm,val) ({gv.parm=val; gf.parm=true; gp.modals[m]=true; break;})
//#define SET_NON_MODAL(parm,val) ({gv.parm=val; gf.parm=true; break;})
//#define EXEC_FUNC(f,v) if(gf.v) { status=f(gv.v);}

    // extract commands and parameters
    while((status = _get_next_gcode_word(&pstr, &letter, &value, &value_int)) == STAT_OK) 
    {
        switch(letter) 
        {
            case 'G':            
            switch((uint8_t)value) 
            {
                case 0:  
                  gv.motion_mode=MOTION_MODE_STRAIGHT_TRAVERSE; 
                  gf.motion_mode=true; gp.modals[MODAL_GROUP_G1]=true; 
                  break; 
                  //SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_TRAVERSE);
                
                case 1:  
                  //SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_FEED);
                   gv.motion_mode=MOTION_MODE_STRAIGHT_FEED; 
                   gf.motion_mode=true; gp.modals[MODAL_GROUP_G1]=true; 
                   break;
                case 2:  
                  //SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_CW_ARC);
                  gv.motion_mode=MOTION_MODE_CW_ARC; 
                  gf.motion_mode=true; gp.modals[MODAL_GROUP_G1]=true; 
                  break;
                  
                case 3:  
                  //SET_MODAL (MODAL_GROUP_G1, motion_mode, MOTION_MODE_CCW_ARC);
                  {gv.motion_mode=MOTION_MODE_CCW_ARC; gf.motion_mode=true; gp.modals[MODAL_GROUP_G1]=true; break;}
                case 4:  
                  //SET_NON_MODAL (next_action, NEXT_ACTION_DWELL);
                  {gv.next_action=NEXT_ACTION_DWELL; gf.next_action=true; break;}
                case 10: 
                  //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G10_DATA);
                   {gv.next_action=NEXT_ACTION_SET_G10_DATA; gf.next_action=true; gp.modals[MODAL_GROUP_G0]=true; break;}
                case 17: 
                  //SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_XY);
                   {gv.select_plane=CANON_PLANE_XY; gf.select_plane=true; gp.modals[MODAL_GROUP_G2]=true; break;}
                case 18: 
                  //SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_XZ);
                   {gv.select_plane=CANON_PLANE_XZ; gf.select_plane=true; gp.modals[MODAL_GROUP_G2]=true; break;}
                case 19: 
                  //SET_MODAL (MODAL_GROUP_G2, select_plane, CANON_PLANE_YZ);
                   {gv.select_plane=CANON_PLANE_YZ; gf.select_plane=true; gp.modals[MODAL_GROUP_G2]=true; break;}
                case 20: 
                  //SET_MODAL (MODAL_GROUP_G6, units_mode, INCHES);
                   {gv.units_mode=INCHES; gf.units_mode=true; gp.modals[MODAL_GROUP_G6]=true; break;}
                case 21: 
                  //SET_MODAL (MODAL_GROUP_G6, units_mode, MILLIMETERS);
                   {gv.units_mode=MILLIMETERS; gf.units_mode=true; gp.modals[MODAL_GROUP_G6]=true; break;}
                case 28: 
                {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G28_POSITION);
                          {gv.next_action= NEXT_ACTION_GOTO_G28_POSITION; gf.next_action=true; gp.modals[MODAL_GROUP_G0]=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G28_POSITION);
                           {gv.next_action= NEXT_ACTION_SET_G28_POSITION; gf.next_action=true; gp.modals[MODAL_GROUP_G0]=true; break;}
                        case 2: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_SEARCH_HOME);
                          {gv.next_action=NEXT_ACTION_SEARCH_HOME; gf.next_action=true; break;}
                        case 3: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_SET_ABSOLUTE_ORIGIN);
                           { gv.next_action=NEXT_ACTION_SET_ABSOLUTE_ORIGIN; gf.next_action=true; break;}
                        case 4: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_HOMING_NO_SET);
                            {gv.next_action=NEXT_ACTION_HOMING_NO_SET; gf.next_action=true; break;}
        
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }

                case 30: 
                {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G30_POSITION);
                           {gv.next_action=NEXT_ACTION_GOTO_G30_POSITION; gf.next_action=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G30_POSITION);
                           {gv.next_action=NEXT_ACTION_SET_G30_POSITION; gf.next_action=true; break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 38: 
                {
                    switch (_point(value)) 
                    {
                        case 2: 
                          gv.next_action=NEXT_ACTION_STRAIGHT_PROBE_ERR; 
                          gf.next_action=true; 
                          break;
                        case 3: 
                          gv.next_action=NEXT_ACTION_STRAIGHT_PROBE; 
                          gf.next_action=true; 
                          break;
                        case 4: 
                           gv.next_action=NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR; 
                           gf.next_action=true; 
                           break;
                        case 5: 
                           gv.next_action=NEXT_ACTION_STRAIGHT_PROBE_AWAY; 
                           gf.next_action=true; 
                           break;
                        default: 
                           status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 40: 
                  break;    // ignore cancel cutter radius compensation. But don't fail G40s.
                case 43: 
                {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_SET_TL_OFFSET);
                          {gv.next_action=NEXT_ACTION_SET_TL_OFFSET; gf.next_action=true; break;}
                        case 2: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET);
                          {gv.next_action=NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET; gf.next_action=true; break;}
                    
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
		case 49: 
                  //SET_NON_MODAL (next_action, NEXT_ACTION_CANCEL_TL_OFFSET);
                  {gv.next_action=NEXT_ACTION_SET_TL_OFFSET; gf.next_action=true; break;}
                case 53: 
                  //SET_NON_MODAL (absolute_override, ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS);
                  {gv.absolute_override=ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS; gf.next_action=true; break;}
                case 54: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G54);
                   {gv.coord_system=G54; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 55: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G55);
                   {gv.coord_system=G55; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 56: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G56);
                  {gv.coord_system=G56; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 57: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G57);
                   {gv.coord_system=G57; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 58: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G58);
                   {gv.coord_system=G58; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 59: 
                  //SET_MODAL (MODAL_GROUP_G12, coord_system, G59);
                   {gv.coord_system=G59; gf.coord_system=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                case 61: 
                  {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G13, path_control, PATH_EXACT_PATH);
                           {gv.path_control=PATH_EXACT_PATH; gf.path_control=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_G13, path_control, PATH_EXACT_STOP);
                           {gv.path_control=PATH_EXACT_STOP; gf.path_control=true; gp.modals[MODAL_GROUP_G12]=true; break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 64: 
                  //SET_MODAL (MODAL_GROUP_G13,path_control, PATH_CONTINUOUS);
                   {gv.path_control=PATH_CONTINUOUS; gf.path_control=true; gp.modals[MODAL_GROUP_G13]=true; break;}
                case 80: 
                  //SET_MODAL (MODAL_GROUP_G1, motion_mode,  MOTION_MODE_CANCEL_MOTION_MODE);
                   {gv.motion_mode=MOTION_MODE_CANCEL_MOTION_MODE; gf.motion_mode=true; gp.modals[MODAL_GROUP_G1]=true; break;}
                case 90: 
                  {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G3, distance_mode, ABSOLUTE_DISTANCE_MODE);
                           {gv.distance_mode=ABSOLUTE_DISTANCE_MODE; gf.distance_mode=true; gp.modals[MODAL_GROUP_G3]=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_G3, arc_distance_mode, ABSOLUTE_DISTANCE_MODE);
                           {gv.arc_distance_mode=ABSOLUTE_DISTANCE_MODE; gf.arc_distance_mode=true; gp.modals[MODAL_GROUP_G3]=true; break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 91: 
                  {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G3, distance_mode, INCREMENTAL_DISTANCE_MODE);
                        {gv.distance_mode=INCREMENTAL_DISTANCE_MODE; gf.distance_mode=true; gp.modals[MODAL_GROUP_G3]=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_G3, arc_distance_mode, INCREMENTAL_DISTANCE_MODE);
                          {gv.arc_distance_mode=INCREMENTAL_DISTANCE_MODE; gf.arc_distance_mode=true; gp.modals[MODAL_GROUP_G3]=true; break;}
 
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 92: 
                  {
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G92_OFFSETS);
                           {gv.next_action=NEXT_ACTION_SET_G92_OFFSETS; gf.next_action=true; gp.modals[MODAL_GROUP_G0]=true; break;}
                        case 1: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_RESET_G92_OFFSETS);
                          {gv.next_action= NEXT_ACTION_RESET_G92_OFFSETS; gf.next_action=true; break;}
                        case 2: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_SUSPEND_G92_OFFSETS);
                          {gv.next_action=NEXT_ACTION_SUSPEND_G92_OFFSETS; gf.next_action=true; break;}
                        case 3: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_RESUME_G92_OFFSETS);
                          {gv.next_action=NEXT_ACTION_RESUME_G92_OFFSETS; gf.next_action=true; break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                }
                case 93: 
                  //SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, INVERSE_TIME_MODE);
                     {gv.feed_rate_mode=INVERSE_TIME_MODE; gf.feed_rate_mode=true; gp.modals[MODAL_GROUP_G5]=true; break;}
                case 94: 
                  //SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, UNITS_PER_MINUTE_MODE);
                       {gv.feed_rate_mode=UNITS_PER_MINUTE_MODE; gf.feed_rate_mode=true; gp.modals[MODAL_GROUP_G5]=true; break;}

                default: 
                  status = STAT_GCODE_COMMAND_UNSUPPORTED;
            }
            break;

            case 'M':
            switch((uint8_t)value) 
            {
                case 0: case 1: case 60:
                   // SET_MODAL (MODAL_GROUP_M4, program_flow, PROGRAM_STOP);
                     {gv.program_flow=PROGRAM_STOP; gf.program_flow=true; gp.modals[MODAL_GROUP_M4]=true; break;}
                case 2: case 30:
                   //  SET_MODAL (MODAL_GROUP_M4, program_flow, PROGRAM_END);
                    {gv.program_flow=PROGRAM_END; gf.program_flow=true; gp.modals[MODAL_GROUP_M4]=true; break;}  
                case 3: 
                  //SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_CW);
                   gv.spindle_control=SPINDLE_RUNNING; 
                   gf.spindle_control=true; 
                   gp.modals[MODAL_GROUP_M7]=true;
                    
                   break; 
//SME Jan 5 2022: there is no CCW for Bantam tool CNC mills                   
//                case 4: 
//                  //SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_CCW);
//                   gv.spindle_control=SPINDLE_CCW; 
//                   gf.spindle_control=true; 
//                   gp.modals[MODAL_GROUP_M7]=true; 
//                   break;
                  
                case 5: 
                  //SET_MODAL (MODAL_GROUP_M7, spindle_control, SPINDLE_OFF);
                   gv.spindle_control=SPINDLE_OFF; 
                   gf.spindle_control=true; 
                   gp.modals[MODAL_GROUP_M7]=true; 
                   break;
                                     
                case 6: 
                  //SET_NON_MODAL (tool_change, true);                    
                   gv.tool_change= true, 
                   gf.tool_change= true;  
                 
                   break;
                    
                //case 7: 
                  //SET_MODAL (MODAL_GROUP_M8, coolant_mist,  COOLANT_ON);
                  //n/a {gv.coolant_mist=COOLANT_ON; gf.coolant_mist=true; gp.modals[MODAL_GROUP_M8]=true; break;}
                //case 8: 
                  //SET_MODAL (MODAL_GROUP_M8, coolant_flood, COOLANT_ON);
                //  {gv.coolant_flood=COOLANT_ON; gf.coolant_flood=true; gp.modals[MODAL_GROUP_M8]=true; break;}
                //  case 9: 
                  //SET_MODAL (MODAL_GROUP_M8, coolant_off,   COOLANT_OFF);
                //   gv.coolant_off=COOLANT_OFF; gf.coolant_off=true; gp.modals[MODAL_GROUP_M8]=true; break;}
                case 48: 
                  //SET_MODAL (MODAL_GROUP_M9, m48_enable, true);
                   {gv.m48_enable=true; gf.m48_enable=true; gp.modals[MODAL_GROUP_M9]=true; break;}
                case 49: 
                  //SET_MODAL (MODAL_GROUP_M9, m48_enable, false);
                  {gv.m48_enable=false; gf.m48_enable=true; gp.modals[MODAL_GROUP_M9]=true; break;}
                case 50:
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_MODAL (MODAL_GROUP_M9, fro_control, true);
                           {gv.fro_control=true; gf.fro_control=true; gp.modals[MODAL_GROUP_M9]=true; break;}
                        case 1: 
                          //SET_MODAL (MODAL_GROUP_M9, tro_control, true);
                           {gv.tro_control=true; gf.tro_control=true; gp.modals[MODAL_GROUP_M9]=true; break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;
                case 51: 
                      //SET_MODAL (MODAL_GROUP_M9, spo_control, true);
                      {gv.spo_control=true; 
                      gf.spo_control=true; 
                      gp.modals[MODAL_GROUP_M9]=true; 
                      break;}
                case 100:
                    switch (_point(value)) 
                    {
                        case 0: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_JSON_COMMAND_SYNC);
                          {gv.next_action=NEXT_ACTION_JSON_COMMAND_SYNC; 
                          gf.next_action=true;
                          break;}
                        case 1: 
                          //SET_NON_MODAL (next_action, NEXT_ACTION_JSON_COMMAND_ASYNC);
                          {gv.next_action=NEXT_ACTION_JSON_COMMAND_ASYNC; 
                          gf.next_action=true; 
                          break;}
                        default: 
                          status = STAT_GCODE_COMMAND_UNSUPPORTED;
                    }
                    break;

                case 101: 
                  //SET_NON_MODAL (next_action, NEXT_ACTION_JSON_WAIT);
                  {gv.next_action=NEXT_ACTION_JSON_WAIT; gf.next_action=true; break;}
                default: 
                  status = STAT_MCODE_COMMAND_UNSUPPORTED;
            }
            break;

            case 'T': 
              //SET_NON_MODAL (tool_select, (uint8_t)trunc(value));
              {gv.tool_select=(uint8_t)trunc(value); gf.tool_select=true; break;}
            case 'F': 
              //SET_NON_MODAL (F_word, value);
               {gv.F_word=value; gf.F_word=true; break;}
            case 'P': 
              //SET_NON_MODAL (P_word, value);                // used for dwell time, G10 coord select
              {gv.P_word=value; gf.P_word=true; break;}
            case 'S':             
              //SET_NON_MODAL (S_word, value);
                if (fp_ZERO(value))//sme: 10-26-2022 treat zero speed as an M5                
                {                
                    gv.spindle_control=SPINDLE_OFF; 
                    gf.spindle_control=true; 
                    gp.modals[MODAL_GROUP_M7]=true;                     
                }
           
                if (value <= 15000)
                {
                    __NOP();
                }
                gv.S_word=value; 
                gf.S_word=true;                 
               break;
            case 'X': 
		       //SET_NON_MODAL (target[AXIS_X], value);
                gv.target[AXIS_X]=value; gf.target[AXIS_X]=true; 
               break; 
            case 'Y':
                gv.target[AXIS_Y]=value; gf.target[AXIS_Y]=true;             
                break;                
            case 'Z': 
		        //SET_NON_MODAL (target[AXIS_Z], value);
                  gv.target[AXIS_Z]=value; gf.target[AXIS_Z]=true;                 
                  break;
             case 'A': 
                //SET_NON_MODAL (target[AXIS_A], value);'M':
                   gv.target[AXIS_A]=value; 
                   gf.target[AXIS_A]=true; 
                   break;         
            case 'H': 
              //SET_NON_MODAL (H_word, value);
               {gv.H_word=(int)value; gf.H_word=true; break;}
            case 'I': 
              //SET_NON_MODAL (arc_offset[0], value);
                {gv.arc_offset[0]=value; gf.arc_offset[0]=true; break;}
            case 'J': 
              //SET_NON_MODAL (arc_offset[1], value);
               {gv.arc_offset[1]=value; gf.arc_offset[1]=true; break;}
            case 'K': 
              //SET_NON_MODAL (arc_offset[2], value);
                {gv.arc_offset[2]=value; gf.arc_offset[2]=true; break;}
            case 'L': 
              //SET_NON_MODAL (L_word, value);
                {gv.L_word=(int)value; gf.L_word=true; break;}
            case 'R': 
              //SET_NON_MODAL (arc_radius, value);
                {gv.arc_radius=value; gf.arc_radius=true; break;}
            case 'N': 
              //SET_NON_MODAL (linenum, value_int);           // line number handled as special case to preserve integer value
               {gv.linenum=value_int; gf.linenum=true; break;}
          default: 
            status = STAT_GCODE_COMMAND_UNSUPPORTED;
        }
        if(status != STAT_OK) 
          break;
    }
    if ((status != STAT_OK) && (status != STAT_COMPLETE)) 
      return (status);
                               
    ritorno(_validate_gcode_block(active_comment));
    
    return (_execute_gcode_block(active_comment));        // if successful execute the block
}

/****************************************************************************************
 * _execute_gcode_block() - execute parsed block
 *
 *  Conditionally (based on whether a flag is set in gf) call the canonical machining
 *  functions in order of execution. Derived from RS274NGC_3 table 8:
 *
 *    0. record the line number
 *    1. comment (includes message) [handled during block normalization]
 *    1a. enable or disable overrides (M48, M49)
 *    1b. set feed override rate (M50)
 *    1c. set traverse override rate (M50.1)
 *    1d. set spindle override rate (M51)
 *    2. set feed rate mode (G93, G94 - inverse time or per minute)
 *    3. set feed rate (F)
 *    3a. Marlin functions (optional)
 *    3b. set feed override rate (M50.1)
 *    3c. set traverse override rate (M50.2)
 *    4. set spindle speed (S)
 *    5. select tool (T)
 *    6. change tool (M6)
 *    7. spindle on or off (M3, M4, M5)
 *    8. coolant on or off (M7, M8, M9)
 * // 9. enable or disable overrides (M48, M49, M50, M51) (see 1a)
 *    10. dwell (G4)
 *    11. set active plane (G17, G18, G19)
 *    12. set length units (G20, G21)
 *    13. cutter radius compensation on or off (G40, G41, G42)
 *    14. cutter length compensation on or off (G43, G49)
 *    15. coordinate system selection (G54, G55, G56, G57, G58, G59)
 *    16. set path control mode (G61, G61.1, G64)
 *    17. set distance mode (G90, G91)
 *    17a. set arc distance mode (G90.1, G91.1)
 *    18.  set retract mode (G98, G99)
 *    19a. homing functions (G28.2, G28.3, G28.1, G28, G30)
 *    19b. update system data (G10)
 *    19c. set axis offsets (G92, G92.1, G92.2, G92.3)
 *    20. perform motion (G0 to G3, G80-G89) as modified (possibly) by G53
 *    21. stop and end (M0, M1, M2, M30, M60)
 *
 *  Values in gv are in original units and should not be unit converted prior
 *  to calling the canonical functions (which do the unit conversions)
 */

stat_t _execute_gcode_block(char *active_comment)
{
    stat_t status = STAT_OK;

    if (gf.linenum) 
    {
        cm_set_model_linenum(gv.linenum);
    }

    EXEC_FUNC(cm_m48_enable, m48_enable);

    if (gf.fro_control) 
    {                                   // feedrate override
        ritorno(cm_fro_control(gv.P_word, gf.P_word));
    }
    if (gf.tro_control) 
    {                                   // traverse override
        ritorno(cm_tro_control(gv.P_word, gf.P_word));
    }
    if (gf.spo_control) 
    {                                   // spindle speed override
        ritorno(spindle_override_control(gv.P_word, gf.P_word));
    }

    EXEC_FUNC(cm_set_feed_rate_mode, feed_rate_mode);       // G93, G94
    EXEC_FUNC(cm_set_feed_rate, F_word); 
 
#define TRAP_MISSING_LINE_NUMBERS
#ifdef TRAP_MISSING_LINE_NUMBERS        
    if (gf.linenum) 
    {
        ritorno(cm_check_linenum());
    }
#else
    if (gf.linenum && gf.checksum) 
    {
        ritorno(cm_check_linenum());
    }
#endif
 
    EXEC_FUNC(spindle_speed_sync, S_word);     
    if (gf.spindle_control) 
    {                               // M3, M4, M5 (spindle OFF, CW, CCW)
        ritorno(spindle_control_sync((spControl)gv.spindle_control));
    }

    if (gv.next_action == NEXT_ACTION_DWELL) 
    {              // G4 - dwell
        ritorno(cm_dwell(gv.P_word));                       // return if error, otherwise complete the block
    }
    EXEC_FUNC(cm_select_plane, select_plane);               // G17, G18, G19
    EXEC_FUNC(cm_set_units_mode, units_mode);               // G20, G21
    //--> cutter radius compensation goes here

    switch (gv.next_action) 
    {                               // Tool length offsets
        case NEXT_ACTION_SET_TL_OFFSET: 
          {                   // G43
            ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, false));
            break;
        }
        case NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET: 
          {        // G43.2
            ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, true));
            break;
        }
        case NEXT_ACTION_CANCEL_TL_OFFSET: 
          {                // G49
            ritorno(cm_cancel_tl_offset());
            break;
        }
        default: {} // quiet the compiler warning about all the things we don't handle here
    }

    EXEC_FUNC(cm_set_coord_system, coord_system);           // G54, G55, G56, G57, G58, G59

    if (gf.path_control) 
    {                                  // G61, G61.1, G64
        status = cm_set_path_control(MODEL, gv.path_control);
    }

    EXEC_FUNC(cm_set_distance_mode, distance_mode);         // G90, G91
    EXEC_FUNC(cm_set_arc_distance_mode, arc_distance_mode); // G90.1, G91.1
    //--> set retract mode goes here

    switch (gv.next_action) 
    {
        case NEXT_ACTION_SET_G28_POSITION: // G28.1 
          { 
            status = cm_set_g28_position(); 
            break;
          }                               
        case NEXT_ACTION_GOTO_G28_POSITION:// G28 
          { 
            status = cm_goto_g28_position(gv.target, gf.target); 
            break;
          }          
        case NEXT_ACTION_SET_G30_POSITION:// G30.1  
          { 
            status = cm_set_g30_position(); 
            break;
          }                               
        case NEXT_ACTION_GOTO_G30_POSITION:// G30 
          { 
            status = cm_goto_g30_position(gv.target, gf.target); 
            break;
          }          

        case NEXT_ACTION_SEARCH_HOME:// G28.2         
          { 
            status = cm_homing_cycle_start(gv.target, gf.target); 
            break;
          }       
        case NEXT_ACTION_SET_ABSOLUTE_ORIGIN: // G28.3 
          { 
            status = cm_set_absolute_origin(gv.target, gf.target); 
            break;
          }     
        case NEXT_ACTION_HOMING_NO_SET:// G28.4       
          { 
            status = cm_homing_cycle_start_no_set(gv.target, gf.target); 
            break;
          }

        case NEXT_ACTION_STRAIGHT_PROBE_ERR: // G38.2    
          { status = cm_straight_probe(gv.target, gf.target, true, true); 
            break;
          }  
        case NEXT_ACTION_STRAIGHT_PROBE: // G38.3         
          { status = cm_straight_probe(gv.target, gf.target, true, false);
            break;
          }
        case NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR:// G38.4
          { status = cm_straight_probe(gv.target, gf.target, false, true);
            break;
          } 
        case NEXT_ACTION_STRAIGHT_PROBE_AWAY: // G38.5  
          { status = cm_straight_probe(gv.target, gf.target, false, false);
            break;
          }

        case NEXT_ACTION_SET_G10_DATA: // G10          
          {  status = cm_set_g10_data((uint8_t)gv.P_word, gf.P_word,               
             gv.L_word, gf.L_word,
             gv.target, gf.target);
             break;
          }

        case NEXT_ACTION_SET_G92_OFFSETS:// G92     
          { 
            status = cm_set_g92_offsets(gv.target, gf.target); 
            break;
          }      
        case NEXT_ACTION_RESET_G92_OFFSETS:   
          { 
            status = cm_reset_g92_offsets();// G92.1 
            break;
          }                        
        case NEXT_ACTION_SUSPEND_G92_OFFSETS: 
          { 
            status = cm_suspend_g92_offsets();// G92.2 
            break;
          }                      
        case NEXT_ACTION_RESUME_G92_OFFSETS: // G92.3 
          { 
            status = cm_resume_g92_offsets(); 
            break;
          }                       

        case NEXT_ACTION_JSON_COMMAND_SYNC:       
          { // M100.0
 
            status = cm_json_command(active_comment); 
            break; 
          }   
        case NEXT_ACTION_JSON_COMMAND_ASYNC:      
          {  // M100.1
             status = cm_json_command_immediate(active_comment); 
             break;
          }     
        case NEXT_ACTION_JSON_WAIT:               
          { 
            status = cm_json_wait(active_comment); 
            break;
          }                  // M101
 // Done differently elsewhere      case NEXT_ACTION_ATC_SEARCH_HOME:
 //         {
 //           cm_change_tool()
 //           break;
 //         }
            
        case NEXT_ACTION_DEFAULT:
          {
            cm_set_absolute_override(MODEL, gv.absolute_override); // apply absolute override & display as absolute
            switch (gv.motion_mode) 
            {
                case MOTION_MODE_CANCEL_MOTION_MODE: // G80 
                  { 
                    cm->gm.motion_mode = gv.motion_mode; 
                    break;
                  }                
                case MOTION_MODE_STRAIGHT_TRAVERSE:// G0  
                  { status = cm_straight_traverse(gv.target, gf.target, PROFILE_NORMAL); 
                    break;
                  } 
                case MOTION_MODE_STRAIGHT_FEED: // G1      
                  { status = cm_straight_feed(gv.target, gf.target, PROFILE_NORMAL); 
                    break;
                  }    
                case MOTION_MODE_CW_ARC:                                                                            // G2
                case MOTION_MODE_CCW_ARC: // G3
                  { status = cm_arc_feed(gv.target,gf.target,                          
                                      gv.arc_offset, gf.arc_offset,
                                      gv.arc_radius, gf.arc_radius,
                                      gv.P_word,     gf.P_word,
                                      gp.modals[MODAL_GROUP_G1],
                                      gv.motion_mode);
                    break;
                  }
                default: break;
            }
            cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF);  // un-set absolute override once the move is planned
        }
        default:
            // quiet the compiler warning about all the things we don't handle
            break;
    }
 
    // do the program stops and ends : M0, M1, M2, M30, M60
    if (gf.program_flow == true) 
    {
        if (gv.program_flow == PROGRAM_STOP) 
        {
            cm_program_stop();
        } 
        else 
        {
            cm_program_end();
        }
    }
    return (status);
}


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

stat_t gc_get_gc(nvObj_t *nv)
{
    ritorno(nv_copy_string(nv, cs.saved_buf));
    nv->valuetype = TYPE_STRING;
    return (STAT_OK);
}

stat_t gc_run_gc(nvObj_t *nv)
{
    return(gcode_parser(*nv->stringp));
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

// no text mode functions here. Move along

#endif // __TEXT_MODE
