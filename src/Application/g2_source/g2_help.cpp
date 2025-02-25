/***************************************************************
 * help.cpp - collected help routines
 * This file is part of the TinyG2 project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
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
 ***************************************************************/
#include "main.h"  
#include "bantam_hal.h"
#include "g2core.h"		// #1
#include "g2_config.h"		// #2
#include "g2_report.h"
#include "g2_help.h"
 
stat_t help_stub(nvObj_t* nv) { return (STAT_OK); }


static void _status_report_advisory() {
    comms_mgr_write_msg((char *)
        "\n\
Note: g2core generates automatic status reports by default\n\
This can be disabled by entering $sv=0\n\
See the wiki below for more details.\n\
");
}

static void _postscript() {
    comms_mgr_write_msg((char *)
        "\n\
For detailed g2core info see: https://github.com/synthetos/g2/wiki\n\
For the latest firmware see: https://github.com/synthetos/g2\n\
Please log any issues at https://github.com/synthetos/g2/issues\n\
Have fun\n");
}

/*
 * help_general() - help invoked as h from the command line
 */
uint8_t help_general(nvObj_t* nv) {
    comms_mgr_write_msg((char *)"\n\n\n### g2core Help ###\n");
    comms_mgr_write_msg((char *)
        "\n\
These commands are active from the command line:\n\
 ^x             Reset (control x) - software reset\n\
  ?             Machine position and gcode model state\n\
  $             Show and set configuration settings\n\
  !             Feedhold - stop motion without losing position\n\
  ~             Cycle Start - restart from feedhold\n\
  h             Show this help screen\n\
  $h            Show configuration help screen\n\
  $test         List self-tests\n\
  $test=N       Run self-test N\n\
  $home=1       Run a homing cycle\n\
  $defa=1       Restore all settings to \"factory\" defaults\n\
");
    _status_report_advisory();
    _postscript();
    rpt_print_system_ready_message();
    return (STAT_OK);
}

/*
 * help_config() - help invoked as $h
 */
stat_t help_config(nvObj_t* nv) {
    comms_mgr_write_msg((char *)"\n\n\n### g2core CONFIGURATION Help ###\n");
    comms_mgr_write_msg((char *)
        "\n\
These commands are active for configuration:\n\
  $sys Show system (general) settings\n\
  $1   Show motor 1 settings (or whatever motor you want 1,2,3,4)\n\
  $x   Show X axis settings (or whatever axis you want x,y,z,a,b,c)\n\
  $m   Show all motor settings\n\
  $q   Show all axis settings\n\
  $o   Show all offset settings\n\
  $$   Show all settings\n\
  $h   Show this help screen\n\n\
");
    comms_mgr_write_msg((char *)
        "\n\
Each $ command above also displays the token for each setting in [ ] brackets\n\
To view settings enter a token:\n\n\
  $<token>\n\n\
For example $yfr to display the Y max feed rate\n\n\
To update settings enter token equals value:\n\n\
  $<token>=<value>\n\n\
For example $yfr=800 to set the Y max feed rate to 800 mm/minute\n\
For configuration details see: https://github.com/synthetos/g2/wiki/g2-Configuration\n\
");
    _status_report_advisory();
    _postscript();
    return (STAT_OK);
}

/*
 * help_defa() - help invoked for defaults
 */
stat_t help_defa(nvObj_t* nv) {
    comms_mgr_write_msg((char *)"\n\n\n### g2core RESTORE DEFAULTS Help ###\n");
    comms_mgr_write_msg((char *)
        "\n\
Enter $defa=1 to reset the system to the factory default values.\n\
This will overwrite any changes you have made.\n");
    _postscript();
    return (STAT_OK);
}

/*
 * help_flash()
 */
stat_t help_flash(nvObj_t* nv) {
    comms_mgr_write_msg((char *)"\n\n\n### g2core FLASH LOADER Help ###\n");
    comms_mgr_write_msg((char *)
        "\n\
Enter $flash=1 to enter the flash loader.\n");
    _postscript();
    return (STAT_OK);
}

/***********************************************************************************
 *   help_boot_loader()  
 * 
 * 	
 ***********************************************************************************/
stat_t help_boot_loader(nvObj_t *nv)
{
   comms_mgr_write_msg((char *)"\n\n\n### TinyG BOOT LOADER Help ###\n");   
   comms_mgr_write_msg((char *)"\nEnter $boot=1 to enter the boot loader.\n");
                                                      
   _postscript();
   return(STAT_OK);
}/* End Function */
/***********************************************************************************
 *   help_boot_loader()  
 * 
 * 	
 ***********************************************************************************/
stat_t help_dfuboot_loader(nvObj_t *nv)
{   
   comms_mgr_write_msg((char *)"\n\n\n### DFU BOOT LOADER Help ###\n");   
   comms_mgr_write_msg((char *)"\
   Enter $dfuboot=1 to enter the dfu bootloader.\n");
                                                      
   _postscript();
   return(STAT_OK);
}/* End Function */
 /***********************************************************************************
 *   help_hw_reset()  
 * 
 * 	
 ***********************************************************************************/
stat_t help_hw_reset(nvObj_t *nv)
{
   fprintf_P( STDERR_SUBSTITUTE, PSTR("\n\n\n### HW Reset Help ###\n"));
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
   
   fprintf_P( STDERR_SUBSTITUTE, PSTR("\
   Enter $hw_reset to reset the hardware .\n"));
   comms_mgr_write_msg(STDERR_SUBSTITUTE);
                                                      
   _postscript();
   return(STAT_OK);
}/* End Function */   



