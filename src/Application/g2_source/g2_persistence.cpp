
/*
 * persistence.cpp - persistence functions
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart Jr.
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
#include "g2core.h"
#include "g2_persistence.h"
#include "g2_sd_persistence.h"//uses get_changed_nvs_count()
#include "g2_stepper.h"// uses: st_runtime_isbusy()

/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/

Persistence *persistence = nullptr;

void persistence_init()
{
    if (persistence == nullptr) 
    {
        return;
    }
    persistence->init();
}

/*
 * read_persistent_value()	- return value (as float) by index
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */

stat_t read_persistent_value(nvObj_t *nv)
{
  stat_t status;
    if (persistence == nullptr) 
    {
        return (STAT_OK);  // it worked! ;-)
    }
    status=persistence->read(nv);
    return status;
}

/*
 * write_persistent_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 *	Note: Removed NAN and INF checks on floats - not needed
 */

stat_t write_persistent_value(nvObj_t *nv)
{
  stat_t status = STAT_OK;
    if (persistence == nullptr) 
    {
        return (STAT_OK);  // it worked! ;-)
    }
     status = persistence->write(nv);
    return status;
}

/*
 * write_persistent_values_callback()
 */
 
stat_t write_persistent_values_callback(void)
{
  static uint32_t nvm_changed_count_copy=0;
  uint16_t nvm_changed_count=get_changed_nvs_count();

    if (persistence == nullptr) {
        return (STAT_OK);  // it worked! ;-)
    }

  if (st_runtime_isbusy() ==true)
  {
     if (nvm_changed_count > nvm_changed_count_copy) 
     {
         nvm_changed_count_copy=nvm_changed_count;
     }
//    return (STAT_OK);
  }

    return persistence->periodic();

}

