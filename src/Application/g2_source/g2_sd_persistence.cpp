/*
 * g2_sd_persistence.cpp - persistence functions for SD cards
 * This file is part of the g2core project
 *
 * Copyright (c) 2019 Matt Staniszewski
 * Copyright (c) 2019 Robert Giseburt
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

#include "diskio.h" //disk_status is defined 
#include "ff.h"
#include "g2core.h"
#include "g2_persistence.h"
#include "g2_canonical_machine.h"
#include "g2_report.h"
#include "g2_util.h"
#include "g2_config.h"  // needed for nvObj_t definition
#include "system_bantam.h"//systick 
#include "g2_sd_persistence.h"  // needed for nvObj_t definition
#include "app.h"

/* When sd card is installed, the signal is low.*/
enum {SD_IS_ATTACHED, SD_IS_DETACHED} sd_card;
bool sd_card_attached(void)
{
    volatile int status=MICROSD_CD_Get();
    bool result=false;
    if (status==SD_IS_ATTACHED)
    {
        result=true;
    }
    return result;
}

#define f_is_open(fp) ((fp)->fs)
#define NVM_VALUE_LEN 4             // NVM value length (float, fixed length)
#define NVM_BASE_ADDR 0x0000        // base address of usable NVM
#define IO_BUFFER_SIZE 512          // this should be evenly divisible by NVM_VALUE_LEN, and <=512 until multi-block reads are fixed (right now they are hanging...)
#define MIN_WRITE_INTERVAL 1000     // minimum interval between persistence file writes
#define MAX_WRITE_FAILURES 3
#define MAX_WRITE_CHANGES IO_BUFFER_SIZE    // maximum number of write values that change - ms: TODO

/***********************************************************************************
 **** STRUCTURE ALLOCATIONS ********************************************************
 ***********************************************************************************/

//**** persistence singleton ****

struct nvmSingleton_t 
{
    float tmp_value;
    FATFS fat_fs;
    FIL file;
    uint8_t file_index;
    uint8_t io_buffer[IO_BUFFER_SIZE];
    uint16_t changed_nvs;
    uint32_t last_write_systick;
    uint8_t write_failures;
} nvm;
 
/***********************************************************************************
 **** GENERIC STATIC FUNCTIONS AND VARIABLES ***************************************
 ***********************************************************************************/
uint16_t get_changed_nvs_count(void){return nvm.changed_nvs;}
stat_t prepare_persistence_file();
stat_t write_persistent_values();
stat_t validate_persistence_file();
uint8_t active_file_index();

// Leaving this in for now in case bugs come up; we can remove it when we're confident
// it's stable
#if 0
# define DEBUG_PRINT(args...) printf(args);
#else
# define DEBUG_PRINT(...) do {} while (0)
#endif

// Analogous to ritorno(a), but for the FatFS return codes and with optional debug output
#define fs_ritorno(a, msg) if((status_code=a) != FR_OK) \
    { DEBUG_PRINT("%s res: %i\n", msg, status_code); \
    return(STAT_PERSISTENCE_ERROR); }

/*
 We cycle between three different files, indexed by a suffix. Each time we need to write
 new values, we copy data from the current file to a new file with NEXT_FILE_INDEX, then
 delete the current file once the write is complete. This ensures that at least one recent
 copy of the file will be preserved if power is lost in the middle of a write.
 */
#define PERSISTENCE_DIR "persist"
#define PERSISTENCE_FILENAME(num) PERSISTENCE_DIR"/persist"#num".bin"
#define PERSISTENCE_FILENAME_CNT 3
#define NEXT_FILE_INDEX (nvm.file_index+1) % PERSISTENCE_FILENAME_CNT
#define PREV_FILE_INDEX (nvm.file_index+PERSISTENCE_FILENAME_CNT-1) % PERSISTENCE_FILENAME_CNT
const char* filenames[PERSISTENCE_FILENAME_CNT] =
{ 
    PERSISTENCE_FILENAME(0), PERSISTENCE_FILENAME(1), PERSISTENCE_FILENAME(2) 
};
#define CRC_LEN 4

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/

class SD_Persistence : public Persistence 
{
   public:
    void init() override;
    stat_t read(nvObj_t *nv) override;
    stat_t write(nvObj_t *nv) override;
    stat_t periodic() override;
    stat_t read_tuid_data(void);
};

SD_Persistence sdp {};

void setup_sd_persistence() 
{
    persistence = &sdp;
}

void SD_Persistence::init()
{
    nvm.file_index = 0;
    nvm.last_write_systick =sys_get_time_ms();
    nvm.write_failures = 0;
    nvm.changed_nvs = 0;
    return;
}
 
/*
 * read_tuid_data	 
 *
 *	tuids data are eight, 34byte strings that are saved as a contiguous group at the
 *      end of the persistence and are retrieved as a group and assigned individually during the
        read  
 */
stat_t SD_Persistence::read_tuid_data(void)
{
  return STAT_OK;
}
 
/*
 * read_persistent_value()	- return value (as float) by index
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */

stat_t SD_Persistence::read(nvObj_t *nv)
{
    FILINFO file_info;  
    ritorno(prepare_persistence_file());
    DEBUG_PRINT("file opened for reading\n");
    //file_info
    fs_ritorno(f_lseek(&nvm.file, nv->index * NVM_VALUE_LEN), "f_lseek during read");
    UINT br;
    fs_ritorno(f_read(&nvm.file, &nvm.io_buffer, NVM_VALUE_LEN, &br), "read value");
    if (br != NVM_VALUE_LEN) 
    {
        return (STAT_PERSISTENCE_ERROR);
    }

    auto type = cfgArray[nv->index].flags & F_TYPE_MASK;
    if ((type == TYPE_INTEGER) || (type == TYPE_DATA)) 
    {
        nv->valuetype = TYPE_INTEGER;
        nv->value_int = *(int32_t *)nvm.io_buffer;
        DEBUG_PRINT("value (i) copied from address %li in file: %li\n", nv->index * NVM_VALUE_LEN, nv->value_int);
    } 
    else if (type == TYPE_BOOLEAN) 
    {
        nv->valuetype = TYPE_BOOLEAN;
        nv->value_int = *(int32_t *)nvm.io_buffer;
        DEBUG_PRINT("value (b) copied from address %li in file: %li\n", nv->index * NVM_VALUE_LEN, nv->value_int);
    } 
    else if (type ==TYPE_FLOAT)
    {
        nv->valuetype = TYPE_FLOAT;
        nv->value_flt = *(float *)nvm.io_buffer;
        DEBUG_PRINT("value (f) copied from address %li in file: %f\n", nv->index * NVM_VALUE_LEN, nv->value_flt);
    }
    return (STAT_OK);
}

/*
 * write_persistent_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 *	Note: Removed NAN and INF checks on floats - not needed
 */

stat_t SD_Persistence::write(nvObj_t *nv)
{
    nvm.changed_nvs++;
    return (STAT_OK);
}

/*
 * write_persistent_values_callback()
 *
 * On ARM, write cached values to a file. No-op on AVR.
 */

stat_t SD_Persistence::periodic()
{
  static volatile DSTATUS diskStatus=STA_NOINIT;
  static volatile bool report_disk_fault_flag=false;
  
#ifdef FILTER_SPURIOUS_SD_CARD_FAULTS  
  static uint32_t disk_status_check_count=0;//sme: 1-26-2022. learn how to filter out transient spurious disk not present faults
#endif 
   // Check the disk status to ensure we catch card-detect pin changes.
 //sme: 01-03-2023 use the actual pin state
  if (sd_card_attached()!=true)
  {
      /* send report only once*/
     if (report_disk_fault_flag!=true)
     {
#ifdef FILTER_SPURIOUS_SD_CARD_FAULTS       
#define SD_CARD_FAULT_PERSISTENCE_COUNT 3      
          if(disk_status_check_count++ >SD_CARD_FAULT_PERSISTENCE_COUNT)
          {
           disk_status_check_count=0;         
           report_disk_fault_flag=true;
           rpt_exception(STAT_NO_MEDIUM_IN_DRIVE,(char*)"SD Card Not Detected");
          }
#else //original
          report_disk_fault_flag=true;
          rpt_exception(STAT_NO_MEDIUM_IN_DRIVE,(char*)"SD Card Not Detected");
       }
#endif       
     } 
     return STAT_NO_MEDIUM_IN_DRIVE;
   }
   else
   { 
     if(report_disk_fault_flag==true)
     {
        report_disk_fault_flag=false;
        disk_status_check_count=0;
     }
     
     if (nvm.changed_nvs > 0) 
     {
         uint32_t time_ms=sys_get_time_ms()-nvm.last_write_systick;
         if (time_ms < MIN_WRITE_INTERVAL) 
         {
             return (STAT_NOOP);
         }
         // this check may not be necessary on ARM, but just in case...
         if (cm->cycle_type != CYCLE_NONE) 
         {
             return(STAT_NOOP);    // can't write when machine is moving
         }

         if(write_persistent_values() == STAT_OK) 
         {
             nvm.changed_nvs = 0;
             nvm.write_failures = 0;
         } 
         else 
         {
             // if the write failed, make sure no half-written output file exists
             f_unlink(filenames[NEXT_FILE_INDEX]);
             if (++nvm.write_failures >= MAX_WRITE_FAILURES) 
             {
                 nvm.changed_nvs = 0;     // give up on these values
                 nvm.write_failures = 0;  // but try again if we get more values later
                 return(rpt_exception(STAT_PERSISTENCE_ERROR, (char*)"write failure"));
             }
         }
         nvm.last_write_systick = sys_get_time_ms();
         return STAT_OK;
     }
   }
 
   return STAT_NOOP;
}

/*
 * active_file_index()
 *
 * Determines which of the existing files is most current and should be used for
 *  value reads. If no files exist, return 0. This assumes that no more than two
 *  files exist at any one time, which should always be the case under our updating
 *  scheme (described in a comment near the PERSISTENCE_FILENAME define).
 */
uint8_t active_file_index()
{   
   uint8_t i = 0;
   for (; i < PERSISTENCE_FILENAME_CNT; ++i) 
   {
       if (f_stat(filenames[i], nullptr) == FR_OK) 
       {
           uint8_t next = (i+1)%PERSISTENCE_FILENAME_CNT;
           if (next > i && f_stat(filenames[next], nullptr) == FR_OK) 
           {
               i = next;
           }
           break;
       }
   }
   return i;
}

/*
 * prepare_persistence_file()
 *
 * ARM only. Ensures that the file is open and has a valid CRC. This should be called
 *  prior to using the file in any other function.
 */
stat_t prepare_persistence_file()
{
  volatile stat_t status;
  uint8_t index ;
#ifdef DEBUG_SDCARD_BRICK_EFFECT
  char *fname_ptr;  
#endif
  FATFS *fs;//throw-away in this usage of validate
   // if the file is already open and valid, no further prep is necessary.
   // NOTE: we don't close the file after every use because the higher latency
   // would slow down consecutive reads. However, we still need to re-validate before
   // every use to ensure that the card status hasn't changed.
     if (f_is_open(&nvm.file.obj)          
     && validate(&nvm.file.obj,&fs) == FR_OK)
     {
       return STAT_OK;
     }

   // mount volume if necessary
   if (!nvm.fat_fs.fs_type) 
   {
       fs_ritorno(f_mount(&nvm.fat_fs, "", 1), "mount");       /* Give a work area to the default drive */
   }
   f_mkdir(PERSISTENCE_DIR);
   index = active_file_index();
#ifdef DEBUG_SDCARD_BRICK_EFFECT      
   if (index>=PERSISTENCE_FILENAME_CNT) 
   {
      index=PERSISTENCE_FILENAME_CNT-1;
   }
   fname_ptr=(char *)filenames[index];
   fs_ritorno(f_open(&nvm.file,(const char*) fname_ptr, FA_READ | FA_OPEN_EXISTING), "open input");
#else  
   fs_ritorno(f_open(&nvm.file, filenames[index], FA_READ | FA_OPEN_EXISTING), "open input");
#endif
    nvm.file_index = index;

   // if CRC doesn't match, delete file and return error
   status=validate_persistence_file();
   if (status != STAT_OK) 
   {
       f_close(&nvm.file);
       f_unlink(filenames[nvm.file_index]);
       nvm.file_index = 0;
       return STAT_PERSISTENCE_ERROR;
   }
   // OK to delete old file now (if it still exists), since we know the current one is good
   f_unlink(filenames[PREV_FILE_INDEX]);
   return STAT_OK;
}

/*
 * validate_persistence_file()
 *
 * ARM only. Helper function that checks the CRC and byte count of the
 *  persistence file. Assumes the file is already open.
 */
stat_t validate_persistence_file()
{
   uint32_t crc = 0;
   uint32_t filecrc = -1;
   UINT br, br_sum = 0;
   fs_ritorno(f_lseek(&nvm.file, 0), "crc check seek");
#ifdef DEBUG_SDCARD_BRICK_EFFECT
#define MAX_OBSERVED_PERSISTENCE_FILE_SIZE 3000 
#define EXTRA_PADDING_MULTIPLIER 2
#define MAX_FREADS_PER_IO_BUFFER_SIZE (((MAX_OBSERVED_PERSISTENCE_FILE_SIZE*EXTRA_PADDING_MULTIPLIER)/IO_BUFFER_SIZE)+1)
   volatile int max_while_loop_iterations=MAX_FREADS_PER_IO_BUFFER_SIZE;
   volatile int while_loop_interation_counter=0;
#endif   
   while (!f_eof(&nvm.file)) 
   {
       fs_ritorno(f_read(&nvm.file, &nvm.io_buffer, IO_BUFFER_SIZE, &br), "file read during CRC check");

       if (f_eof(&nvm.file)) 
       {
           br -= std::min((UINT)CRC_LEN, br); // don't include old CRC in current CRC calculation
           memcpy(&filecrc, nvm.io_buffer+br, CRC_LEN); // copy old CRC out of read buffer
       }
       // update calculated CRC
       crc = crc32(crc, nvm.io_buffer, br);
       br_sum += br;
#ifdef DEBUG_SDCARD_BRICK_EFFECT
       if (while_loop_interation_counter++ >=while_loop_interation_counter)
       {           
           rpt_exception( STAT_PERSISTENCE_FILE_TOO_LARGE,(char*)"Persistence file suspiciously too large, exiting CRC check");
           __NOP();           
           break;
       }
#endif        
   }

   UINT file_size=nv_index_max() * NVM_VALUE_LEN;
   UINT strings_size=0; 
   UINT file_size_with_strings=file_size+strings_size;
   if (br_sum !=file_size_with_strings)
   {
       DEBUG_PRINT("bad byte count in file: %i\n", br_sum);
       return STAT_PERSISTENCE_ERROR;
   }
   DEBUG_PRINT("crc: %lu from file, %lu calculated\n", filecrc, crc); 
   return crc == filecrc ? STAT_OK : STAT_PERSISTENCE_ERROR;
}

/*
 * write_persistent_values()
 *
 * ARM only. Writes all the values from the write cache to the SD card. Since we can't
 *  rewrite individual pieces of data in the middle of an existing file, this requires
 *  rewriting all the data into a new file.
 */
stat_t write_persistent_values()
{
   uint16_t io_byte_count=0;
   static int debug_ndx=555;
   FILINFO file_info;
   volatile static int file_index, fsize1, fsize2=0;
   DEBUG_PRINT("writing new version\n");

   FIL f_out;
   UINT bw;
   nvObj_t *nv = nv_reset_nv_list();  // sets *nv to the start of the body
   cmDistanceMode saved_distance_mode;

#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME
   uint32_t start_time_ms = sys_get_time_ms();
   uint32_t nvm_access_start_time_ms,file_write_star_time_ms;
   static uint16_t nvm_access_time_ms, file_write_time_ms,delta_persistence_time_ms;
#endif

   // Save the current units mode
   saved_distance_mode  = (cmDistanceMode)cm_get_distance_mode(ACTIVE_MODEL);

   // attempt to open file with previously persisted values
   if (prepare_persistence_file() == STAT_OK) 
   {
       fs_ritorno(f_lseek(&nvm.file, 0), "f_lseek to input file start");
   }

   // open new file for writing updated values
   fs_ritorno(f_open(&f_out, filenames[NEXT_FILE_INDEX], FA_WRITE | FA_OPEN_ALWAYS), "open output");
   fs_ritorno(f_sync(&f_out), "sync output file");
   DEBUG_PRINT("opened %s for writing\n", filenames[NEXT_FILE_INDEX]);
   file_index=active_file_index(); 
   uint32_t crc = 0;
   uint16_t step = IO_BUFFER_SIZE/NVM_VALUE_LEN;

   for (index_t cnt = 0; cnt < nv_index_max(); cnt += step) 
   {
       // try to read old values from existing file
       io_byte_count = (uint16_t)std::min((float)IO_BUFFER_SIZE, (float)((nv_index_max()-cnt) * NVM_VALUE_LEN));
       UINT br = 0;
       f_read(&nvm.file, &nvm.io_buffer, io_byte_count, &br);
       DEBUG_PRINT("read %i bytes from old file\n", br);

       // if we didn't get enough bytes from the old file, pad the buffer with defaults
       // to keep the length correct
       // FIXME: integrate this with default-setting code in config.cpp
       for (; br<io_byte_count; br += NVM_VALUE_LEN) 
       {
           index_t index = cnt + br/NVM_VALUE_LEN;
           memcpy(nvm.io_buffer+br, &cfgArray[index].def_value, NVM_VALUE_LEN);
       }
       DEBUG_PRINT("io_buffer populated with %i bytes total\n", br);

      // update the values in the buffer from the changes indexes in the current range
      for (nv->index = cnt; nv->index < std::min(nv_index_max(), cnt + step); nv->index++) 
      {
        // Found an entry that needs to be written
        if (nv->index == 0 || cfgArray[nv->index].flags & F_PERSIST) 
        {
#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME          
   f        nvm_access_start_time_ms = sys_get_time_ms(); 
#endif
           if (nv->index==debug_ndx)
           {
             __NOP();//__no_operation();
           }
          nv_get_nvObj(nv);

          // Get the index for the current NVM value
          index_t index = (nv->index - cnt) * NVM_VALUE_LEN;

          // Write out based on the value type
          if (nv->valuetype == TYPE_INTEGER || nv->valuetype == TYPE_BOOLEAN || nv->valuetype == TYPE_DATA) 
          {
              memcpy(nvm.io_buffer + index, &nv->value_int, NVM_VALUE_LEN);
              DEBUG_PRINT("item index: %li , write index: %li (cnt: %li), value: %li\n", nv->index, index, cnt,
                        nv->value_int);
          } 
          else if (nv->valuetype == TYPE_FLOAT) 
          {
              float value_flt = nv->value_flt;
              memcpy(nvm.io_buffer + index, &value_flt, NVM_VALUE_LEN);
              DEBUG_PRINT("item index: %li , write index: %li (cnt: %li), value: %f\n", nv->index, index, cnt, nv->value_flt);
          } 
          else 
          {              
              // next - ignore strings and other stuff which shouldn't be set to persist anyway
              //10-27-2021 sme: we shall not ignore strings that represent hex digit tool uuid's
          
          }//end else
#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME          
          nvm_access_time_ms=sys_get_delta_time_ms( nvm_access_start_time_ms);
#endif          
        }//end if persistent
      }//end for --writing new file
        // write updated values to output file and sync
      
#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME 
       file_write_star_time_ms =sys_get_time_ms();  
#endif     
       fs_ritorno(f_write(&f_out, &nvm.io_buffer, io_byte_count, &bw), "new file write");
       
#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME          
       file_write_time_ms=sys_get_delta_time_ms(file_write_star_time_ms);    
#endif
       
       if (bw != io_byte_count) 
         return (STAT_PERSISTENCE_ERROR);
       fs_ritorno(f_sync(&f_out), "out sync");

       // update CRC
       crc = crc32(crc, nvm.io_buffer, io_byte_count);
   }//end for --reading old file


   // write CRC in final 4 bytes
   fs_ritorno(f_write(&f_out, &crc, CRC_LEN, &bw), "write crc");
   DEBUG_PRINT("wrote crc: %lu\n", crc);

   // close both old and new files
   fs_ritorno(f_close(&f_out), "close output");
   if (f_is_open(&nvm.file.obj)) 
   {
       fs_ritorno(f_close(&nvm.file), "close input");
       // if we made it here, it's now safe to delete the older file
       fs_ritorno(f_unlink(filenames[nvm.file_index]), "old file delete");
       DEBUG_PRINT("deleted obsolete file %s\n", filenames[nvm.file_index]);
       nvm.file_index = 0;
   }
#ifdef TRACK_PERSISTENCE_FILE_ACCESS_TIME
     delta_persistence_time_ms=sys_get_delta_time_ms(start_time_ms);
#endif
   // Restore units mode
   cm_set_distance_mode(saved_distance_mode);

   return (STAT_OK);
}
 