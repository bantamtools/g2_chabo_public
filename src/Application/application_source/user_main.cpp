/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h" 
#include "bantam_hal.h"
      
/* USER CODE BEGIN Includes */
#include "system_bantam.h"   
#include "comms_mgr.h"   
#include "sd_card_file_mgr.h"
#include "spi_comm.h"
#include "spindle_ctrl.h"
#include "motion_ctrl.h"
#include "main_controller.h"
#include "g2_text_parser.h"//prints a2d converted spindle thermister readings
#include "kernel.h"     
#include "time.h" 
#include "g2_sd_persistence.h"
#include "app.h"

#ifdef DEPLOY_MCU_WD_TIMER
#include "stm32f7xx_hal_iwdg.h"
#endif
/* USER CODE END Includes */

/**************************************************************************
 * 
 * Function: main
 *
 *
 **************************************************************************/
int user_main(void)
{
   volatile bool sd_card_detected_flag=false;

  /* MCU Configuration----------------------------------------------------------*/
    // Initialize GPIOs, UART, steppers, and LEDs
    gpio_init();
    uart_init();
#ifdef CHABO_USE_MATTS_TMC2660_INIT
    stepper_init_matt();
#endif
    pwm_led_device_init();
    check_and_update_chabo_pid();
    
  /* Turn on leds */  
  hal_write_pin(0,GPIO_DBG1_PIN,GPIO_PIN_SET);
  hal_write_pin(0,GPIO_DBG0_PIN,GPIO_PIN_RESET);
  comms_mgr_init();//must occur prior to g2_main inits 
  motion_ctrl_init();
  spindle_enable_power();//energizes relay for 24V power supply to spindle and ESC
  spindle_startup_pwm();//applies idle level, 10% duty cycle, required to turn on esc driver

  /* 9-8-2022:
   * start_systime_interrupt needs to start here to have a get_syst_time_ms() tp provide a time out
   * for checking for presense of SD card prior to performing g2_main() which depends on persistence
   * file for config_init
   */ 
   start_systime_interrupt();

   sd_card_detected_flag=SD_Detect();
   if(!sd_card_detected_flag)
   {
      __NOP();//timed out
   }
  /* G2 Core Application initialization */
  g2_main(); 
  spindle_ctrl_init();

//#ifdef TEST_MCU_WD_TIMER
//   main_test_iwdg();
//#else
  kernel_init_task_control_block(); 
  kernel_scheduler();
 
  /* Loop forever */
  while (1)
  { 
    __NOP();//__no_operation();//should never get here
  }

//#endif 
}/* End Main function */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}
#endif
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
