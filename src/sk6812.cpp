// Peripheral usage
#include "sk6812.h"
#include "bantam_hal.h"
#include "g2_config.h"
#include "g2_controller.h" //blink rates

//#include "g2_controller.h"
volatile PwmRgbwPixelS pwm_rgbw_setting;

volatile PwmLedColorTestInfoS led_color_test_info;
// LED driver variables
PwmLedDeviceIdE pwm_device_id=SPOT_LEDS;
PwmLedDeviceInfoS pwm_led_device_info[PWM_LED_DEVICE_COUNT]; //array of structs

// LED color and write buffers
uint32_t rgb_arr[MAX_NUM_BYTES] = { 0 };
uint32_t wr_buf[WR_BUF_LEN_MAX] = { 0 };

// Max supported color
int32_t maxColour = LED_MAX_SETTING;

// R G B and W values
int32_t r_val = 0;
int32_t g_val = 0;
int32_t b_val = 0;
int32_t w_val = 0;

int32_t inc = 6; // how much to increase color each refresh
int8_t toggle = 0;

int32_t moving_pixel_index = 0;
int32_t pwm_moving_LED_index = 0;

// Flag to indicate the DMA has finished its transfer
volatile bool dma_complete = false;
void pwm_select_led_device(int id)
{
    if (id < PWM_LEDS_MAX_ID)
    {
        id = PWM_LEDS_MIN_ID;
    }
    if (id > PWM_LEDS_MAX_ID)
    {
        id = PWM_LEDS_MAX_ID;
    }    
    pwm_device_id=(PwmLedDeviceIdE)id;
}
 
/* This function is called after XDMAC interrupt event */
static void XDMAC_EventHandler(XDMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    static volatile int xdmac_incomplete_transfer_counter=0;
#define MAX_INCOMPLETE_XFR_COUNT 100 //arbitrary
    // Stop the DMA and set completion flag
    if (event == XDMAC_TRANSFER_COMPLETE) 
    {
      XDMAC_ChannelDisable(XDMAC_CHANNEL_1);
    }
#if 1//sme 8-26-2022 Examination of the interrupt shows it is possible to have a XDMAC_TRANSFER_ERROR
    //wherein, the comments indicate it to be a spurious firing of that interrupt, not a failure in an
    //actual xfr operation, 
    else 
    {       
        if(++xdmac_incomplete_transfer_counter>MAX_INCOMPLETE_XFR_COUNT)
        {
          xdmac_incomplete_transfer_counter=1;  
        }
        //return;--no don't return here only return if positive this is not a spurious finring .  
        //Otherwise there will be un-done housekeeping status .
        //it is harmless to clear status when there was not actual transfer so this is the safest route to take
    }
#endif
    dma_complete = true;
  
    // Stop the appropriate PWM output(s) and clear active flag(s)
    if (pwm_led_device_info[STAT_LED].active) 
    {
       PWM0_ChannelsStop(PWM_CHANNEL_0_MASK); // Stop the primary synchronous channel to stop all PWM outputs
       pwm_led_device_info[STAT_LED].active = false;     
    }
    
    if (pwm_led_device_info[SPOT_LEDS].active) 
    {
       PWM0_ChannelsStop(PWM_CHANNEL_0_MASK); // Stop the primary synchronous channel to stop all PWM outputs
       pwm_led_device_info[SPOT_LEDS].active = false;     
    }
}

/*
 * Set an individual LED in the display
 * Note that the index will be which Pixel in the strip you want to set
 * and each Pixel has either 3 (rgb) or 4 (rgbw) LEDs 
 */
void pwm_led_set_RGBW(PwmLedDeviceInfoS led_device, uint32_t index, uint32_t r, uint32_t g, uint32_t b, uint32_t w) 
{
  //Get the offset in the buffer when this pixel's index maps to: 
  int pixel_offset = led_device.num_bpp * index;
  
  //Offset ordering of r, g, b[,w] LEDs in each pixel depends on whether it is a rgbw or a rgb type.
  if (led_device.num_bpp == NUM_BYTES_PER_PIXEL_RGBW) 
  { // 4 LED Pixel, GRBW
    rgb_arr[pixel_offset + RGBW_GREEN_OFFSET] = g;
    rgb_arr[pixel_offset + RGBW_RED_OFFSET]   = r;  
    rgb_arr[pixel_offset + RGBW_WHITE_OFFSET] = w;
  } 
  else 
  { // 3 LED Pixel, RGB
    rgb_arr[pixel_offset + RGB_RED_OFFSET]   = r;
    rgb_arr[pixel_offset + RGB_GREEN_OFFSET] = g;
  }
  //In either case, the 'b' blue LED is in the same additional offset of 2:
  rgb_arr[pixel_offset + RGBW_BLUE_OFFSET] = b;
}


// Set All the LEDs in all the Pixels
void pwm_led_set_all_RGBW(PwmLedDeviceInfoS led_device, uint32_t r, uint32_t g, uint32_t b, uint32_t w) 
{
	for (uint_fast8_t i = 0; i < led_device.num_pixels; ++i)
    {
		pwm_led_set_RGBW(led_device, i, r, g, b, w);
    }
}

// Send the write buffer to the DMA channel: sme: This all makes more sense when we understand 
//the DMA transfer is refreshing both the single rgb LED and the multi pixel rgbw spot leds ring
void pwm_led_render(PwmLedDeviceInfoS led_device) 
{ 
    uint_fast8_t pwm_channel_offset;
    volatile int wrbuf_offset=0; 
    uint8_t pwm_output=0;
    uint8_t bit_value=0;
    volatile int wrbuf_offset_result1 = 0;
    volatile int wrbuf_offset_result2 = 0;
    
    // Ongoing transfer, wait for completion
    if (XDMAC_ChannelIsBusy(XDMAC_CHANNEL_1)) 
    {  // Wait for DMA to process
       while(!dma_complete){ }
       dma_complete = false;
    }
  
    // Set up the PWM channel offset into the write buffer
    // DMA data must be sent in synchronous channel order
    // (i.e. PWM_CHANNEL_0, PWM_CHANNEL_1, PWM_CHANNEL_2...)
    // FYI: sme: The two separate pwm channels that supply the two different smart led devices are serviced
    //      by  single write buffer, wr_buf, with the STAT_LED device data applied to even-numbered bytes, and the 
    //      SPOT_LEDS device data applied to odd-numbered bytes. This is the "interleaved" method means.
    //      Now, the number of bytes sent out on the wr_buf reflect the smart led device that is recieving a new setting. 
    //      The other device's data is still present, and unchanged. so, the old data is harmlessly re-written to 
    //      the other device, limited to the number of bytes required to be written by the device receiving the new data.
    //      
    if (led_device.pwm_channel == PWM_CHANNEL_1_MASK) 
    {
       pwm_channel_offset = 1;//Interleaving: every odd-numbered buf entry gets populated
    } 
    else 
    {
       pwm_channel_offset = 0;//Interleaving: every even-numbered buf entry gets populated
    }

    // Load the LED values into array, interleaving 
    // the data based on the PWM channel offset
	for (uint_fast8_t pixel_offset = 0; pixel_offset < led_device.num_bytes; ++pixel_offset) 
    {          
		for (uint_fast8_t bit_offset = 0; bit_offset < BITS_PER_LED; ++bit_offset) 
        { 
            // default--gets changed when needed,below; 
            pwm_output = PWM_LO;
            
            // Examine each bit by using left shifts to land it at the MSB position 
            // and apply the single MSB AND-mask operation   
            bit_value=(rgb_arr[pixel_offset] << bit_offset) & MSB_BIT_COMPARE_AND_MASK;
 
            if (bit_value != BIT_LOW_VALUE)
            {
                pwm_output = PWM_HI;                
            }
            //Map to the pixel's offset in the wr_buff
            //Note, the complexity of having one write buffer to accommodate PWM_LED_DEVICE_COUNT==2 
            //separate pwm driven led devices--> two separate pwm output channels:
            wrbuf_offset =((((BITS_PER_LED * pixel_offset) + bit_offset)* PWM_LED_DEVICE_COUNT) + pwm_channel_offset);
            wr_buf[wrbuf_offset]=pwm_output;
        }        
    }
  
    // Move buffer from data cache to SRAM, prevents coherency issues
    DCACHE_CLEAN_BY_ADDR((uint32_t *)wr_buf, sizeof(wr_buf));

    // Start the DMA transfer - sends data for both PWM channels (2x strip write buffer)
    XDMAC_ChannelTransfer(XDMAC_CHANNEL_1, 
                       (const void *)(&wr_buf),          
                       (const void *)(&PWM0_REGS->PWM_DMAR),
                       (led_device.wr_buf_tot * PWM_LED_DEVICE_COUNT));
  
    PWM0_ChannelsStart(PWM_CHANNEL_0_MASK); // Start the primary synchronous channel to start all PWM outputs
  
    // Set the flag for channel active
    led_device.active = true;

    // Wait for DMA to process
    while(!dma_complete){}
    dma_complete = false;
}

// Sets all LEDs to off (black) and populates reset pulse
void pwm_led_device_init(void) 
{
    // Fill array with reset pulse
    for (uint_fast8_t i = 0; i < WR_BUF_LEN_MAX; i++) 
    {
      wr_buf[i] = 0xff;
    }
    // Initialize the LED array
    memset((void*)&pwm_led_device_info, 0, sizeof(pwm_led_device_info));
    
    // Init Stat LEDs (1-pixel single, RGB unit)    
    pwm_led_device_info[STAT_LED].num_bpp     = NUM_BYTES_PER_PIXEL_RGB;
    pwm_led_device_info[STAT_LED].num_pixels  = NUM_PIXELS_STAT_LED;
    pwm_led_device_info[STAT_LED].pwm_channel = PWM_CHANNEL_0_MASK;
    pwm_led_device_info[STAT_LED].num_bytes  = (pwm_led_device_info[STAT_LED].num_bpp * pwm_led_device_info[STAT_LED].num_pixels);
    pwm_led_device_info[STAT_LED].wr_buf_tot = (pwm_led_device_info[STAT_LED].num_bytes * BITS_PER_LED) + NUM_BYTES_RESET_PULSE;     
    pwm_led_device_info[STAT_LED].active = false;   

    //Init Spotlight LEDs (16-pixel RGBW ring)
    pwm_led_device_info[SPOT_LEDS].num_bpp     = NUM_BYTES_PER_PIXEL_RGBW;
    pwm_led_device_info[SPOT_LEDS].num_pixels  = NUM_PIXELS_SPOT_LED; 
    pwm_led_device_info[SPOT_LEDS].pwm_channel = PWM_CHANNEL_1_MASK;        
    pwm_led_device_info[SPOT_LEDS].num_bytes = (pwm_led_device_info[SPOT_LEDS].num_bpp * pwm_led_device_info[SPOT_LEDS].num_pixels);
    pwm_led_device_info[SPOT_LEDS].wr_buf_tot = (pwm_led_device_info[SPOT_LEDS].num_bytes * BITS_PER_LED) + NUM_BYTES_RESET_PULSE;
    pwm_led_device_info[SPOT_LEDS].active = false; 
        
    // Set LEDs to off (black)
    pwm_led_set_all_RGBW(pwm_led_device_info[STAT_LED], LED_OFF, LED_OFF, LED_OFF, LED_OFF);
    
    // Register XDMAC callback
    XDMAC_ChannelCallbackRegister(XDMAC_CHANNEL_1, XDMAC_EventHandler, (uintptr_t)NULL);  
}

void pwm_led_color_test(PwmLedDeviceInfoS led_device) 
{
  int r, g, b, w;
  uint8_t increment = 50;
  uint32_t dly = 25;

  // Red
  for (r=0; r < maxColour; r += increment) 
  {
    //leds arg ordering:        r  g  b  w
    pwm_led_set_all_RGBW(led_device, r, LED_OFF, LED_OFF, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Green
  for (g = 0; g < maxColour+1; g += increment) 
  {
    //leds arg ordering:        r  g  b  w     
    pwm_led_set_all_RGBW(led_device, LED_OFF, g, LED_OFF, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Blue
  for (b = 0; b < maxColour+1; b += increment) 
  {
    //leds arg ordering:        r  g  b  w       
    pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF, b, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Yellow
  for (r = g = 0; r < maxColour+1 && g < maxColour+1; r += increment, g += increment) 
  {
    //leds arg ordering:        r  g  b  w   
    pwm_led_set_all_RGBW(led_device, r, g, LED_OFF, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Cyan
  for (g = b = 0; g < maxColour+1 && b < maxColour+1; g += increment, b += increment) 
  {
    //leds arg ordering:        r  g  b  w   
    pwm_led_set_all_RGBW(led_device, LED_OFF, g, b, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Magenta
  for (r = b=0; r < maxColour+1 && b < maxColour+1; r += increment, b += increment) 
  {
    //leds arg ordering:        r  g  b  w 
    pwm_led_set_all_RGBW(led_device, r, LED_OFF, b, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // White (RGB LEDs)
  for (r = g = b = 0; r < maxColour+1 && g < maxColour+1 && b < maxColour+1; r += increment, g += increment, b += increment) 
  {
    //leds arg ordering:        r  g  b  w   
    pwm_led_set_all_RGBW(led_device, r, g, b, LED_OFF);
    pwm_led_render(led_device);
    delay_ms(dly);
  }

  // Warm White (4 LED pixels only)
  if (led_device.num_bpp == 4) 
  {
       // Warm White
        for ( w = 0; w < maxColour+1; w += increment) 
        {
           //leds arg ordering:        r  g  b  w             
           pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF, LED_OFF, w);
           pwm_led_render(led_device);
           delay_ms(dly);
        }
  }

  // Turn off LEDs when done
  //leds arg ordering:             r         g       b        w 
  pwm_led_set_all_RGBW(led_device, LED_OFF,LED_OFF, LED_OFF, LED_OFF);
  pwm_led_render(led_device);
}

/*
 * Pulse an Individual LED in the display so that it goes from 0 to 255 and back to 0 and repeats over.
 *
 */
void pwm_pwm_pulse_all_pixels_single_colour(PwmLedDeviceInfoS led_device, uint8_t color) 
{
	switch (color) 
    {
	case 'r':
		pwm_pulse_all_pixels(led_device, &r_val, color);
		break;
	case 'g':
		pwm_pulse_all_pixels(led_device, &g_val, color);
		break;
	case 'b':
		pwm_pulse_all_pixels(led_device, &b_val, color);
	case 'w':
		pwm_pulse_all_pixels(led_device, &w_val, color);
		break;
	default:
		break;
	}
}

void pwm_pulse_all_pixels(PwmLedDeviceInfoS led_device, int32_t *color_val, uint8_t color) 
{
	if (toggle == 0) 
    {		//increment color
		*color_val = *color_val + inc;
		if (*color_val > maxColour) 
        {
			*color_val = maxColour;
			toggle = 1;
		}
	} 
    else 
    {		// Decrement color
		*color_val = *color_val - inc;
		if (*color_val < 0) 
        {
			*color_val = 0;
			toggle = 0;
		}
	}

	switch (color) 
    {
	case 'r':
		pwm_led_set_all_RGBW(led_device, *color_val, LED_OFF, LED_OFF, LED_OFF);
		break;
	case 'g':
		pwm_led_set_all_RGBW(led_device, LED_OFF, *color_val, LED_OFF, LED_OFF);
		break;
	case 'b':
		pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF, *color_val, LED_OFF);
		break;   
	case 'w':
		pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF,LED_OFF, *color_val);
		break;
	default:
		break;
	}
}

/*
 * Move a RBGW LED from one pixel to the next and repeat over.
 */
void pwm_moving_pixel_single_colour(PwmLedDeviceInfoS led_device, uint32_t r, uint32_t g, uint32_t b, uint32_t w) 
{
	pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF,LED_OFF,LED_OFF);		// clear the previous led
    pwm_led_set_RGBW(led_device, moving_pixel_index, r, g, b, w);

	moving_pixel_index++;
	if (moving_pixel_index > led_device.num_pixels) 
    {
		moving_pixel_index = 0;
	}
}

/*
 * Move through all the LEDs and Pixels in a , so is Pixel 0, R->G->B->W , then Pixel 1, R->G->B->W etc
 */
void pwm_moving_LED(PwmLedDeviceInfoS led_device) 
{

	pwm_led_set_all_RGBW(led_device,  LED_OFF, LED_OFF,LED_OFF,LED_OFF);		// clear the previous led
	int32_t ledIndex = 0;

	moving_pixel_index = (int32_t) pwm_moving_LED_index / led_device.num_bpp;
	ledIndex = pwm_moving_LED_index % led_device.num_bpp;		// modulus

	switch (ledIndex) 
    {
	case RGBW_RED_OFFSET:
		pwm_led_set_RGBW(led_device, moving_pixel_index, LED_MAX_SETTING,  LED_OFF, LED_OFF,LED_OFF);
		break;
	case RGBW_GREEN_OFFSET:
		pwm_led_set_RGBW(led_device, moving_pixel_index, LED_OFF, LED_MAX_SETTING, LED_OFF, LED_OFF);
		break;
	case RGBW_BLUE_OFFSET:
		pwm_led_set_RGBW(led_device, moving_pixel_index, LED_OFF, LED_OFF, LED_MAX_SETTING, LED_OFF);
		break;
	case RGBW_WHITE_OFFSET:
		pwm_led_set_RGBW(led_device, moving_pixel_index, LED_OFF, LED_OFF, LED_OFF,LED_MAX_SETTING);
		break;
	default:
		break;
	}

	pwm_moving_LED_index++;
	if (pwm_moving_LED_index > led_device.num_bytes) 
    {
		pwm_moving_LED_index = 0;
	}
}

/*
 * Move an LED from one Pixel at the Next when the max pixel is reached the decrement back to the start and repeat over.
 */
void pwm_led_color_cylon(PwmLedDeviceInfoS led_device, uint32_t r, uint32_t g, uint32_t b, uint32_t w) 
{
	pwm_led_set_all_RGBW(led_device, LED_OFF, LED_OFF, LED_OFF, LED_OFF);		// clear the previous led
	pwm_led_set_RGBW(led_device, moving_pixel_index, r, g, b, w);

	if (toggle == 0) 
    {		//increase the index
		moving_pixel_index++;
		if (moving_pixel_index > led_device.num_pixels - 1) 
        {
			moving_pixel_index = led_device.num_pixels - 1;
			toggle = 1;
		}
	} 
    else 
    {		// Decrement color
		moving_pixel_index--;
		if (moving_pixel_index < 0) 
        {
			moving_pixel_index = 0;
			toggle = 0;
		}
	}
}

/* 
 * Print funcs for console commands
*/
void pwm_rgbw_print(nvObj_t *nv)
{
  const char fmt[]="[rgbw leds setting] red=%d, green=%d, blue=%d, white=%d, hz=%d\n";

   sprintf(cs.out_buf,fmt,
           pwm_rgbw_setting.red,
           pwm_rgbw_setting.green,
           pwm_rgbw_setting.blue,  
           pwm_rgbw_setting.white,
           pwm_rgbw_setting.blink_hz);
   comms_mgr_write_msg(cs.out_buf) ; 
}
void rgbdemo_print(nvObj_t *nv)
{
  const char *states[LEDS_DEMO_COUNT]={"Off", "Running"};
  const char fmt[]="[rgbw leds demo state] Off/Running: %s\n";
  uint8_t value=(uint8_t)nv->value;
 // if (value<LED_TEST_COUNT)
  //{
  //  led_color_test_info.led_color_test_id=value;
  //}
  /* Protect against array bounds overrun */
  if (value > LED_TEST_IDLE)
  {
    value=LEDS_DEMO_RUNNING;
  }
  sprintf(cs.out_buf,fmt,states[value]);
   
  comms_mgr_write_msg(cs.out_buf) ;  
}


//volatile PwmLedColorTestInfoS led_color_test_info ={LED_TEST_IDLE,LED_OFF, LED_TEST_DEFAULT_INCREMENT};
void pwm_led_start_color_test(void)
{
  led_color_test_info.led_color_test_id=LED_TEST_START_COLOR;
  led_color_test_info.brightness=MIN_COLOR_BRIGHTNESS;
}
void pwm_led_end_color_test(PwmLedDeviceInfoS led_device)
{
     pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],LED_OFF, LED_OFF, LED_OFF, LED_OFF);
     pwm_led_render(led_device);
     led_color_test_info.led_color_test_id = LED_TEST_IDLE;
}
void pwm_led_color_state_machine(void) 
{
  int temp_test_id = (int)led_color_test_info.led_color_test_id;
  switch(led_color_test_info.led_color_test_id)
  {
  case LED_TEST_IDLE: 
    /* nothing to do here*/
    __NOP();
    break;
    
  case LED_TEST_RED:     
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     led_color_test_info.brightness, 
                     LED_OFF, 
                     LED_OFF, 
                     LED_OFF);
    break;
    
  case LED_TEST_GREEN:
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     LED_OFF, 
                     led_color_test_info.brightness, 
                     LED_OFF, 
                     LED_OFF);
    break;
    
  case LED_TEST_BLUE: 
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     LED_OFF, 
                     LED_OFF, 
                     led_color_test_info.brightness, 
                     LED_OFF);
    break;
    
  case LED_TEST_YELLOW:
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     led_color_test_info.brightness, 
                     led_color_test_info.brightness, 
                     LED_OFF, 
                     LED_OFF);
    break;
    
  case LED_TEST_CYAN:
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     LED_OFF, 
                     led_color_test_info.brightness, 
                     led_color_test_info.brightness,
                     LED_OFF);
    break;
    
  case LED_TEST_MAGENTA:
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     led_color_test_info.brightness,
                     LED_OFF, 
                     led_color_test_info.brightness, 
                     LED_OFF);
    break;
    
  case LED_TEST_WHITE:
    pwm_led_set_all_RGBW(pwm_led_device_info[pwm_device_id],
                     led_color_test_info.brightness, 
                     led_color_test_info.brightness, 
                     led_color_test_info.brightness, 
                     LED_OFF);
    break;
    
  case LED_TEST_WARM_WHITE:
    pwm_led_set_all_RGBW(
                     pwm_led_device_info[pwm_device_id],
                     LED_OFF, 
                     LED_OFF, 
                     LED_OFF, 
                     led_color_test_info.brightness);
    break; 
    
    default:
    /* should never land here unless data corruption */
    __NOP();
    break;
  }/* End Switch */
  
 /* Color test continuation housekeeping, when test id is not idle:*/
 if (led_color_test_info.led_color_test_id !=LED_TEST_IDLE)
 {
    pwm_led_render(pwm_led_device_info[pwm_device_id]);
    led_color_test_info.brightness += led_color_test_info.increment; 

    if(led_color_test_info.brightness >= MAX_COLOR_BRIGHTNESS)
    {
       /* Advance to the next test:*/
      led_color_test_info.brightness=MIN_COLOR_BRIGHTNESS;
      
      temp_test_id++; /* work around the compiler's strict type checking on enum inrementing*/
      led_color_test_info.led_color_test_id=( PwmLedColorTestIdE )temp_test_id; 
      
      /* Loop around to repeat the entire test, as needed*/
      if(led_color_test_info.led_color_test_id > LED_TEST_LAST_COLOR)
      {
        led_color_test_info.led_color_test_id = LED_TEST_START_COLOR;
      }
    }
 }
}/* End function */