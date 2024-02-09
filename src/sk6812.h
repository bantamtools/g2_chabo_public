
#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812

#include <stdint.h>
#include <string.h>
#include "definitions.h"
#include "util.h"
#include "main.h"
#include "bantam_hal.h"
#include "g2core.h"  // #1

    
// Definitions

#define MSB_BIT_COMPARE_AND_MASK 0x80
#define BIT_LOW_VALUE 0
#define BITS_PER_LED 8
#define LED_OFF 0
#define LED_MAX_SETTING 255
#define RGBW_DEFAULT_SETTING LED_MAX_SETTING
#define RGBW_DEFAULT_BLINK_HZ LED_OFF
#define MIN_COLOR_BRIGHTNESS LED_OFF
#define MAX_COLOR_BRIGHTNESS 255

// Number of smart RGB/RGBW LEDs
// replaced by enum below #define NUM_RGB_LEDS 2
#define NUM_PIXELS_SPOT_LED 16
#define NUM_PIXELS_STAT_LED 1

#define NUM_BYTES_PER_PIXEL_RGBW 4
#define NUM_BYTES_PER_PIXEL_RGB  3

// LED pulses for hi and lo...timing tweaked slightly for tolerances
#define PWM_HI (47)
#define PWM_LO (68)

// Reset pulse length 70 bytes note at frequency of 833Khz this is just over required 80usec
#define NUM_BYTES_RESET_PULSE (70)

// Maximum number of bytes (assumes a 50-pixel RGBW string)
#define MAX_NUM_BYTES 200

typedef enum
{
  RGBW_GREEN_OFFSET,
  RGBW_RED_OFFSET,
  RGBW_BLUE_OFFSET,
  RGBW_WHITE_OFFSET,
  RGBW_COLOR_COUNT
}PwmRgbwOffsetsE;

typedef enum
{
  RGB_RED_OFFSET,
  RGB_GREEN_OFFSET,
  RGB_BLUE_OFFSET,
  RGB_COLOR_COUNT
}PwmRgbOffsetsE;

typedef struct
{
 uint8_t red;
 uint8_t green;
 uint8_t blue;
 uint8_t white;
 uint8_t blink_hz;//0= solid, 1= 1hz, 2= 2 hz...
}PwmRgbwPixelS;

typedef union
{
  PwmRgbwPixelS struc_data;
  uint8_t bdata[sizeof(PwmRgbwPixelS)];
}PwmRgbwPixelU; 
/* 
 * the state machine version subjugates this test to the kernel scheduling
 *  architecture, thereby applying non-blocking delays instead of the 
 *  blocking HAL_delay calls
*/
typedef enum
{
  LED_TEST_IDLE,
  LED_TEST_RED,
  LED_TEST_GREEN,
  LED_TEST_BLUE,
  LED_TEST_YELLOW,
  LED_TEST_CYAN,
  LED_TEST_MAGENTA,
  LED_TEST_WHITE,
  LED_TEST_WARM_WHITE,
  LED_TEST_COUNT,
  LED_TEST_START_COLOR=LED_TEST_RED,
  LED_TEST_LAST_COLOR=LED_TEST_WARM_WHITE   
}PwmLedColorTestIdE;
typedef enum
{
  LEDS_DEMO_OFF,
  LEDS_DEMO_RUNNING,
  LEDS_DEMO_COUNT
}PwmLedDemoStateE;
#define RGB_DEMO_DEFAULT_STATE LED_TEST_IDLE
typedef struct
{
  PwmLedColorTestIdE led_color_test_id;
  int brightness;
  int increment;
}PwmLedColorTestInfoS;

// Type Definitions
typedef struct 
{

  uint8_t           num_bpp;      // Number of bytes per pixel (RGB = 3, RGBW = 4))
  uint8_t           num_pixels;   // Number of pixels in LED strip
  uint16_t          num_bytes;    // Number of bytes (# pixels x # bpp)
  uint16_t          wr_buf_tot;   // Write buffer length (# bytes x 8 + NUM_BYTES_RESET_PULSE)
    
  PWM_CHANNEL_MASK  pwm_channel;  // PWM output
  bool              active;       // Channel is active
} PwmLedDeviceInfoS;

typedef enum led_dout 
{
  STAT_LED,
  SPOT_LEDS,
  PWM_LED_DEVICE_COUNT,
  PWM_LEDS_MAX_ID= SPOT_LEDS,
  PWM_LEDS_MIN_ID= STAT_LED
} PwmLedDeviceIdE;
// Maximum write buffer size (based on MAX_NUM_BYTES)
#define WR_BUF_LEN_MAX ((MAX_NUM_BYTES * BITS_PER_LED) + NUM_BYTES_RESET_PULSE) * PWM_LED_DEVICE_COUNT  // PWM_LED_DEVICE_COUNT==x2 to hold data for both PWM outputs

// Function Prototypes
void pwm_led_set_RGBW(PwmLedDeviceInfoS, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
void pwm_led_set_all_RGBW(PwmLedDeviceInfoS, uint32_t, uint32_t, uint32_t, uint32_t);
void pwm_led_render(PwmLedDeviceInfoS);
void pwm_led_device_init(void);
void pwm_led_color_test(PwmLedDeviceInfoS);
void pwm_pwm_pulse_all_pixels_single_colour(PwmLedDeviceInfoS, uint8_t);
void pwm_moving_pixel_single_colour(PwmLedDeviceInfoS, uint32_t, uint32_t, uint32_t,uint32_t);
void pwm_moving_LED(PwmLedDeviceInfoS);
void pwm_pulse_all_pixels(PwmLedDeviceInfoS, int32_t *, uint8_t);
void pwm_led_color_cylon(PwmLedDeviceInfoS, uint32_t, uint32_t, uint32_t, uint32_t);
/* led_color_test, adapted to run under kernel: */
void pwm_led_color_state_machine(void);
void pwm_select_led_device(int id);

/* Data stores accessed in config_array, in config_app.cpp: */
extern volatile PwmRgbwPixelS pwm_rgbw_setting;
extern PwmLedDemoStateE rgbw_demo_state;
extern volatile PwmLedColorTestInfoS led_color_test_info;
extern PwmLedDeviceInfoS pwm_led_device_info[PWM_LED_DEVICE_COUNT];
#endif
