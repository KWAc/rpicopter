#ifndef DEFS_h
#define DEFS_h


#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
  #define A_LED_PIN             37
  #define B_LED_PIN             36
  #define C_LED_PIN             35
  #define LED_ON                HIGH
  #define LED_OFF               LOW
  #define SLIDE_SWITCH_PIN      40
  #define PUSHBUTTON_PIN        41
  #define USB_MUX_PIN           -1
  #define CLI_SLIDER_ENABLED    DISABLED
  #define BATTERY_VOLT_PIN      0      // Battery voltage on A0
  #define BATTERY_CURR_PIN      1      // Battery current on A1
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
  #define A_LED_PIN             27
  #define B_LED_PIN             26
  #define C_LED_PIN             25
  #define LED_ON                LOW
  #define LED_OFF               HIGH
  #define SLIDE_SWITCH_PIN      (-1)
  #define PUSHBUTTON_PIN        (-1)
  #define CLI_SLIDER_ENABLED    DISABLED
  #define USB_MUX_PIN           23
  #define BATTERY_VOLT_PIN      1      // Battery voltage on A1
  #define BATTERY_CURR_PIN      2      // Battery current on A2
#endif

#define ATTITUDE_SAMPLE_CNT 10

#define INERTIAL_TIMEOUT  5     // in ms
#define SER_PKT_TIMEOUT   500   // in ms

// Number of samples for gyrometer calibration
#define COMPASS_FOR_YAW   0

// Number of channels
#define APM_IOCHANNEL_COUNT 	8

// PID array (6 pids, two for each axis)
#define PID_SIZE        3 // Size of PID array
#define PID_ARGS        4 // Nr of arguments for PID configuration
#define COMP_ARGS       4 // Nr of arguments for on-flight drift compensation

#define PID_PIT_RATE 	  0
#define PID_ROL_RATE 	  1
#define PID_PIT_STAB 	  2
#define PID_ROL_STAB 	  3
#define PID_YAW_RATE 	  4
#define PID_YAW_STAB 	  5

// Motor numbers definitions for X configuration
#define MOTOR_FL   2    // Front left
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Baud rate
#define BAUD_RATE_A  115200 // IO USB
#define BAUD_RATE_B  38400  // GPS
#define BAUD_RATE_C  9600   // RADIO

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_OFF   1000   // Motors completely off
// Normal throttle range
#define RC_THR_MIN   1100   // Minimum throttle bias
#define RC_THR_ACRO  1125   // Minimum throttle to begin with stabilization
#define RC_THR_MAX   1900   // Maximum throttle bias
// Maximum allowed throttle value, settable by user
#define RC_THR_80P   0.8 * (RC_THR_MAX - RC_THR_MIN) + RC_THR_MIN

#define RC_YAW_MIN   -180
#define RC_YAW_MAX   180

#define RC_PIT_MIN   -45
#define RC_PIT_MAX   45

#define RC_ROL_MIN   -45
#define RC_ROL_MAX   45

// battery monitor types
#define AP_BATT_VOLT_OFFSET        0.5   // Offset will get added to the voltage measured
#define AP_BATT_CELL_COUNT         4     // Used for calculation of the percentage of the residual capacity
#define AP_BATT_CAPACITY_DEFAULT   10000 // Total capacity of the battery

#define MAIN_LOOP_T_MS 10                // Update frequency: 100 Hz ~ 10 ms per 1 s

#endif /*DEFS_h*/
