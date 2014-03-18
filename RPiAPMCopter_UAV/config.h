#ifndef DEFS_h
#define DEFS_h


#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
  #define A_LED_PIN          37
  #define B_LED_PIN          36
  #define C_LED_PIN          35
  #define LED_ON             HIGH
  #define LED_OFF            LOW
  #define SLIDE_SWITCH_PIN   40
  #define PUSHBUTTON_PIN     41
  #define USB_MUX_PIN        -1
  #define CLI_SLIDER_ENABLED DISABLED
  #define BATTERY_VOLT_PIN   0      // Battery voltage on A0
  #define BATTERY_CURR_PIN   1      // Battery current on A1
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
  #define A_LED_PIN          27
  #define B_LED_PIN          26
  #define C_LED_PIN          25
  #define LED_ON             LOW
  #define LED_OFF            HIGH
  #define SLIDE_SWITCH_PIN   (-1)
  #define PUSHBUTTON_PIN     (-1)
  #define CLI_SLIDER_ENABLED DISABLED
  #define USB_MUX_PIN        23
  #define BATTERY_VOLT_PIN   1      // Battery voltage on A1
  #define BATTERY_CURR_PIN   2      // Battery current on A2
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// General settings
//////////////////////////////////////////////////////////////////////////////////////////

// PID indices
#define PID_PIT_RATE 	       0
#define PID_ROL_RATE 	       1
#define PID_PIT_STAB 	       2
#define PID_ROL_STAB 	       3
#define PID_YAW_RATE 	       4
#define PID_YAW_STAB 	       5

// Motor numbers definitions for X configuration
#define MOTOR_FR             0      // Front right  (CW)
#define MOTOR_BL             1      // back left    (CW)
#define MOTOR_FL             2      // Front left   (CCW)
#define MOTOR_BR             3      // back right   (CCW)

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_OFF           1000   // Motors completely off
// Normal throttle range
#define RC_THR_MIN           1100   // Minimum throttle bias
#define RC_THR_ACRO          1125   // Minimum throttle to begin with stabilization
#define RC_THR_MAX           1900   // Maximum throttle bias
// Maximum allowed throttle value, settable by user
#define RC_THR_80P           0.8 * (RC_THR_MAX - RC_THR_MIN) + RC_THR_MIN

// Degree range for remote control
#define RC_YAW_MIN           -180
#define RC_YAW_MAX           +180
#define RC_PIT_MIN           -45
#define RC_PIT_MAX           +45
#define RC_ROL_MIN           -45
#define RC_ROL_MAX           +45

//////////////////////////////////////////////////////////////////////////////////////////
// Setup serial ports
//////////////////////////////////////////////////////////////////////////////////////////
#define BAUD_RATE_A          115200 // IO USB
#define BAUD_RATE_B          38400  // GPS
#define BAUD_RATE_C          9600   // RADIO

//////////////////////////////////////////////////////////////////////////////////////////
// Main loop
//////////////////////////////////////////////////////////////////////////////////////////
#define MAIN_LOOP_T_MS       6      // Update frequency of the main loop: 166.6 Hz
#define INERT_TIMEOUT        5      // in ms

//////////////////////////////////////////////////////////////////////////////////////////
// Scheduler module
//////////////////////////////////////////////////////////////////////////////////////////
#define NO_PRC_SCHED         16     // Maximum number of processes in scheduler

//////////////////////////////////////////////////////////////////////////////////////////
// Receiver module
//////////////////////////////////////////////////////////////////////////////////////////
#define RADIO_MAX_OFFS       7      // Maximum length of command message via radio without stop bit
#define APM_IOCHAN_CNT 	     8

#define COM_PKT_TIMEOUT      500    // in ms
#define UART_A_TIMEOUT       100    // in ms

#define PID_SIZE             3      // Size of PID array
#define PID_ARGS             4      // Nr of arguments for PID configuration

#define COMP_ARGS            4      // Nr of arguments for on-flight drift compensation

//////////////////////////////////////////////////////////////////////////////////////////
// Device module
//////////////////////////////////////////////////////////////////////////////////////////
#define ATTITUDE_SAMPLE_CNT  10     // Inertial calibration sample count

#define INERT_ANNEAL_SLOPE   20.f   // Slope modifier of the annealing function
#define INERT_FUSION_RATE    5.f    // Sensor fusion rate: higher => faster annealing

#define INERT_LOWPATH_FILT   0.5f   // Filter for the accelerometer

#define CMP_FOR_YAW          0      // Compass
#define GPS_FOR_YAW          0      // GPS

//////////////////////////////////////////////////////////////////////////////////////////
// Error handling
//////////////////////////////////////////////////////////////////////////////////////////
#define VOLTAGE_ALARM_LOW    10.f   // 2.5V per LiPo cell is already close to death of the cell
#define VOLTAGE_ALARM_HIGH   21.f   // 5 * 4.2 V (my setup)
#define THR_STEP_S           1.25f
#define THR_TAKE_OFF         1300
#define MAX_FALL_SPEED_MS    0.833f
#define ALTI_MEASURE_TIME    100    // Time in ms to measure the hight if the model takes down

#endif /*DEFS_h*/
