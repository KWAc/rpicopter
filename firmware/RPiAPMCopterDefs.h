#ifndef DEFS_h
#define DEFS_h

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  #define A_LED_PIN 27
  #define C_LED_PIN 25
#else
  #define A_LED_PIN 37
  #define C_LED_PIN 35
#endif

// Number of samples for gyrometer calibration
#define COMPASS_FOR_YAW   0

// Number of channels
#define APM_IOCHANNEL_COUNT 	8

// PID array (6 pids, two for each axis)
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
#define BAUD_RATE    115200

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

#endif /*DEFS_h*/
