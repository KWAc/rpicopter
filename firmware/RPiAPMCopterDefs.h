#ifndef DEFS_h
#define DEFS_h

bool  GYRO_CALI         = 1;
float GYRO_ROL_DRIFT    = 0;
float GYRO_PIT_DRIFT    = 0;
float GYRO_YAW_DRIFT    = 0;

// Number of channels
#define APM_IOCHANNEL_COUNT 	8

// Motor control
PID      PIDS[6];       // PID array (6 pids, two for each axis)
// PID array (6 pids, two for each axis)
#define PID_PITCH_RATE 	  0
#define PID_ROLL_RATE 	  1
#define PID_PITCH_STAB 	  2
#define PID_ROLL_STAB 	  3
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
#define RC_THR_ACRO  1200   // Minimum throttle to begin with stabilization
#define RC_THR_MAX   1900   // Maximum throttle bias
// Maximum allowed throttle value, settable by user
#define RC_THR_80P   0.8 * (RC_THR_MAX - RC_THR_MIN) + RC_THR_MIN

#define RC_YAW_MIN   -180
#define RC_YAW_MAX   180

#define RC_PIT_MIN   -45
#define RC_PIT_MAX   45

#define RC_ROL_MIN   -45
#define RC_ROL_MAX   45

// Compass
bool COMPASS_AVAIL = 1;

// Remote control
uint32_t RC_PACKET_T = 0;
int16_t  RC_CHANNELS[APM_IOCHANNEL_COUNT] = { 0,0,0,0,0,0,0,0 };

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 inertial;
// Magnetometer aka compass
AP_Compass_HMC5843 compass;

#endif /*DEFS_h*/
