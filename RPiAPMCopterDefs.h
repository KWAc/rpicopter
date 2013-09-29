#ifndef DEFS_h
#define DEFS_h

// Number of channels
#define APM_IOCHANNEL_COUNT 	8

// PID array (6 pids, two for each axis)
#define PID_PITCH_RATE 	0
#define PID_ROLL_RATE 	1
#define PID_PITCH_STAB 	2
#define PID_ROLL_STAB 	3
#define PID_YAW_RATE 	4
#define PID_YAW_STAB 	5

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1000
#define RC_THR_MAX   1800
#define RC_THR_80P   0.8 * (RC_THR_MAX - RC_THR_MIN) + RC_THR_MIN

#define RC_YAW_MIN   -180
#define RC_YAW_MAX   180

#define RC_PIT_MIN   -45
#define RC_PIT_MAX   45

#define RC_ROL_MIN   -45
#define RC_ROL_MAX   45

uint32_t LSTPKT = 0;
int16_t  CHANNELS[APM_IOCHANNEL_COUNT] = { 0,0,0,0,0,0,0,0 };
char     COMBUF[32];   
int      COMBUFOFFSET = 0;

// PID array (6 pids, two for each axis)
PID pids[6];
// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

#endif /*DEFS_h*/
