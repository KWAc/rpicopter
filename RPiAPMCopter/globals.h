#ifndef GLOB_h
#define GLOB_h

#include "defines.h"


// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 inertial;

// Magnetometer aka compass
AP_Compass_HMC5843 compass;
bool COMPASS_INITIALIZED = 0;

// Barometer
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
bool BAROMETER_INITIALIZED = 0;

// GPS
AP_GPS_UBLOX gps;

// battery monitor
BattMonitor battery;

// Motor control 
// PID array (6 pids, two for each axis)
PID PIDS[6];

float OUT_HEADING       = 0.f;
float OUT_PIT           = 0.f;
float OUT_ROL           = 0.f;
float OUT_YAW           = 0.f;

// Gyrometer calibration:
// After boot of the APM and before flight the gyrometer is calibrated on (even) ground
float GYRO_ROL_OFFS     = 0.f;
float GYRO_PIT_OFFS     = 0.f;
float GYRO_YAW_OFFS     = 0.f;

// On flight correction in case the quadrocopter is drifting (values _signed_)
// e.g. because of imbalances of the built, gyrometer not nice calibrated
float GYRO_ROL_COR      = 0.f; // left to right or right to left
float GYRO_PIT_COR      = 0.f; // front to back or back to front

// Remote control
int16_t  RC_CHANNELS[APM_IOCHANNEL_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// Timer for last package via WiFi connection
uint32_t iWiFiTimer = 0;
// Emitter timers
uint32_t iFastTimer = 0;
uint32_t iMediTimer = 0;
uint32_t iSlowTimer = 0;
uint32_t iUslwTimer = 0;

#endif
