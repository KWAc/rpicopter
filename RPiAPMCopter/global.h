#ifndef GLOB_h
#define GLOB_h

#include "BattMonitor.h"
#include "emitter.h"
#include "device.h"
#include "config.h"


// Motor control 
PID PIDS[6]; // PID array (6 pids, two for each axis)
// Remote control
int16_t RC_CHANNELS[APM_IOCHANNEL_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 inertial;
// Magnetometer aka compass
AP_Compass_HMC5843 compass;
// Barometer
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
// GPS
AP_GPS_UBLOX gps;
// battery monitor
BattMonitor battery;

// Scheduler for serial output
Emitters ser_scheduler(&hal);
// Comprehensive sensor recording class
Device hal_board(&hal, &inertial, &compass, &barometer, &gps, &battery, PIDS);

// On flight correction in case the quadrocopter is drifting (values _signed_)
// e.g. because of imbalances of the built, gyrometer not nice calibrated
float GYRO_ROL_COR      = 0.f; // left to right or right to left
float GYRO_PIT_COR      = 0.f; // front to back or back to front

// Timer for last package via WiFi connection
uint32_t iWiFiTimer = 0;

#endif

