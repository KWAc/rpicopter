#ifndef GLOB_h
#define GLOB_h

#include "BattMonitor.h"
#include "emitter.h"
#include "device.h"
#include "receiver.h"
#include "config.h"


///////////////////////////////////////////////////////////
// ArduPilot Hardware Abstraction Layer
///////////////////////////////////////////////////////////
const AP_HAL::HAL&         hal = AP_HAL_AVR_APM2;

///////////////////////////////////////////////////////////
// Board specific sensors
///////////////////////////////////////////////////////////
/* TODO: Maybe add for support other ArduPilot hardware (APM1, PX4, ..) */
AP_InertialSensor_MPU6000  _INERT;                      // MPU6050 accel/gyro chip
AP_Compass_HMC5843         _COMP;                       // Magnetometer aka compass
AP_Baro_MS5611             _BARO(&AP_Baro_MS5611::spi); // Barometer
AP_GPS_UBLOX               _GPS;                        // GPS
BattMonitor                _BAT;                        // battery monitor

///////////////////////////////////////////////////////////
// Abstracted hardware abstraction classes :D 
// Take any sensor if derived from ArduPilot library!
// Only exception is the battery monitor
// to circumvent the usage of AP_Param for changing settings
///////////////////////////////////////////////////////////
Emitters                   _SCHED     (&hal);           // Scheduler for serial output
Device                     _HAL_BOARD (&hal, &_INERT, &_COMP, &_BARO, &_GPS, &_BAT); // Comprehensive sensor recording class
Receiver                   _RECVR     (&_HAL_BOARD);    // Receiver class for remote control and configuration of settings

#endif

