#ifndef GLOB_h
#define GLOB_h

#include "config.h"
#include "BattMonitor.h"
#include "scheduler.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "rcframe.h"
#include "navigation.h"


///////////////////////////////////////////////////////////
// ArduPilot Hardware Abstraction Layer
///////////////////////////////////////////////////////////
const AP_HAL::HAL&         hal = AP_HAL_BOARD_DRIVER;

///////////////////////////////////////////////////////////
// Board specific sensors
///////////////////////////////////////////////////////////
/* TODO: Maybe add for support other ArduPilot hardware (APM1, PX4, ..) */
AP_InertialSensor_MPU6000  _INERT;                                                            // MPU6050 accel/gyro chip
AP_Compass_HMC5843         _COMP;                                                             // Magnetometer aka compass
AP_Baro_MS5611             _BARO      (&AP_Baro_MS5611::spi);                                       // Barometer
BattMonitor                _BAT;                                                              // battery monitor
AP_RangeFinder_MaxsonarXL  _SON(hal.analogin->channel(3), new ModeFilterInt16_Size5(2) );     // Sonar

AP_GPS                     _GPS;                                                              // GPS
GPS_Glitch                 _GPS_GLITCH(_GPS);
AP_AHRS_DCM                _AHRS      (_INERT, _BARO, _GPS);
AP_InertialNav             _INERT_NAV (_AHRS, _BARO, _GPS_GLITCH);

///////////////////////////////////////////////////////////
// Abstracted hardware abstraction classes :D
// Take any sensor if derived from ArduPilot library!
// Only exception is the battery monitor
// to circumvent the usage of AP_Param for changing settings
///////////////////////////////////////////////////////////
Scheduler                  _SCHED     (&hal);
Device                     _HAL_BOARD (&hal, &_INERT, &_COMP, &_BARO, &_GPS, &_BAT, &_SON, &_AHRS, &_INERT_NAV);
Receiver                   _RECVR     (&_HAL_BOARD);
Exception                  _EXCP      (&_HAL_BOARD, &_RECVR);

// Currently just a quad-copter with X-frame is implemented
UAVNav                     _UAV       (&_HAL_BOARD, &_RECVR, &_EXCP);
M4XFrame                   _MODEL     (&_HAL_BOARD, &_RECVR, &_EXCP, &_UAV);

#endif

