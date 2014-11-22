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
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

///////////////////////////////////////////////////////////
// Container for all the PIDs of the copter
// PID indices are defined in "config.h"
///////////////////////////////////////////////////////////
AC_PID                         _PIDS[NR_OF_PIDS];

///////////////////////////////////////////////////////////
// Board specific sensors
///////////////////////////////////////////////////////////
#define CONFIG_BARO_TYPE HAL_BARO_DEFAULT
#if CONFIG_BARO_TYPE == HAL_BARO_BMP085
  AP_Baro_BMP085               _BARO;
#elif CONFIG_BARO_TYPE == HAL_BARO_PX4
  AP_Baro_PX4                  _BARO;
#elif CONFIG_BARO_TYPE == HAL_BARO_VRBRAIN
  AP_Baro_VRBRAIN              _BARO;
#elif CONFIG_BARO_TYPE == HAL_BARO_HIL
  AP_Baro_HIL                  _BARO;
#elif CONFIG_BARO_TYPE == HAL_BARO_MS5611
  AP_Baro_MS5611               _BARO(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO_TYPE == HAL_BARO_MS5611_SPI
  AP_Baro_MS5611               _BARO(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO_TYPE setting
#endif

#define CONFIG_COMPASS_TYPE HAL_COMPASS_DEFAULT
#if CONFIG_COMPASS_TYPE == HAL_COMPASS_PX4
  AP_Compass_PX4               _COMP;
#elif CONFIG_COMPASS_TYPE == HAL_COMPASS_VRBRAIN
  AP_Compass_VRBRAIN           _COMP;
#elif CONFIG_COMPASS_TYPE == HAL_COMPASS_HMC5843
  AP_Compass_HMC5843           _COMP;
#elif CONFIG_COMPASS_TYPE == HAL_COMPASS_HIL
  AP_Compass_HIL               _COMP;
#else
 #error Unrecognized CONFIG_COMPASS_TYPE setting
#endif

/*
#define CONFIG_INS_TYPE HAL_INS_DEFAULT
#if CONFIG_INS_TYPE == HAL_INS_MPU6000
  AP_InertialSensor_MPU6000    _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_PX4
  AP_InertialSensor_PX4        _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_VRBRAIN
  AP_InertialSensor_VRBRAIN    _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_HIL
  AP_InertialSensor_HIL        _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_FLYMAPLE
  AP_InertialSensor_Flymaple   _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_L3G4200D
  AP_InertialSensor_L3G4200D   _INERT;
#elif CONFIG_INS_TYPE == HAL_INS_MPU9250
  AP_InertialSensor_MPU9250    _INERT;
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif
*/

AP_InertialSensor              _INERT;
BattMonitor                    _BAT;                                                              // battery monitor
RangeFinder                    _SON_RF;

AP_GPS                         _GPS;                                                              // GPS
GPS_Glitch                     _GPS_GLITCH  (_GPS);
Baro_Glitch                    _BARO_GLITCH (_BARO);
AP_AHRS_DCM                    _AHRS        (_INERT, _BARO, _GPS);
AP_InertialNav                 _INERT_NAV   (_AHRS, _BARO, _GPS_GLITCH, _BARO_GLITCH);

///////////////////////////////////////////////////////////
// Abstracted hardware abstraction classes :D
// Take any sensor if derived from ArduPilot library!
///////////////////////////////////////////////////////////
Scheduler                      _SCHED_NAV   (&hal); // Scheduler for navigation system
Scheduler                      _SCHED_OUT   (&hal); // Scheduler for network output

Device                         _HAL_BOARD   (&hal, &_INERT, &_COMP, &_BARO, &_GPS, &_BAT, &_SON_RF, &_AHRS, &_INERT_NAV, &_PIDS[0]);
Receiver                       _RECVR       (&_HAL_BOARD, &_SCHED_OUT);
Exception                      _EXCP        (&_HAL_BOARD, &_RECVR);

// Currently just a quad-copter with X-frame is implemented
UAVNav                         _UAV         (&_HAL_BOARD, &_RECVR, &_EXCP);
M4XFrame                       _MODEL       (&_HAL_BOARD, &_RECVR, &_EXCP, &_UAV);

#endif

