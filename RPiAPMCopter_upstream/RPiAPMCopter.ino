////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Notify.h>
#include <AP_Mission.h>
#include <AP_Buffer.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Declination.h>

#include <GCS_MAVLink.h>
#include <DataFlash.h>
#include <Filter.h>
#include <PID.h>
////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
#include <AP_Compass.h>
#include <AP_RangeFinder.h>
#include <AP_Baro.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_InertialNav.h>
#include <AP_InertialNav_NavEKF.h>
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_AHRS.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>
////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "output.h"
#include "global.h"
#include "math.h"


// Main callback routine for the model, which is used by the task manager
inline void main_loop() {
  _MODEL.run();
}
// Altitude estimation and AHRS system (yaw correction with GPS, barometer, ..)
inline void ahrs_loop() {
  _HAL_BOARD.update_inav();
#ifdef SONAR_TYPE
  _HAL_BOARD.read_rf_m();
#endif
}

#if SIGM_FOR_ATTITUDE
Task taskMain(&main_loop, MAIN_T_MS, 1);
Task taskAHRS(&ahrs_loop, AHRS_T_MS, 1);
#else // Set it to 100 Hz
Task taskMain(&main_loop, 10, 1);
Task taskAHRS(&ahrs_loop, 10, 1);
#endif

void setup() {
  // Totally necessary
  _GPS = &_AUTO_GPS;

  // Prepare scheduler for the main loop ..
  _SCHED.add_task(&taskMain, 0);  // Main loop with own attitude calculation (166 Hz)
  _SCHED.add_task(&taskAHRS, 0);  // Inertial, GPS, Compass, Barometer sensor fusions (slow) ==> running at 50 Hz
  // .. and the sensor output functions
  _SCHED.add_task(&outAtti,  75);
  _SCHED.add_task(&outBaro,  1000);
  _SCHED.add_task(&outGPS,   1000);
  _SCHED.add_task(&outComp,  2000);
  _SCHED.add_task(&outBat,   5000);
  _SCHED.add_task(&outPID,   5000);

  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE_A); // USB
  hal.uartB->begin(BAUD_RATE_B); // GPS
  hal.uartC->begin(BAUD_RATE_C); // RADIO
  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", progress_f(1, 9) );
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", progress_f(2, 9) );
  _HAL_BOARD.init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", progress_f(3, 9) );
  _HAL_BOARD.init_barometer();

  hal.console->printf("%.1f%%: Init inertial sensor\n", progress_f(4, 9) );
  _HAL_BOARD.init_inertial();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", progress_f(5, 9) );
  _HAL_BOARD.init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", progress_f(6, 9) );
  _HAL_BOARD.init_gps();

  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", progress_f(7, 9) );
  _HAL_BOARD.init_batterymon();
  
  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init range finder\n", progress_f(8, 9) );
  _HAL_BOARD.init_rf();
  
  hal.console->printf("\n%.1f%%: Init inertial navigation\n", progress_f(9, 9) );
  _HAL_BOARD.init_inertial_nav();
}

void loop() { 
  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  _RECVR.try_uartAC();
  // send some json formatted information about the model over serial port
  _SCHED.run(); // Wrote my own small and absolutely fair scheduler
}

AP_HAL_MAIN();












