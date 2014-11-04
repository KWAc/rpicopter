////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>
#include <AP_Buffer.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_Math.h>
#include <AP_Mission.h>
#include <AP_Notify.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_RangeFinder.h>
#include <AP_Terrain.h> 
#include <AP_Vehicle.h>

#include <DataFlash.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <PID.h>
#include <RC_Channel.h>     // RC Channel Library
#include <StorageManager.h>

////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "output.h"
#include "global.h"
#include "arithmetics.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
// Settings stored in the flash
inline void load_settings();
// Tasks
inline void rcvr_loop(int);
inline void inav_loop(int);
inline void batt_loop(int);

Task taskRCVR(&rcvr_loop, 0, 1);
Task taskINAV(&inav_loop, 0, 1);
Task taskRBat(&batt_loop, 0, 1);


// Altitude estimation and AHRS system (yaw correction with GPS, barometer, ..)
void inav_loop(int) {  
  _HAL_BOARD.update_inav();
  _HAL_BOARD.read_rf_cm();
}

// Receiver thread (remote control), which should be limited to 50 Hz
void rcvr_loop(int) {
  _RECVR.try_any();
}

// Read the battery. 
// The voltage is used for adjusting the motor speed,
// as the speed is dependent on the voltage 
void batt_loop(int) {
  _HAL_BOARD.read_bat();
  _MODEL.calc_batt_comp();
}

void load_settings() {
  if (!_COMP._learn.load() ) { }
  // change the default for the AHRS_GPS_GAIN for ArduCopter
  // if it hasn't been set by the user
  if (!_AHRS.gps_gain.load() ) { }
  // Setup different AHRS gains for ArduCopter than the default
  // but allow users to override in their config
  if (!_AHRS._kp.load() ) { }
  if (!_AHRS._kp_yaw.load() ) { }
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED_NAV.add_task(&taskRCVR, RCVR_T_MS);
  _SCHED_NAV.add_task(&taskINAV, INAV_T_MS);
  _SCHED_NAV.add_task(&taskRBat, BATT_T_MS);
  // .. and the sensor output functions
  _SCHED_OUT.add_task(&outAtti,  30);
  _SCHED_OUT.add_task(&outBaro,  500);
  _SCHED_OUT.add_task(&outBat,   750);
  _SCHED_OUT.add_task(&outGPS,   1000);
  _SCHED_OUT.add_task(&outComp,  1500);
  _SCHED_OUT.add_task(&outPIDAtt,2000);
  _SCHED_OUT.add_task(&outPIDAlt,2000);

  // Wait for one second
  hal.scheduler->delay(1000);
  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE_A, 256, 256);  // USB
  hal.uartB->begin(BAUD_RATE_B, 256, 16);   // GPS
  hal.uartC->begin(BAUD_RATE_C, 128, 128);  // RADIO
  
  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", progress_f(1, 10) );
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // PID Configuration
  hal.console->printf("%.1f%%: Load PID configuration\n", progress_f(2, 10) );
  _HAL_BOARD.load_pids();

  // Load settings from EEPROM
  hal.console->printf("%.1f%%: Load settings from EEPROM\n", progress_f(3, 10) );
  //load_settings();
  
  hal.console->printf("%.1f%%: Init barometer\n", progress_f(4, 10) );
  _HAL_BOARD.init_barometer();

  hal.console->printf("%.1f%%: Init inertial sensor\n", progress_f(5, 10) );
  _HAL_BOARD.init_inertial();

  // Compass initializing
  hal.console->printf("\n%.1f%%: Init compass: ", progress_f(6, 10) );
  _HAL_BOARD.init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", progress_f(7, 10) );
  //_HAL_BOARD.init_gps();

  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", progress_f(8, 10) );
  _HAL_BOARD.init_batterymon();

  // battery monitor initializing
  hal.console->printf("%.1f%%: Init range finder\n", progress_f(9, 10) );
  //_HAL_BOARD.init_rf();

  hal.console->printf("%.1f%%: Init inertial navigation\n", progress_f(10, 10) );
  _HAL_BOARD.init_inertial_nav();
}

void loop() {
  // send some json formatted information about the model over serial port
  _SCHED_NAV.run();
  _SCHED_OUT.run();
  
  // Attitude-, Altitude and Navigation control loop
  _MODEL.run();
}

AP_HAL_MAIN();











