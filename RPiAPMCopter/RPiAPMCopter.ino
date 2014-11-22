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
#include <AP_Param.h>

#include <DataFlash.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <AC_PID.h>
#include <RC_Channel.h>     // RC Channel Library
#include <StorageManager.h>

#include "parameters.h"

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
inline void comp_loop(int); // compass loop for saving the offsets periodically

Task taskRCVR(&rcvr_loop, 0, 1);
Task taskINAV(&inav_loop, 0, 1);
Task taskRBat(&batt_loop, 0, 1);
Task taskCOMP(&comp_loop, 0, 1);


// EEPROM
const AP_Param::Info var_info[] PROGMEM = {
  GSCALAR(format_version, "SYSID_SW_MREV", 0),
  
  // PIDs
  GOBJECTN(_PIDS[PID_PIT_RATE], PIT_RATE, "PID_PIT_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_ROL_RATE], ROL_RATE, "PID_ROL_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_YAW_RATE], YAW_RATE, "PID_YAW_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_THR_RATE], THR_RATE, "PID_THR_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_ACC_RATE], ACC_RATE, "PID_ACC_RATE", AC_PID),
  GOBJECTN(_PIDS[PID_PIT_STAB], PIT_STAB, "PID_PIT_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_ROL_STAB], ROL_STAB, "PID_ROL_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_YAW_STAB], YAW_STAB, "PID_YAW_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_ACC_STAB], ACC_STAB, "PID_ACC_STAB", AC_PID),
  GOBJECTN(_PIDS[PID_THR_STAB], THR_STAB, "PID_THR_STAB", AC_PID),
  
  // Device objects
  GOBJECT(_COMP,        "COMPASS_",   Compass),
  GOBJECT(_INERT,       "INS_",       AP_InertialSensor),
  GOBJECT(_INERT_NAV,   "INAV_",      AP_InertialNav),
  GOBJECT(_AHRS,        "AHRS_",      AP_AHRS),
  GOBJECT(_BAT,         "BATT_",      AP_BattMonitor),
  GOBJECT(_BARO,        "GND_",       AP_Baro),
  GOBJECT(_GPS,         "GPS_",       AP_GPS),
  GOBJECT(_GPS_GLITCH,  "GPSGLITCH_", GPS_Glitch),
  GOBJECT(_BARO_GLITCH, "BAROGLTCH_", Baro_Glitch),
  GOBJECT(_SON_RF,      "RNGFND",     RangeFinder),

  AP_VAREND
};
// Setup the var_info table
AP_Param param_loader(var_info);

void load_settings() {
  if (!AP_Param::check_var_info() ) {
    hal.console->printf("Bad var table\n");
  }

  // change the default for the AHRS_GPS_GAIN for ArduCopter
  // if it hasn't been set by the user
  if (!_AHRS.gps_gain.load()) {
    _AHRS.gps_gain.set_and_save(1.0);
  }
  // disable centrifugal force correction, it will be enabled as part of the arming process
  _AHRS.set_correct_centrifugal(false);
  _AHRS.set_armed(false);

  // setup different AHRS gains for ArduCopter than the default
  // but allow users to override in their config
  if (!_AHRS._kp.load()) {
    _AHRS._kp.set_and_save(0.1);
  }
  if (!_AHRS._kp_yaw.load()) {
    _AHRS._kp_yaw.set_and_save(0.1);
  }

  // setup different Compass learn setting for ArduCopter than the default
  // but allow users to override in their config
  if (!_COMP._learn.load()) {
      _COMP._learn.set_and_save(0);
  }

  if (!g.format_version.load() || 
    g.format_version != Parameters::k_format_version) {
    // erase all parameters
    hal.console->printf("Firmware change: erasing EEPROM...\n");
    AP_Param::erase_all();
    // save the current format version
    g.format_version.set_and_save(Parameters::k_format_version);
    hal.console->printf("done\n");
  } else {
    uint32_t before = hal.scheduler->micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf("Load all settings took %d us\n", hal.scheduler->micros() - before);
  }
}

// Altitude estimation and AHRS system (yaw correction with GPS, barometer, ..)
void inav_loop(int) {  
  _HAL_BOARD.update_inav();
  _HAL_BOARD.read_rf_cm();
}

// Receiver thread (remote control), which should be limited to 50 Hz
void rcvr_loop(int) {
  _RECVR.try_any();
}

// save offsets if automatic offset learning is on
void comp_loop(int) {
  if(_HAL_BOARD.m_pComp->learn_offsets_enabled() ) {
    _HAL_BOARD.m_pComp->save_offsets();
  }
}

// Read the battery. 
// The voltage is used for adjusting the motor speed,
// as the speed is dependent on the voltage 
void batt_loop(int) {
  _HAL_BOARD.read_bat();
  _MODEL.calc_batt_comp();
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED_NAV.add_task(&taskRCVR, RCVR_T_MS);
  _SCHED_NAV.add_task(&taskINAV, INAV_T_MS);
  _SCHED_NAV.add_task(&taskRBat, BATT_T_MS);
  _SCHED_NAV.add_task(&taskCOMP, COMP_T_MS);
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
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", progress_f(1, 9) );
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // Load settings from EEPROM
  hal.console->printf("%.1f%%: Load settings from EEPROM\n", progress_f(2, 9) );
  load_settings();
  
  // Init the barometer
  hal.console->printf("%.1f%%: Init barometer\n", progress_f(3, 9) );
  _HAL_BOARD.init_barometer();

  // Init the accelerometer and gyrometer
  hal.console->printf("%.1f%%: Init inertial sensor\n", progress_f(4, 9) );
  _HAL_BOARD.init_inertial();

  // Init the compass
  hal.console->printf("%.1f%%: Init compass: ", progress_f(5, 9) );
  _HAL_BOARD.init_compass();

  // Init the GPS
  hal.console->printf("%.1f%%: Init GPS", progress_f(6, 9) );
  //_HAL_BOARD.init_gps();

  // Init the battery monitor
  hal.console->printf("\n%.1f%%: Init battery monitor\n", progress_f(7, 9) );
  _HAL_BOARD.init_batterymon();

  // Init the range finder
  hal.console->printf("%.1f%%: Init range finder\n", progress_f(8, 9) );
  //_HAL_BOARD.init_rf();

  // Init the inav system
  hal.console->printf("%.1f%%: Init inertial navigation\n", progress_f(9, 9) );
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












