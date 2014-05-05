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


////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
inline void main_loop();
inline void inav_loop();
Task taskINAV(&inav_loop, INAV_T_MS, 1);

// Attitude-, Altitude and Navigation control loop
void main_loop() {
  static uint_fast16_t timer = 0;
  uint_fast16_t time = _HAL_BOARD.m_pHAL->scheduler->millis();
  if(time - timer >= _RECVR.get_update_rate_ms() ) {
    _MODEL.run();
    timer = time;
  }
}

// Altitude estimation and AHRS system (yaw correction with GPS, barometer, ..)
void inav_loop() {  
  _HAL_BOARD.update_inav();
#ifdef SONAR_TYPE
  _HAL_BOARD.read_rf_m();
#endif
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED.add_task(&taskINAV, 0);  // Inertial, GPS, Compass, Barometer sensor fusions (slow) ==> running at 50 Hz
  // .. and the sensor output functions
  _SCHED.add_task(&outAtti,  75);
  _SCHED.add_task(&outBaro,  1000);
  _SCHED.add_task(&outGPS,   1000);
  _SCHED.add_task(&outComp,  1500);
  _SCHED.add_task(&outBat,   1750);
  _SCHED.add_task(&outPIDAtt,2000);
  _SCHED.add_task(&outPIDAlt,2000);

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
  //_HAL_BOARD.init_gps();

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
  // Don't use the scheduler for the time critical main loop (~20% faster)
  main_loop();
}

AP_HAL_MAIN();












