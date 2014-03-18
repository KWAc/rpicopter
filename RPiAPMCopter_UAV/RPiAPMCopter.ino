////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Notify.h>
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
#include <AP_Baro.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_GPS.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>
////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "output.h"
#include "global.h"
#include "math.h"


inline void set_channels(int_fast16_t &pit, int_fast16_t &rol, int_fast16_t &yaw, int_fast16_t &thr);
inline void main_loop();
Task taskMain(&main_loop, MAIN_LOOP_T_MS, 1);

/*
 * Sets references to values in the eight channel rc input
 */
void set_channels(int_fast16_t &pit, int_fast16_t &rol, int_fast16_t &yaw, int_fast16_t &thr) {
  rol = _RECVR.m_pChannelsRC[0];
  pit = _RECVR.m_pChannelsRC[1];
  thr = _RECVR.m_pChannelsRC[2] > RC_THR_80P ? RC_THR_80P : _RECVR.m_pChannelsRC[2];
  yaw = _RECVR.m_pChannelsRC[3];
}

/*
 * Fast and time critical loop for:
 * - controlling the quadrocopter
 * - fetching rc signals
 * - filtering and processing sensor data necessary for flight
 */
void main_loop() {
  // additional filter or rc variables
  static float targ_yaw = 0.f; // yaw target from rc

  // Wait until new orientation data (normally 5 ms max)
  while(_INERT.wait_for_sample(INERT_TIMEOUT) == 0);

  // Handle all defined problems (time-outs, broken gyrometer, GPS signal ..)
  _EXCP.handle();
  
  // Variables to store remote control commands
  int_fast16_t rcthr, rcyaw, rcpit, rcrol;
  set_channels(rcpit, rcrol, rcyaw, rcthr);

  // Update sensor information
  _HAL_BOARD.update_inertial();
  Vector3f vAtti = _HAL_BOARD.get_atti_cor(); // returns the fused sensor value (gyrometer and accelerometer)
  Vector3f vGyro = _HAL_BOARD.get_gyro_cor();     // returns the sensor value from the gyrometer
  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stabilise PIDS
    float pit_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_STAB].get_pid((float)rcpit - vAtti.x, 1), -250, 250);
    float rol_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_STAB].get_pid((float)rcrol - vAtti.y, 1), -250, 250);
    float yaw_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_STAB].get_pid(wrap180_f(targ_yaw - vAtti.z), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = vAtti.z; // remember this yaw for when pilot stops
    }

    // rate PIDS
    int_fast16_t pit_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_RATE].get_pid(pit_stab_output - vGyro.x, 1), -500, 500);
    int_fast16_t rol_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_RATE].get_pid(rol_stab_output - vGyro.y, 1), -500, 500);
    int_fast16_t yaw_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_RATE].get_pid(yaw_stab_output - vGyro.z, 1), -500, 500);

    int_fast16_t fFL = rcthr + rol_output + pit_output - yaw_output;
    int_fast16_t fBL = rcthr + rol_output - pit_output + yaw_output;
    int_fast16_t fFR = rcthr - rol_output + pit_output + yaw_output;
    int_fast16_t fBR = rcthr - rol_output - pit_output - yaw_output;

    hal.rcout->write(MOTOR_FL, fFL);
    hal.rcout->write(MOTOR_BL, fBL);
    hal.rcout->write(MOTOR_FR, fFR);
    hal.rcout->write(MOTOR_BR, fBR);
  }
  else {
    // motors off
    hal.rcout->write(MOTOR_FL, RC_THR_OFF);
    hal.rcout->write(MOTOR_BL, RC_THR_OFF);
    hal.rcout->write(MOTOR_FR, RC_THR_OFF);
    hal.rcout->write(MOTOR_BR, RC_THR_OFF);

    // reset yaw target so we maintain this on take-off
    targ_yaw = vAtti.z;

    // reset PID integrals whilst on the ground
    for(uint_fast8_t i = 0; i < 6; i++) {
      _HAL_BOARD.m_pPIDS[i].reset_I();
    }
  }
}

double progress_f(uint_fast8_t iStep, uint_fast8_t iMax) {
  return (double)iStep*100.f/(double)iMax;
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED.addTask(&taskMain,  0);
  // .. and the sensor output functions
  _SCHED.addTask(&taskAtti,  75);
  _SCHED.addTask(&taskBaro,  1000);
  _SCHED.addTask(&taskGPS,   1000);
  _SCHED.addTask(&taskComp,  2000);
  _SCHED.addTask(&taskBat,   5000);
  _SCHED.addTask(&taskPID,   5000);

  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE_A); // USB
  hal.uartB->begin(BAUD_RATE_B); // GPS
  hal.uartC->begin(BAUD_RATE_C); // RADIO
  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", progress_f(1, 7) );
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", progress_f(2, 7) );
  _HAL_BOARD.init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", progress_f(3, 7) );
  _HAL_BOARD.init_barometer();

  hal.console->printf("%.1f%%: Init inertial sensor\n", progress_f(4, 7) );
  _HAL_BOARD.init_inertial();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", progress_f(5, 7) );
  _HAL_BOARD.init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", progress_f(6, 7) );
  _HAL_BOARD.init_gps();

  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", progress_f(7, 7) );
  _HAL_BOARD.init_batterymon();
}

void loop() { 
  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  _RECVR.try_uartAC();
  // send some json formatted information about the model over serial port
  _SCHED.run(); // Wrote my own small and absolutely fair scheduler
}

AP_HAL_MAIN();










