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
#include <GCS_MAVLink.h> // Rediculous dependency to AP_InertialSensor_MPU6000
#include <DataFlash.h>   // Rediculous dependency to AP_InertialSensor_MPU6000
#include <AP_Declination.h>
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


/*
 * Sets references to values in the eight channel rc input
 */
inline void set_channels(int16_t &pit, int16_t &rol, int16_t &yaw, int16_t &thr) {
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
inline void main_loop() {
  // additional filter or rc variables
  static float targ_yaw = 0.f; // yaw target from rc

  // Wait until new orientation data (normally 5 ms max)
  while(_INERT.wait_for_sample(INERTIAL_TIMEOUT) == 0);

  // Variables to store remote control commands
  int16_t rcthr, rcyaw, rcpit, rcrol;
  set_channels(rcpit, rcrol, rcyaw, rcthr);

  // Reduce throttle if no update for more than 500 ms
  uint32_t packet_t = _RECVR.time_elapsed(); // Measure time elapsed since last successful package from WiFi or radio
  if(packet_t > SER_PKT_TIMEOUT && rcthr > RC_THR_OFF) {
    // how much to reduce?
    float fDecr = 1.25 * ((float)packet_t / 25.f);
    int16_t fDelta = rcthr - (int16_t)fDecr;
    // reduce thrust..
    rcthr = (int16_t)fDecr < 0 ? RC_THR_OFF : fDelta > RC_THR_MIN ? fDelta : RC_THR_OFF;
    // reset yaw, pitch and roll
    rcyaw = 0; // yaw
    rcpit = 0; // pitch
    rcrol = 0; // roll
  }

  // Update sensor information
  _HAL_BOARD.update_inertial();
  Vector3f vAttitude = _HAL_BOARD.get_atti(); // returns the fused sensor value (gyrometer and accelerometer)
  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stabilise PIDS
    float pit_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_STAB].get_pid((float)rcpit - vAttitude.x, 1), -250, 250);
    float rol_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_STAB].get_pid((float)rcrol - vAttitude.y, 1), -250, 250);
    float yaw_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_STAB].get_pid(wrap180_float(targ_yaw - vAttitude.z), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = vAttitude.z; // remember this yaw for when pilot stops
    }

    // rate PIDS
    int16_t pit_output = (int16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_RATE].get_pid(pit_stab_output - _HAL_BOARD.m_vGyro.x, 1), -500, 500);
    int16_t rol_output = (int16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_RATE].get_pid(rol_stab_output - _HAL_BOARD.m_vGyro.y, 1), -500, 500);
    int16_t yaw_output = (int16_t)constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_RATE].get_pid(yaw_stab_output - _HAL_BOARD.m_vGyro.z, 1), -500, 500);

    int16_t fFL = rcthr + rol_output + pit_output - yaw_output;
    int16_t fBL = rcthr + rol_output - pit_output + yaw_output;
    int16_t fFR = rcthr - rol_output + pit_output + yaw_output;
    int16_t fBR = rcthr - rol_output - pit_output - yaw_output;

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
    targ_yaw = vAttitude.z;

    // reset PID integrals whilst on the ground
    for(int i = 0; i < 6; i++) {
      _HAL_BOARD.m_pPIDS[i].reset_I();
    }
  }
}

void setup() {
  // Prepare scheduler
  _SCHED.addFastEmitter(&emitAtti);
  _SCHED.addMediEmitter(&emitBaro);
  _SCHED.addMediEmitter(&emitGPS);
  _SCHED.addSlowEmitter(&emitComp);
  _SCHED.addUslwEmitter(&emitBat);
  _SCHED.addUslwEmitter(&emitPID);

  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE_A); // USB
  hal.uartB->begin(BAUD_RATE_B); // GPS
  hal.uartC->begin(BAUD_RATE_C); // RADIO
  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", 1.f*100.f/7.f);
  for(uint16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", 2.f*100.f/7.f);
  _HAL_BOARD.init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", 3.f*100.f/7.f);
  _HAL_BOARD.init_barometer();

  hal.console->printf("%.1f%%: Init inertial sensor\n", 4.f*100.f/7.f);
  _HAL_BOARD.init_inertial();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", 5.f*100.f/7.f);
  _HAL_BOARD.init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", 6.f*100.f/7.f);
  _HAL_BOARD.init_gps();

  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", 7.f*100.f/7.f);
  _HAL_BOARD.init_batterymon();
}

void loop() {
  static unsigned long timer = 0;
  
  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  bool bOK = _RECVR.read_uartA(hal.console->available() ); 	// Try WiFi (serial) first
  if(!bOK) {
    _RECVR.read_uartC(hal.uartC->available() ); 	        // If not working: Try radio next
  }
  
  // Main loop runs at 100 Hz
  unsigned long time = hal.scheduler->millis() - timer;
  if(time > MAIN_LOOP_T_MS) {
    main_loop();        // time critical stuff
    timer = hal.scheduler->millis();
  }

  // send some json formatted information about the model over serial port
  _SCHED.run(); // Wrote my own small and absolutely fair scheduler
}

AP_HAL_MAIN();










