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
inline void set_channels(float &pit, float &rol, float &yaw, float &thr) {
  thr = (float)_RECVR.m_pChannelsRC[2] > RC_THR_80P ? RC_THR_80P : (float)_RECVR.m_pChannelsRC[2]; 
  yaw = (float)_RECVR.m_pChannelsRC[3];
  pit = (float)_RECVR.m_pChannelsRC[1];
  rol = (float)_RECVR.m_pChannelsRC[0];
}

/* 
 * Fast and time critical loop for: 
 * - controlling the quadrocopter 
 * - fetching rc signals
 * - filtering and processing sensor data necessary for flight
 */
inline void main_loop() {
  static float filt_rol = 0.f; // drift compensated Roll
  static float filt_pit = 0.f; // drift compensated Pitch
  static float filt_yaw = 0.f; // drift compensated Yaw
  // additional filter or rc variables
  static float targ_yaw = 0.f; // yaw target from rc
  static float comp_yaw = 0.f; // heading of the compass
  // timer for gyro and compass used for drift compensation
  static uint32_t timer = 0;

  // Wait until new orientation data (normally 5 ms max)
  while(_INERT.wait_for_sample(INERTIAL_TIMEOUT) == 0);

  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  bool bOK = _RECVR.read_uartA(hal.console->available() ); 	// Try WiFi (serial) first
  if(!bOK) {
    _RECVR.read_uartC(hal.uartC->available() ); 	        // If not working: Try radio next
  }

  // Variables to store remote control commands
  float rcthr, rcyaw, rcpit, rcrol;
  set_channels(rcpit, rcrol, rcyaw, rcthr);

  // Reduce throttle if no update for more than 500 ms
  uint32_t packet_t = hal.scheduler->millis() - _RECVR.timeElapsed(); // Measure time elapsed since last successful package from WiFi or radio
  if(packet_t > 500 && rcthr > RC_THR_OFF) {
    // how much to reduce?
    float fDecr = 1.25 * ((float)packet_t / 25.f);
    float fDelta = rcthr - fDecr;    
    // reduce thrust..
    rcthr = fDecr < 0 ? RC_THR_OFF : fDelta > RC_THR_MIN ? fDelta : RC_THR_OFF;
    // reset yaw, pitch and roll
    rcyaw = 0.f; // yaw
    rcpit = 0.f; // pitch
    rcrol = 0.f; // roll
  }

  // Update sensor information
  _INERT.update();
  // Ask MPU6050's gyroscope for changes in attitude
  Vector3f gyro = _HAL_BOARD.read_gyro(); // x=PITCH, y=ROLL, z=YAW
  // Ask MPU6050's accelerometer for attitude
  Vector3f atti = _HAL_BOARD.read_atti(); // x=PITCH, y=ROLL, z=YAW

  // Compensate yaw drift a bit with the help of the compass    
  uint32_t time = timer != 0 ? hal.scheduler->millis() - timer : INERTIAL_TIMEOUT;
  timer = hal.scheduler->millis();

  // Calculate absolute attitude from relative gyrometer changes
  filt_pit += gyro.x * (float)time/1000.f;
  filt_pit = wrap_180(filt_pit);
  
  filt_rol += gyro.y * (float)time/1000.f;
  filt_rol = wrap_180(filt_rol);

  filt_yaw += gyro.z * (float)time/1000.f;
  filt_yaw = wrap_180(filt_yaw);

  /* 
   * Fuse these values with the data from the accelerometer
   * If the accelerometer gives values near zero (equilibrium) the annealing rate is big because vehicle is not moving much, 
   * otherwise the rate is becomes very small very fast.
   * This minimizes noise from acceleration/deceleration to a minimum.
   */
  filt_pit = sensor_fuse(filt_pit, atti.x, time, activation(atti.x, 20.f) );
  filt_rol = sensor_fuse(filt_rol, atti.y, time, activation(atti.y, 20.f) );

  // For yaw changes the compass could be used, otherwise anneal to zero
  if(COMPASS_FOR_YAW) {
    float fThrPerc = (rcthr - RC_THR_MIN) / (RC_THR_80P - RC_THR_MIN);
    _COMP.set_throttle(fThrPerc);

    float comp_yaw = _HAL_BOARD.read_comp(0, 0);
    if(_COMP.use_for_yaw() ) {
      filt_yaw = sensor_fuse(filt_yaw, -comp_yaw, time, 0.125);
    }
  } 
  else {
    filt_yaw = sensor_fuse(filt_yaw, 0, time, 0.125);
  }

  _HAL_BOARD.m_fRol = filt_rol;
  _HAL_BOARD.m_fPit = filt_pit;
  _HAL_BOARD.m_fYaw = filt_yaw;

  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stablise PIDS
    float pit_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_STAB].get_pid(rcpit - filt_pit, 1), -250, 250); 
    float rol_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_STAB].get_pid(rcrol - filt_rol, 1), -250, 250);
    float yaw_stab_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_STAB].get_pid(wrap_180(targ_yaw - filt_yaw), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = atti.z; // remember this yaw for when pilot stops
    }

    // rate PIDS
    float pit_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_PIT_RATE].get_pid(pit_stab_output - gyro.x, 1), -500, 500); 
    float rol_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_ROL_RATE].get_pid(rol_stab_output - gyro.y, 1), -500, 500);   
    float yaw_output = constrain_float(_HAL_BOARD.m_pPIDS[PID_YAW_RATE].get_pid(yaw_stab_output - gyro.z, 1), -500, 500);  

    float fFL = rcthr + rol_output + pit_output - yaw_output;
    float fBL = rcthr + rol_output - pit_output + yaw_output;
    float fFR = rcthr - rol_output + pit_output + yaw_output;
    float fBR = rcthr - rol_output - pit_output - yaw_output;

    // mix pid outputs and send to the motors.
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
    targ_yaw = atti.z;

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
  // remote control and quadro control loop
  main_loop();        // time critical stuff

  // send some json formatted information about the model over serial port
  _SCHED.run();
}

AP_HAL_MAIN();










