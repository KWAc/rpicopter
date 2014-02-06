////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
//#include <stdlib.h>
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
#include <AP_BattMonitor.h>
////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "BattMonitor.h"
#include "globals.h"
#include "defines.h"
#include "filter.h"
#include "sensors.h"
#include "math.h"
#include "parser.h"
#include "output.h"
#include "setup.h"


/*
 * Sets references to values in the eight channel rc input
 */ 
inline void set_channels(float &pit, float &rol, float &yaw, float &thr) {
  thr = (float)RC_CHANNELS[2] > RC_THR_80P ? RC_THR_80P : (float)RC_CHANNELS[2]; 
  yaw = (float)RC_CHANNELS[3];
  pit = (float)RC_CHANNELS[1];
  rol = (float)RC_CHANNELS[0];
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
  while(inertial.wait_for_sample(INERTIAL_TIMEOUT) == 0);
  
  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  parse_input(hal.console->available() ); 	// Parse incoming text
  
  // Variables to store remote control commands
  float rcthr, rcyaw, rcpit, rcrol;
  set_channels(rcpit, rcrol, rcyaw, rcthr);
  
  // Reduce throttle if no update for more than 500 ms
  uint32_t packet_t = hal.scheduler->millis() - iWiFiTimer;
  if(packet_t > 500 && rcthr > RC_THR_OFF) {
    // Try via radio signal
    bool bRecv = radio_rc();
    if(bRecv == true) {
      set_channels(rcpit, rcrol, rcyaw, rcthr);
    } else { // Nothing worked ..
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
  }
  
  // Update sensor information
  inertial.update();
  // Ask MPU6050's gyroscope for changes in attitude
  float gyroRol, gyroPit, gyroYaw;
  get_gyroscope(gyroRol, gyroPit, gyroYaw);
  // Ask MPU6050's accelerometer for attitude
  float attiRol, attiPit, attiYaw;
  get_attitude(attiRol, attiPit, attiYaw);
  // Remove offset for equal motor thrust at start (if ground is not totally even ..)
  attiRol -= GYRO_ROL_OFFS;
  attiPit -= GYRO_PIT_OFFS;

  // On-flight correction for drift(s) to any side(s)
  // only if the user was sending some valid values(different from zero)
  if( GYRO_ROL_COR || GYRO_PIT_COR) {
    attiRol += GYRO_ROL_COR;
    attiPit += GYRO_PIT_COR;
  }
  
  // Compensate yaw drift a bit with the help of the compass    
  uint32_t time = timer != 0 ? hal.scheduler->millis() - timer : INERTIAL_TIMEOUT;
  timer = hal.scheduler->millis();

  // Calculate absolute attitude from relative gyrometer changes
  filt_rol += gyroRol * (float)time/1000.f;
  filt_rol = wrap_180(filt_rol);

  filt_pit += gyroPit * (float)time/1000.f;
  filt_pit = wrap_180(filt_pit);

  filt_yaw += gyroYaw * (float)time/1000.f;
  filt_yaw = wrap_180(filt_yaw);

  /* 
   * Fuse these values with the data from the accelerometer
   * If the accelerometer gives values near zero (equilibrium) the annealing rate is big because vehicle is not moving much, 
   * otherwise the rate is becomes very small very fast.
   * This minimizes noise from acceleration/deceleration to a minimum.
   */
  filt_rol = sensor_fuse(filt_rol, attiRol, time, activation(attiRol, 20.f) );
  filt_pit = sensor_fuse(filt_pit, attiPit, time, activation(attiPit, 20.f) );

  // For yaw changes the compass could be used, otherwise anneal to zero
  if(COMPASS_FOR_YAW) {
    if(COMPASS_INITIALIZED) {
      float fThrPerc = (rcthr - RC_THR_MIN) / (RC_THR_80P - RC_THR_MIN);
      compass.set_throttle(fThrPerc);

      bool healthy = get_compass_heading(comp_yaw, 0, 0);
      OUT_HEADING = comp_yaw;
      if(healthy && compass.use_for_yaw() )
        filt_yaw = sensor_fuse(filt_yaw, -comp_yaw, time, 0.125);
    }
  } 
  else {
    filt_yaw = sensor_fuse(filt_yaw, 0, time, 0.125);
  }

  // The save filtered attitude in proper variables for calculation of the motor values
  OUT_ROL = filt_rol;  OUT_PIT = filt_pit;  OUT_YAW = filt_yaw;

  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stablise PIDS
    float rol_stab_output = constrain_float(PIDS[PID_ROL_STAB].get_pid(rcrol - OUT_ROL, 1), -250, 250);
    float pit_stab_output = constrain_float(PIDS[PID_PIT_STAB].get_pid(rcpit - OUT_PIT, 1), -250, 250); 
    float yaw_stab_output = constrain_float(PIDS[PID_YAW_STAB].get_pid(wrap_180(targ_yaw - OUT_YAW), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = attiYaw; // remember this yaw for when pilot stops
    }

    // rate PIDS
    float rol_output = constrain_float(PIDS[PID_ROL_RATE].get_pid(rol_stab_output - gyroRol, 1), -500, 500);  
    float pit_output = constrain_float(PIDS[PID_PIT_RATE].get_pid(pit_stab_output - gyroPit, 1), -500, 500);  
    float yaw_output = constrain_float(PIDS[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

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
    targ_yaw = attiYaw;

    // reset PID integrals whilst on the ground
    for(int i = 0; i < 6; i++) {
      PIDS[i].reset_I();
    }
  }
}

/* 
 * NOT time critical loop for: 
 * - sending general information over the serial port
 * - Potentially slow calculations, logging and printing output should be done here
 */
// pEmitters: Array of iSize_N elements
// iTickrate: the time in ms until the first emitter in the array will emit again
inline void scheduler(Emitter **pEmitters, uint16_t iSize_N, uint32_t &iTimer, const uint16_t &iTickRate) { 
  uint32_t time = hal.scheduler->millis() - iTimer;
  for(uint16_t i = 0; i < iSize_N; i++) {
    if(time > iTickRate + pEmitters[i]->getDelay(i) ) { 
      if(pEmitters[i]->emit() ) {
        if(i == (iSize_N - 1) ) { // Reset everything if last emitter successfully emitted
          for(uint16_t i = 0; i < iSize_N; i++) {
            pEmitters[i]->reset();
          }
          iTimer = hal.scheduler->millis();
        }
      }
    }
  }
}

void setup() {
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
  init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", 3.f*100.f/7.f);
  init_baro();
  
  hal.console->printf("%.1f%%: Init inertial sensor\n", 4.f*100.f/7.f);
  init_inertial();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", 5.f*100.f/7.f);
  init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", 6.f*100.f/7.f);
  init_gps();
  
  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", 7.f*100.f/7.f);
  init_batterymon();
}

Emitter *fast_emitters[1] = { &emitAtti };
Emitter *medi_emitters[2] = { &emitBaro, &emitGPS };
Emitter *slow_emitters[1] = { &emitComp };
Emitter *uslw_emitters[2] = { &emitPIDS, &emitBatt };

void loop() {
  // remote control and quadro control loop
  main_loop();        // time critical stuff
  
  // send some json formatted information about the model over serial port
  scheduler(fast_emitters, 1, iFastTimer, 75);
  scheduler(medi_emitters, 2, iMediTimer, 1000);
  scheduler(slow_emitters, 1, iSlowTimer, 2000);
  scheduler(uslw_emitters, 2, iUslwTimer, 5000);
}

AP_HAL_MAIN();









