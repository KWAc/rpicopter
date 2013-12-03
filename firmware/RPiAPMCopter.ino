////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Notify.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GCS_MAVLink.h>
#include <AP_Declination.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <PID.h>
////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "globals.h"
#include "defines.h"
#include "filter.h"
#include "sensors.h"
#include "math.h"
#include "parser.h"
#include "output.h"
#include "setup.h"


/* 
 * Fast and time critical loop for: 
 * - controlling the quadrocopter 
 * - fetching rc signals
 * - filtering and processing sensor data necessary for flight
 */
inline void fast_loop() {
  static float filt_rol    = 0.f; // drift and other shit compensated Roll
  static float filt_pit    = 0.f; // drift and other shit compensated Pitch
  static float filt_yaw    = 0.f; // drift and other shit compensated Yaw

  static float targ_yaw    = 0.f; // yaw target from rc
  static float comp_yaw    = 0.f; // heading of the compass
  static int timer         = 0; // timer for gyro and compass used for drift compensation

  // Wait until new orientation data (normally 5 ms max)
  while(inertial.sample_available() == 0);

  // serial bytes available?
  int bytesAvailable = hal.console->available();
  parse_input(bytesAvailable); 	// Parse incoming text

  // Reduce throttle stepwise if no update for more than 500 ms
  if(hal.scheduler->millis() - RC_PACKET_T > 500) {
    // reduce throttle ..
    if(RC_CHANNELS[2] > RC_THR_MIN) {
      RC_CHANNELS[2] -= 1.25;
    }
    else { // .. and switch it off
      RC_CHANNELS[2] = RC_THR_OFF;
    }
    // reset yaw, pitch and roll
    RC_CHANNELS[3] = 0; // yaw
    RC_CHANNELS[1] = 0; // pitch
    RC_CHANNELS[0] = 0; // roll
    hal.scheduler->delay(25); // Wait 25 ms 
  }

  // Variables to store radio in 
  long rcthr, rcyaw, rcpit, rcrol;
  // Read RC transmitter 
  rcthr = RC_CHANNELS[2]; 
  rcyaw = RC_CHANNELS[3];
  rcpit = RC_CHANNELS[1];
  rcrol = RC_CHANNELS[0];

  // Set an upper limit for the throttle (80% of maximum)
  // to allow to counter regulate if copter is changing angle
  if(rcthr > RC_THR_80P) {
    rcthr = RC_THR_80P;
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

  // Compensate yaw drift a bit with the help of the compass    
  int time = hal.scheduler->millis() - timer;
  timer = hal.scheduler->millis();

  // Calculate absolute attitude from relative gyrometer changes
  filt_rol += gyroRol * time/1000;
  filt_rol = wrap_180(filt_rol);

  filt_pit += gyroPit * time/1000;
  filt_pit = wrap_180(filt_pit);

  filt_yaw += gyroYaw * time/1000;
  filt_yaw = wrap_180(filt_yaw);

  /* 
   * Fuse these values with the data from the accelerometer
   * If the accelerometer gives values near zero (equilibrium) the annealing rate is big because vehicle is not moving much, 
   * otherwise the rate is becomes very small very fast.
   * This minimizes noise from acceleration/deceleration to a minimum.
   */
  filt_rol = sensor_fuse(filt_rol, attiRol, time, activation(attiRol, 20) );
  filt_pit = sensor_fuse(filt_pit, attiPit, time, activation(attiPit, 20) );

  // For yaw changes the compass could be used, otherwise anneal to zero
  if(COMPASS_FOR_YAW) {
    if(COMPASS_INITIALIZED) {
      float fThrPerc = ((float)rcthr - RC_THR_MIN) / (RC_THR_80P - RC_THR_MIN);
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
    float rol_stab_output = constrain_float(PIDS[PID_ROL_STAB].get_pid((float)rcrol - OUT_ROL, 1), -250, 250);
    float pit_stab_output = constrain_float(PIDS[PID_PIT_STAB].get_pid((float)rcpit - OUT_PIT, 1), -250, 250); 
    float yaw_stab_output = constrain_float(PIDS[PID_YAW_STAB].get_pid(wrap_180(targ_yaw - OUT_YAW), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      targ_yaw = attiYaw; // remember this yaw for when pilot stops
    }

    // rate PIDS
    float rol_output = constrain_float(PIDS[PID_ROL_RATE].get_pid(rol_stab_output - gyroRol, 1), -500, 500);  
    float pit_output = constrain_float(PIDS[PID_PIT_RATE].get_pid(pit_stab_output - gyroPit, 1), -500, 500);  
    float yaw_output = constrain_float(PIDS[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

    float fFL = (float)rcthr + rol_output + pit_output - yaw_output;
    float fBL = (float)rcthr + rol_output - pit_output + yaw_output;
    float fFR = (float)rcthr - rol_output + pit_output + yaw_output;
    float fBR = (float)rcthr - rol_output - pit_output - yaw_output;

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
 * Slow and NOT time critical loop for: 
 * - sending general information over the serial port
 * - Potentially slow calculations, logging and printing output should be done here
 */
 inline void medium_loop() {
  static int timer = 0;
  int time = hal.scheduler->millis() - timer;
  
  // send every 0.25 s
  if(time > 250) {
    send_attitude(OUT_PIT, OUT_ROL, OUT_YAW);
    
    OUT_BARO = get_baro();
    send_baro(OUT_BARO);

    timer = hal.scheduler->millis();
  }
}
 
inline void slow_loop() {
  static int timer = 0;
  int time = hal.scheduler->millis() - timer;
  
  // send every 1 s
  if(time > 1000) {
    send_comp(OUT_HEADING);
    
    timer = hal.scheduler->millis();
  }
}

inline void very_slow_loop() {
  static int timer = 0;
  int time = hal.scheduler->millis() - timer;
  
  // send every 5 s
  if(time > 5000) {
    send_pids();
    
    timer = hal.scheduler->millis();
  }
}

void setup() {
  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE);
  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", 1.f*100.f/6.f);
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", 2.f*100.f/6.f);
  init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", 3.f*100.f/6.f);
  init_baro();
  
  hal.console->printf("%.1f%%: Init inertial sensor\n", 4.f*100.f/6.f);
  init_inertial();

  hal.console->printf("\n%.1f%%: Attitude calibration. Vehicle should stand on plane ground!\n", 5.f*100.f/6.f);
  attitude_calibration();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", 6.f*100.f/6.f);
  init_compass();
}

void loop() {
  // remote control and quadro control loop
  fast_loop();        // time critical stuff
  
  // send some json formatted information about the model over serial port
  medium_loop();      // barometer and attitude
  slow_loop();        // compass
  very_slow_loop();   // general configuration (e.g. PIDs) of the copter
}

AP_HAL_MAIN();









