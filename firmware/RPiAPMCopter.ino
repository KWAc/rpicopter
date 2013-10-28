////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Notify.h>          // Notify library
#include <AP_InertialSensor_MPU6000.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GCS_MAVLink.h>
#include <AP_Compass.h>
#include <PID.h>

#include "RPiAPMCopterGlobals.h"
#include "RPiAPMCopterDefs.h"
#include "RPiAPMCopterFilter.h"
#include "RPiAPMCopterSensors.h"
#include "RPiAPMCopterCommon.h"
#include "RPiAPMCopterParser.h"


Vector3f attitude_calibration() {
  Vector3f offset;
  measure_attitude_offset(offset);

  GYRO_ROL_OFFS  = offset.x;
  GYRO_PIT_OFFS  = offset.y;
  GYRO_YAW_OFFS  = offset.z;

  hal.console->printf("Gyroscope calibrated - Offsets are roll:%f, pitch:%f, yaw:%f\n", GYRO_ROL_OFFS, GYRO_PIT_OFFS, GYRO_YAW_OFFS);
  return offset;
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
  PIDS[PID_PIT_RATE].kP(0.5);
  PIDS[PID_PIT_RATE].kI(0.5);
  PIDS[PID_PIT_RATE].imax(50);

  PIDS[PID_ROL_RATE].kP(0.5);
  PIDS[PID_ROL_RATE].kI(0.5);
  PIDS[PID_ROL_RATE].imax(50);

  PIDS[PID_YAW_RATE].kP(2.5);
  PIDS[PID_YAW_RATE].kI(0.5);
  PIDS[PID_YAW_RATE].imax(50);

  PIDS[PID_PIT_STAB].kP(4.5);
  PIDS[PID_ROL_STAB].kP(4.5);
  PIDS[PID_YAW_STAB].kP(5);

  // Turn off Barometer to avoid bus collisions
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  hal.console->printf("%.1f%%: Turn off barometer\n", 3.f*100.f/6.f);
  // we need to stop the barometer from holding the SPI bus
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);
#endif

  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  hal.console->printf("%.1f%%: Init inertial sensor\n", 4.f*100.f/6.f);
  inertial.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_200HZ);

  hal.console->printf("\n%.1f%%: Attitude calibration. Vehicle should stand on plane ground!\n", 5.f*100.f/6.f);
  inertial.update();
  attitude_calibration();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass\n", 6.f*100.f/6.f);
  if(!compass.init() ) {
    COMPASS_INITIALIZED = 0;
    hal.console->printf("Init compass failed!\n");
  }

  compass.accumulate();
  compass.motor_compensation_type(1);                              // throttle
  compass.set_offsets(0, 0, 0);                                    // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.f) );                            // set local difference between magnetic north and true north

  hal.console->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
  case AP_COMPASS_TYPE_HIL:
    hal.console->printf("HIL\n");
    break;
  case AP_COMPASS_TYPE_HMC5843:
    hal.console->println("HMC5843\n");
    break;
  case AP_COMPASS_TYPE_HMC5883L:
    hal.console->printf("HMC5883L\n");
    break;
  case AP_COMPASS_TYPE_PX4:
    hal.console->printf("PX4\n");
    break;
  default:
    hal.console->printf("unknown\n");
    break;
  }
  COMPASS_INITIALIZED = 1;
}

void loop() {
  static float filt_rol    = 0.f; // drift and other shit compensated Roll
  static float filt_pit    = 0.f; // drift and other shit compensated Pitch
  static float filt_yaw    = 0.f; // drift and other shit compensated Yaw

  static float targ_yaw    = 0.f; // yaw target from rc
  static float comp_yaw    = 0.f; // heading of the compass
  static float timer       = 0.f; // timer for gyro and compass used for drift compensation

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
  
  // Ask MPU6050's gyroscope for relative changes  in attitude
  float gyroRol, gyroPit, gyroYaw;
  get_gyroscope(gyroRol, gyroPit, gyroYaw);  // nice relative values

  // Ask MPU6050's accelerometer for attitude (absolute value)
  float attiRol, attiPit, attiYaw;
  get_attitude(attiRol, attiPit, attiYaw);

  // Remove offset for equal motor thrust at start (if ground is not totally even ..)
  attiRol -= GYRO_ROL_OFFS;
  attiPit -= GYRO_PIT_OFFS;

  // Compensate yaw drift a bit with the help of the compass    
  float time = hal.scheduler->millis() - timer;
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
      if(healthy && compass.use_for_yaw() )
        filt_yaw = sensor_fuse(filt_yaw, -comp_yaw, time, 0.125);
    }
  } 
  else {
    filt_yaw = sensor_fuse(filt_yaw, 0, time, 0.125);
  }
/*
  hal.console->printf("accroll:%.3f\troll:%.3f\taccpitch:%.3f\tpitch:%.3f\taccyaw:%.3f\tyaw:%.3f\n", 
   attiRol, filt_rol, 
   attiPit, filt_pit, 
   attiYaw, filt_yaw);  
*/
/*
  hal.console->printf("roll:%.3f\tfilt_roll:%.3f\tpitch:%.3f\tfilt_pitch:%.3f\tyaw:%.3f\tfilt_yaw:%.3f\tPit Rate:%f\tRol Rate:%f\n", 
   attiRol, filt_rol, 
   attiPit, filt_pit, 
   attiYaw, filt_yaw, 
   activation(attiPit), activation(attiRol));
*/
  // The save filtered attitude in proper variables for calculation of the motor values
  attiRol     = filt_rol;
  attiPit     = filt_pit;
  attiYaw     = filt_yaw;

  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stablise PIDS
    float rol_stab_output = constrain_float(PIDS[PID_ROL_STAB].get_pid((float)rcrol - attiRol, 1), -250, 250);
    float pit_stab_output = constrain_float(PIDS[PID_PIT_STAB].get_pid((float)rcpit - attiPit, 1), -250, 250); 
    float yaw_stab_output = constrain_float(PIDS[PID_YAW_STAB].get_pid(wrap_180(targ_yaw - attiYaw), 1), -360, 360);

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
/*
    hal.console->printf("Motor Out - attiRol:%f\tattiPit:%f\tattiYaw:%f\tFL:%f\tBL:%f\tFR:%f\tBR:%f\n", 
     attiRol, attiPit, attiYaw,
     fFL, fBL, fFR, fBR);
*/
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

AP_HAL_MAIN();









