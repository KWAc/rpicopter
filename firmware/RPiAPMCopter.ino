#include <AP_InertialSensor.h>
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

#include "RPiAPMCopterDefs.h"
#include "RPiAPMCopterFilter.h"
#include "RPiAPMCopterSensors.h"
#include "RPiAPMCopterCommon.h"


//checksum verifier
uint8_t verify_chksum(char *str, char *chk) {
  hal.console->printf(str);
  hal.console->printf("\n");

  uint8_t nc = 0;
  for(int i=0; i<strlen(str); i++) 
    nc = (nc + str[i]) << 1;

  long chkl = strtol(chk, NULL, 16);                    // supplied chksum to long
  if(chkl == (long)nc)                                  // compare
    return true;

  return false;
}

// Parse incoming text
void parse_input(int &bytesAvail) {
  static uint32_t offset = 0;
  char buffer[32];
  memset(buffer, 0, sizeof(buffer) );

  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)hal.console->read();                 // read next byte
    if(c == '\n') {                                     // new line reached - process cmd
      buffer[offset] = '\0';                            // null terminator
      // process cmd
      char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
      char *chk = strtok(NULL, "*");                    // chk = chksum

      if(verify_chksum(str, chk)) {                     // if chksum OK
        char *ch = strtok(str, ",");                    // first channel
        RC_CHANNELS[0] = (uint16_t)strtol(ch, NULL, 10);   // parse       
        for(int i = 1; i < APM_IOCHANNEL_COUNT; i++) {  // loop through final 3 RC_CHANNELS
          char *ch = strtok(NULL, ",");
          RC_CHANNELS[i] = (uint16_t)strtol(ch, NULL, 10);   
        }
        RC_PACKET_T = hal.scheduler->millis();             // update last valid packet
        memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
      } 
      else {
        hal.console->printf("Invalid checksum\n");
        memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
      }
      offset = 0;
    }
    else if(c != '\r' && offset < sizeof(buffer)-1) {
      buffer[offset++] = c;                             // store in buffer and continue until newline
    }
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
  PIDS[PID_PITCH_RATE].kP(0.7);
  PIDS[PID_PITCH_RATE].kI(1);
  PIDS[PID_PITCH_RATE].imax(50);

  PIDS[PID_ROLL_RATE].kP(0.7);
  PIDS[PID_ROLL_RATE].kI(1);
  PIDS[PID_ROLL_RATE].imax(50);

  PIDS[PID_YAW_RATE].kP(2.7);
  PIDS[PID_YAW_RATE].kI(1);
  PIDS[PID_YAW_RATE].imax(50);

  PIDS[PID_PITCH_STAB].kP(4.5);
  PIDS[PID_ROLL_STAB].kP(4.5);
  PIDS[PID_YAW_STAB].kP(10);

  // Turn off Barometer to avoid bus collisions
  hal.console->printf("%.1f%%: Turn off barometer\n", 3.f*100.f/6.f);
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);

  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  hal.console->printf("%.1f%%: Calibrate gyros\n", 4.f*100.f/6.f);
  inertial.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_200HZ, NULL);

  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  hal.console->printf("\n%.1f%%: Initialize MPU6050\n", 5.f*100.f/6.f);
  hal.scheduler->suspend_timer_procs();  // stop bus collisions
  inertial.dmp_init();
  hal.scheduler->resume_timer_procs();

  // Compass initializing
  hal.console->printf("%.1f%%: Compass initialization\n", 6.f*100.f/6.f);
  if(!compass.init() ) {
    hal.console->printf("Compass initialisation failed!\n");
    COMPASS_AVAIL = 0;
  }

  compass.set_offsets(0, 0, 0);                                     // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0) );                             // set local difference between magnetic north and true north

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
}

void loop() {
  static float    yaw_target  = 0;  

  // Calibrate gyro initially for 15s
  if(GYRO_CALI == true) {
    Vector3f drift, offset;
    int samples = 0;
    measure_gyro_drift(drift, offset, samples);
    if(samples > 15) { // Number of samples
      GYRO_ROL_DRIFT = drift.x;
      GYRO_PIT_DRIFT = drift.y;
      GYRO_YAW_DRIFT = drift.z + 0.5f;
      
      GYRO_ROL_OFFS  = offset.x;
      GYRO_PIT_OFFS  = offset.y;
      GYRO_YAW_OFFS  = offset.z;

      hal.console->printf("Gyroscope calibrated - Drifts are roll:%f, pitch:%f, yaw:%f\n", GYRO_ROL_DRIFT, GYRO_PIT_DRIFT, GYRO_YAW_DRIFT);
      GYRO_CALI = false;
    }
  } else {
    // Wait until new orientation data (normally 5 ms max)
    while(inertial.num_samples_available() == 0);

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
    long rcthr, rcyaw, rcpit, rcroll;
    // Read RC transmitter 
    rcthr  = RC_CHANNELS[2]; 
    rcyaw  = RC_CHANNELS[3];
    rcpit  = RC_CHANNELS[1];
    rcroll = RC_CHANNELS[0];

    // Set an upper limit for the throttle (80% of maximum)
    // to allow to counter regulate if copter is changing angle
    if(rcthr > RC_THR_80P) {
      rcthr = RC_THR_80P;
    }

    // Update sensor information
    inertial.update();
    // Ask MPU6050 for altitude (absolute value)
    float altiRoll, altiPitch, altiYaw;  
    get_altitude(altiRoll, altiPitch, altiYaw);
    // Ask MPU6050 for gyroscope for relative changes
    float gyroPitch, gyroRoll, gyroYaw;
    get_gyroscope(gyroPitch, gyroRoll, gyroYaw);

    // Compensate yaw drift a bit via high and low path filtering with the help of the compass
    float darol = 0;
    float dapit = 0;
    float dayaw = 0;
    
    static float filt_rol = 0;
    static float filt_pit = 0;
    static float filt_yaw = 0;
    
    static float last_rol = 0;
    static float last_pit = 0;
    static float last_yaw = 0;
    
    static float heading  = 0; 
    static float timer    = 0;

    float time            = hal.scheduler->millis() - timer;
    dayaw = drift_filter(filt_yaw, altiYaw, last_yaw, GYRO_YAW_DRIFT, time);

    if(COMPASS_AVAIL) {
      bool healthy = get_compass_heading(heading, 0, 0);
      if(healthy)
        filt_yaw = sensor_fuse(filt_yaw, heading, time, 0.0125);
      timer = hal.scheduler->millis();
    }
    
    last_rol    = altiRoll;
    last_pit    = altiPitch;
    last_yaw    = altiYaw;
    
    altiRoll   -= GYRO_ROL_OFFS; 
    altiPitch  -= GYRO_PIT_OFFS;
    altiYaw     = filt_yaw;
    
    //hal.console->printf("Gyroscope - roll:%.3f\tpitch:%.3f\tyaw:%.3f\theading:%f\t\n", altiRoll, altiPitch, altiYaw, heading);
    
    // Throttle raised, turn on stabilisation.
    if(rcthr > RC_THR_ACRO) {
      // Stablise PIDS
      float roll_stab_output  = constrain_float(PIDS[PID_ROLL_STAB].get_pid((float)rcroll - altiRoll, 1), -250, 250);
      float pitch_stab_output = constrain_float(PIDS[PID_PITCH_STAB].get_pid((float)rcpit - altiPitch, 1), -250, 250); 
      float yaw_stab_output   = constrain_float(PIDS[PID_YAW_STAB].get_pid(wrap_180(yaw_target - altiYaw), 1), -360, 360);

      // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
      if(abs(rcyaw ) > 5) {
        yaw_stab_output = rcyaw;
        yaw_target = altiYaw; // remember this yaw for when pilot stops
      }

      // rate PIDS
      long roll_output  =  (long)constrain_float(PIDS[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
      long pitch_output =  (long)constrain_float(PIDS[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
      long yaw_output   =  (long)constrain_float(PIDS[PID_ROLL_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

      // mix pid outputs and send to the motors.
      hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
      hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
      hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
      hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
    } 
    else {
      // motors off
      hal.rcout->write(MOTOR_FL, RC_THR_OFF);
      hal.rcout->write(MOTOR_BL, RC_THR_OFF);
      hal.rcout->write(MOTOR_FR, RC_THR_OFF);
      hal.rcout->write(MOTOR_BR, RC_THR_OFF);

      // reset yaw target so we maintain this on take-off
      yaw_target = altiYaw;

      // reset PID integrals whilst on the ground
      for(int i = 0; i < 6; i++) {
        PIDS[i].reset_I();
      }
    }
  }
}

AP_HAL_MAIN();






