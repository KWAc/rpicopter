#include <AP_InertialSensor.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GCS_MAVLink.h>

#include <PID.h>
#include "RPiAPMCopterDefs.h"


float wrap_180(float x) {
  return x < -180 ? (x + 360) : (x > 180 ? (x - 360) : x);
}

//checksum verifier
uint8_t verify_chksum(char *str, char *chk) {
  hal.console->printf(str);
  hal.console->printf("\n");

  uint8_t nc = 0;
  for(int i=0; i<strlen(str); i++) 
    nc = (nc + str[i]) << 1;

  long chkl = strtol(chk, NULL, 16); 	// supplied chksum to long
  if(chkl == (long)nc)   				// compare
    return true;

  return false;
}

// Parse incoming text
inline
void ParseInput(int &bytesAvail) {
  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)hal.console->read(); // read next byte
    if(c == '\n') { // new line reached - process cmd
      COMBUF[COMBUFOFFSET] = '\0'; // null terminator
      // process cmd
      char *str = strtok(COMBUF, "*"); // str = roll, pit, thr, yaw
      char *chk = strtok(NULL, "*"); // chk = chksum

      if(verify_chksum(str, chk)) { // if chksum OK
        char *ch = strtok(str, ","); // first channel
        CHANNELS[0] = (uint16_t)strtol(ch, NULL, 10); // parse       
        for(int i = 1; i < APM_IOCHANNEL_COUNT; i++) { // loop through final 3 channels
          char *ch = strtok(NULL, ",");
          CHANNELS[i] = (uint16_t)strtol(ch, NULL, 10);   
        }
        LSTPKT = hal.scheduler->millis(); // update last valid packet
        memset(COMBUF, 0, sizeof(COMBUF)); // flush buffer after everything
      } 
      else {
        hal.console->printf("Invalid chksum\n");
        memset(COMBUF, 0, sizeof(COMBUF)); // flush buffer after everything
      }
      COMBUFOFFSET = 0;
    }
    else if(c != '\r') {
      COMBUF[COMBUFOFFSET++] = c; 							// store in buffer and continue until newline
    }
  }
}

void setup() {
  // Set baud rate when connected to RPi
  //hal.uartA->begin(57600);
  hal.uartA->begin(115200);

  hal.console->printf("Setup device ..\n");

  // Enable the motors and set at 490Hz update
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", 1.f*100.f/5.f);
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", 2.f*100.f/5.f);
  pids[PID_PITCH_RATE].kP(0.7);
  pids[PID_PITCH_RATE].kI(1);
  pids[PID_PITCH_RATE].imax(50);

  pids[PID_ROLL_RATE].kP(0.7);
  pids[PID_ROLL_RATE].kI(1);
  pids[PID_ROLL_RATE].imax(50);

  pids[PID_YAW_RATE].kP(2.7);
  pids[PID_YAW_RATE].kI(1);
  pids[PID_YAW_RATE].imax(50);

  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);

  // Turn off Barometer to avoid bus collisions
  hal.console->printf("%.1f%%: Turn off barometer\n", 3.f*100.f/5.f);
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);

  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  hal.console->printf("%.1f%%: Calibrate gyros\n", 4.f*100.f/5.f);
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, NULL);

  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  hal.console->printf("\n%.1f%%: Initialize MPU6050\n", 5.f*100.f/5.f);
  hal.scheduler->suspend_timer_procs();  // stop bus collisions
  ins.dmp_init();
  hal.scheduler->resume_timer_procs();
  
  // initialize command buffer
  memset(COMBUF, 0, sizeof(COMBUF));
}

void loop() {
  static float yaw_target = 0;  
  // Wait until new orientation data (normally 5 ms max)
  while (ins.num_samples_available() == 0);

  // serial bytes available?
  int bytesAvailable = hal.console->available();
  ParseInput(bytesAvailable); 	// Parse incoming text

  // turn throttle off if no update for 0.5seconds
  /*if(hal.scheduler->millis() - LSTPKT > 500) {
    CHANNELS[2] = RC_THR_MIN;
  }*/

  long rcthr, rcyaw, rcpit, rcroll; 									// Variables to store radio in 
  // Read RC transmitter 
  rcthr 	= CHANNELS[2]; 
  rcyaw 	= CHANNELS[3];
  rcpit 	= CHANNELS[1];
  rcroll 	= CHANNELS[0];
  // Set an upper limit for the settable throttle (80% of maximum)
  // to allow to counter regulate if copter is changing angle
  /*if(rcthr > RC_THR_80P) {
   rcthr = RC_THR_80P;
   }*/

  // Ask MPU6050 for orientation
  ins.update();
  float roll, pitch, yaw;  
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  roll 	= ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw 	= ToDeg(yaw) ;

  // Ask MPU6050 for gyro data
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);

  // Do the magic
  if(rcthr > RC_THR_MIN) { 										// Throttle raised, turn on stablisation.
    // Stablise PIDS
    float pitch_stab_output     = constrain_float(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    float roll_stab_output 	= constrain_float(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output 	= constrain_float(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw; 												// remember this yaw for when pilot stops
    }

    // rate PIDS
    long pitch_output 	=  (long) constrain_float(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
    long roll_output 	=  (long) constrain_float(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    long yaw_output 	=  (long) constrain_float(pids[PID_ROLL_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

    // mix pid outputs and send to the motors.
    hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
  } 
  else {
    // motors off
    hal.rcout->write(MOTOR_FL, RC_THR_MIN);
    hal.rcout->write(MOTOR_BL, RC_THR_MIN);
    hal.rcout->write(MOTOR_FR, RC_THR_MIN);
    hal.rcout->write(MOTOR_BR, RC_THR_MIN);

    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;

    // reset PID integrals whilst on the ground
    for(int i = 0; i < 6; i++) {
      pids[i].reset_I();
    }
  }
}

AP_HAL_MAIN();




