////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Notify.h>
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
#include "extended_readouts.h"



////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////
inline void set_channels(int_fast32_t &, int_fast32_t &, int_fast32_t &, int_fast32_t &, int_fast32_t &);
inline void main_loop();
inline void navi_sensor_refr();
Task taskMain(&main_loop, MAIN_LOOP_T_MS, 1);
Task taskAlti(&navi_sensor_refr, ALTI_ESTIM_T_MS, 23);

//inline void hold_pos_xy(int_fast16_t &, int_fast16_t &, int_fast16_t &, int_fast16_t &, const int_fast32_t);
inline void hold_pos_z(int_fast16_t &, int_fast16_t &, int_fast16_t &, int_fast16_t &, const int_fast32_t);
//TODO: inline void hold_attitude();
//TODO: inline void follow_waypoint();

////////////////////////////////////////////////////////////////////////
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void navi_sensor_refr() {
  if(_RECVR.m_Waypoint.mode == GPSPosition::NOTHING_F) {
    return;
  }

  _HAL_BOARD.update_intertial_nav();
#ifdef SONAR_TYPE
  _HAL_BOARD.read_rf_m();
#endif
}

////////////////////////////////////////////////////////////////////////
// Saving the current controls from the remote control into references
////////////////////////////////////////////////////////////////////////
void set_channels(int_fast32_t &pit, int_fast32_t &rol, int_fast32_t &yaw, int_fast32_t &thr, 
int_fast32_t &alt_cm) 
{
  rol = _RECVR.m_rgChannelsRC[0];
  pit = _RECVR.m_rgChannelsRC[1];
  thr = _RECVR.m_rgChannelsRC[2] > RC_THR_80P ? RC_THR_80P : _RECVR.m_rgChannelsRC[2];
  yaw = _RECVR.m_rgChannelsRC[3];
  alt_cm = _RECVR.m_Waypoint.altitude_cm;
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void hold_pos_z(int_fast16_t &iFL, int_fast16_t &iBL, int_fast16_t &iFR, int_fast16_t &iBR, const int_fast32_t rcalt_cm) {
  const float fBias_g   = 0.25f;
  const float fScaleF_g = 100.0f;

  int_fast16_t iAltZOutput = 0; // Barometer & Sonar
  int_fast16_t iAccZOutput = 0; // Accelerometer
    
  if(_RECVR.m_Waypoint.mode == GPSPosition::NOTHING_F) {
    return;
  }

  // Return estimated altitude by GPS and barometer 
  bool bOK_H, bOK_R, bOK_G;
  int_fast32_t fCurAlti_cm = altitude_cm(&_HAL_BOARD, bOK_H);
  float fClimbRate_cms = climbrate_cms(&_HAL_BOARD, bOK_R);
  // Get the acceleration in g
  Vector3f vAccel_g = accel_g(&_HAL_BOARD, bOK_G) * fScaleF_g;
  
  if(!bOK_H || !bOK_R || !bOK_G) {
    return;
  }
  
  // Calculate the motor speed changes by the error from the height estimate and the current climb rates
  // If the quadro is going down, because of an device error, then this code is not used
  if(_RECVR.m_Waypoint.mode != GPSPosition::CONTRLD_DOWN_F) {
    float fAltZStabOut = _HAL_BOARD.m_rgPIDS[PID_THR_STAB].get_pid((float)(rcalt_cm - fCurAlti_cm), 1);
    iAltZOutput        = _HAL_BOARD.m_rgPIDS[PID_THR_RATE].get_pid(fAltZStabOut - fClimbRate_cms, 1);
  }
  
  // If the quad-copter is going down too fast, fAcceleration_g becomes greater
  if(_RECVR.m_Waypoint.mode == GPSPosition::CONTRLD_DOWN_F && vAccel_g.z > fBias_g) {
    _EXCP.pause_take_down();
  }
  
  // else: the fAcceleration_g becomes smaller
  if(_RECVR.m_Waypoint.mode == GPSPosition::CONTRLD_DOWN_F && vAccel_g.z <= fBias_g) {
    _EXCP.continue_take_down();
  }
  
  // Don't change the throttle if acceleration is below a certain bias
  if(abs(vAccel_g.z) >= fBias_g) {
    //vAccel_g.z         = sign_f(vAccel_g.z) * (abs(vAccel_g.z) - fBias_g) * fScaleF_g;
    float fAccZStabOut = _HAL_BOARD.m_rgPIDS[PID_ACC_STAB].get_pid(vAccel_g.z, 1);
    iAccZOutput        = _HAL_BOARD.m_rgPIDS[PID_ACC_RATE].get_pid(fAccZStabOut, 1);
  }

  // Modify the speed of the motors to hold the altitude
  iFL += iAltZOutput + iAccZOutput;
  iBL += iAltZOutput + iAccZOutput;
  iFR += iAltZOutput + iAccZOutput;
  iBR += iAltZOutput + iAccZOutput;
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

  // Variables to store remote control commands plus "rcalt" for the desired altitude in cm
  int_fast32_t rcpit, rcrol, rcyaw, rcthr, rcalt;
  set_channels(rcpit, rcrol, rcyaw, rcthr, rcalt);

  // Update sensor information
  _HAL_BOARD.update_inertial();
  Vector3f vAtti = _HAL_BOARD.get_atti_cor_deg(); // returns the fused sensor value (gyrometer and accelerometer)
  Vector3f vGyro = _HAL_BOARD.get_gyro_cor_deg(); // returns the sensor value from the gyrometer

  // Throttle raised, turn on stabilisation.
  if(rcthr > RC_THR_ACRO) {
    // Stabilise PIDS
    float pit_stab_output = constrain_float(_HAL_BOARD.m_rgPIDS[PID_PIT_STAB].get_pid((float)rcpit - vAtti.x, 1), -250, 250);
    float rol_stab_output = constrain_float(_HAL_BOARD.m_rgPIDS[PID_ROL_STAB].get_pid((float)rcrol - vAtti.y, 1), -250, 250);
    float yaw_stab_output = constrain_float(_HAL_BOARD.m_rgPIDS[PID_YAW_STAB].get_pid(wrap180_f(targ_yaw - vAtti.z), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = vAtti.z; // remember this yaw for when pilot stops
    }

    // rate PIDS
    int_fast16_t pit_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_rgPIDS[PID_PIT_RATE].get_pid(pit_stab_output - vGyro.x, 1), -500, 500);
    int_fast16_t rol_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_rgPIDS[PID_ROL_RATE].get_pid(rol_stab_output - vGyro.y, 1), -500, 500);
    int_fast16_t yaw_output = (int_fast16_t)constrain_float(_HAL_BOARD.m_rgPIDS[PID_YAW_RATE].get_pid(yaw_stab_output - vGyro.z, 1), -500, 500);

    int_fast16_t iFL = rcthr + rol_output + pit_output - yaw_output;
    int_fast16_t iBL = rcthr + rol_output - pit_output + yaw_output;
    int_fast16_t iFR = rcthr - rol_output + pit_output + yaw_output;
    int_fast16_t iBR = rcthr - rol_output - pit_output - yaw_output;

    // Hold the altitude
    hold_pos_z(iFL, iBL, iFR, iBR, rcalt);

    hal.rcout->write(MOTOR_FL, iFL);
    hal.rcout->write(MOTOR_BL, iBL);
    hal.rcout->write(MOTOR_FR, iFR);
    hal.rcout->write(MOTOR_BR, iBR);
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
      _HAL_BOARD.m_rgPIDS[i].reset_I();
    }
  }
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED.add_task(&taskMain,  0);  // no explanations ..
  _SCHED.add_task(&taskAlti,  0);
  // .. and the sensor output functions
  _SCHED.add_task(&taskAtti,  75);
  _SCHED.add_task(&taskBaro,  1000);
  _SCHED.add_task(&taskGPS,   1000);
  _SCHED.add_task(&taskComp,  2000);
  _SCHED.add_task(&taskBat,   5000);
  _SCHED.add_task(&taskPID,   5000);

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
  _HAL_BOARD.init_gps();

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
}

AP_HAL_MAIN();












