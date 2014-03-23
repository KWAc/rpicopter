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
#include <AP_RangeFinder.h>
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
#include "extended_readouts.h"



////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////
inline void set_channels(int_fast32_t &, int_fast32_t &, int_fast32_t &, int_fast32_t &, int_fast32_t &);
inline void main_loop();
inline void alti_sensor_refr();
Task taskMain(&main_loop, MAIN_LOOP_T_MS, 1);
Task taskAlti(&alti_sensor_refr, ALTI_ESTIM_T_MS, 1);

inline void hold_altitude(int_fast16_t &, int_fast16_t &, int_fast16_t &, int_fast16_t &, const int_fast32_t);
//TODO: inline void hold_attitude();
//TODO: inline void follow_waypoint();

////////////////////////////////////////////////////////////////////////
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void alti_sensor_refr() {
  if(_RECVR.m_Waypoint.m_eMode == GPSPosition::NOTHING_F) {
    return;
  }
  
  _HAL_BOARD.read_gps();
  _HAL_BOARD.read_baro();
#ifdef SONAR_TYPE
  _HAL_BOARD.read_rf_m();
#endif
}

////////////////////////////////////////////////////////////////////////
// Saving the current controls from the remote control into references
////////////////////////////////////////////////////////////////////////
void set_channels(int_fast32_t &pit, int_fast32_t &rol, int_fast32_t &yaw, int_fast32_t &thr, 
int_fast32_t &alt_m) 
{
  rol = _RECVR.m_rgChannelsRC[0];
  pit = _RECVR.m_rgChannelsRC[1];
  thr = _RECVR.m_rgChannelsRC[2] > RC_THR_80P ? RC_THR_80P : _RECVR.m_rgChannelsRC[2];
  yaw = _RECVR.m_rgChannelsRC[3];
  alt_m = _RECVR.m_Waypoint.altitude_m;
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void hold_altitude(int_fast16_t &iFL, int_fast16_t &iBL, int_fast16_t &iFR, int_fast16_t &iBR, const int_fast32_t rcalt_m) {
  if(_RECVR.m_Waypoint.m_eMode == GPSPosition::NOTHING_F) {
    return;
  }

  // Return estimated altitude by GPS and barometer 
  bool bOK_H, bOK_R;
  float fCurAlti_cm       = altitude_m(&_HAL_BOARD, bOK_H) * 100.f;
  // Estimate current acceleration data
  float fClimbRate_cms    = climbrate_ms(&_HAL_BOARD, bOK_R) * 100.f;
  
  if(!bOK_H || !bOK_R) {
    return;
  }

  // Calculate the motor speed changes by the error from the height estimate and the current climb rates
  float fAltStabOut       = _HAL_BOARD.m_rgPIDS[PID_THR_STAB].get_pid(fCurAlti_cm - (float)(rcalt_m*100), 1);
  int_fast16_t iAltOutput = _HAL_BOARD.m_rgPIDS[PID_THR_RATE].get_pid(fAltStabOut - fClimbRate_cms, 1);

  // Modify the speed of the motors
  iFL += iAltOutput;
  iBL += iAltOutput;
  iFR += iAltOutput;
  iBR += iAltOutput;
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
    hold_altitude(iFL, iBL, iFR, iBR, rcalt);

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
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", progress_f(1, 8) );
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // PID Configuration
  hal.console->printf("%.1f%%: Set PID configuration\n", progress_f(2, 8) );
  _HAL_BOARD.init_pids();

  hal.console->printf("%.1f%%: Init barometer\n", progress_f(3, 8) );
  _HAL_BOARD.init_barometer();

  hal.console->printf("%.1f%%: Init inertial sensor\n", progress_f(4, 8) );
  _HAL_BOARD.init_inertial();

  // Compass initializing
  hal.console->printf("%.1f%%: Init compass: ", progress_f(5, 8) );
  _HAL_BOARD.init_compass();

  // GPS initializing
  hal.console->printf("%.1f%%: Init GPS", progress_f(6, 8) );
  _HAL_BOARD.init_gps();

  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init battery monitor\n", progress_f(7, 8) );
  _HAL_BOARD.init_batterymon();
  
  // battery monitor initializing
  hal.console->printf("\n%.1f%%: Init range finder\n", progress_f(8, 8) );
  _HAL_BOARD.init_rf();
}

void loop() { 
  // Commands via serial port (in this case WiFi -> RPi -> APM2.5)
  _RECVR.try_uartAC();
  // send some json formatted information about the model over serial port
  _SCHED.run(); // Wrote my own small and absolutely fair scheduler
}

AP_HAL_MAIN();












