#include <AP_InertialSensor_MPU6000.h>
#include <AP_InertialNav.h>

#include "rcframe.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "navigation.h"
#include "arithmetics.h"


////////////////////////////////////////////////////////////////////////
// Abstract class implementation
////////////////////////////////////////////////////////////////////////
Frame::Frame(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) {
  // Module pointers
  m_pHalBoard   = pDev;
  m_pReceiver   = pRecv;
  m_pExeption   = pExcp;
  m_pNavigation = pUAV;
  // Float initialization
  m_fRCRol      = 0.f;
  m_fRCPit      = 0.f;
  m_fRCYaw      = 0.f;
  m_fRCThr      = 0.f;
}

void Frame::read_receiver() {
  m_fRCRol = static_cast<float>(m_pReceiver->get_channel(RC_ROL) );
  m_fRCPit = static_cast<float>(m_pReceiver->get_channel(RC_PIT) );
  m_fRCThr = static_cast<float>(m_pReceiver->get_channel(RC_THR) > RC_THR_80P ? RC_THR_80P : m_pReceiver->get_channel(2) );
  m_fRCYaw = static_cast<float>(m_pReceiver->get_channel(RC_YAW) );
}

void Frame::run() {  
  // .. and update inertial information
  m_pHalBoard->update_attitude();
  
  // Handle all defined problems (time-outs, broken gyrometer, GPS signal ..)
  m_pExeption->handle();

  // Read from receiver module
  read_receiver();
  
  // Must get called before altitude hold calculation
  attitude_hold();
  altitude_hold();
  auto_navigate();
  
  // Give the compass the throttle information ..
  // .. after the motor input was post-processed by the functions above
  if(m_pHalBoard->m_pComp->healthy() ) {
    float fPercThr = (float)(m_fRCThr - RC_THR_OFF) / (float)(RC_THR_80P - RC_THR_OFF);
    m_pHalBoard->m_pComp->set_throttle(fPercThr);
  }
  
  // Output to the motors of the model
  servo_out();
}

////////////////////////////////////////////////////////////////////////
// Class implementation
////////////////////////////////////////////////////////////////////////
M4XFrame::M4XFrame(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) : Frame(pDev, pRecv, pExcp, pUAV)
{
  _FL = _BL = _FR = _BR = RC_THR_OFF;
  m_fBattComp = 0.f;
  m_fTiltComp = 0.f;
  m_iAccZTimer = m_iAltHTimer = pDev->m_pHAL->scheduler->millis();
}

void M4XFrame::servo_out() {
  m_pHalBoard->m_pHAL->rcout->write(MOTOR_FL, _FL);
  m_pHalBoard->m_pHAL->rcout->write(MOTOR_BL, _BL);
  m_pHalBoard->m_pHAL->rcout->write(MOTOR_FR, _FR);
  m_pHalBoard->m_pHAL->rcout->write(MOTOR_BR, _BR);
}

void M4XFrame::clear() {
  _FL = RC_THR_OFF;
  _BL = RC_THR_OFF;
  _FR = RC_THR_OFF;
  _BR = RC_THR_OFF;
}

void M4XFrame::set(int_fast16_t FL, int_fast16_t BL, int_fast16_t FR, int_fast16_t BR) {
  _FL = FL;
  _BL = BL;
  _FR = FR;
  _BR = BR;
}

void M4XFrame::add(int_fast16_t FL, int_fast16_t BL, int_fast16_t FR, int_fast16_t BR) {
  _FL += FL;
  _BL += BL;
  _FR += FR;
  _BR += BR;
}

void M4XFrame::calc_tilt_comp() {
  // For safety, always reset the correction term
  m_fTiltComp = 1.f;

  if( in_range(RC_ROL_MIN, RC_ROL_MAX, m_fRCRol) &&
      in_range(RC_PIT_MIN, RC_PIT_MAX, m_fRCPit) ) 
  {
    m_fTiltComp = 1.f / (cos(ToRad(m_fRCRol) ) * cos(ToRad(m_fRCPit) ) );
  }
}

void M4XFrame::calc_batt_comp() {
  // For safety, always reset the correction term
  m_fBattComp = 1.f;

  float fCurVoltage = m_pHalBoard->get_bat().voltage_V;
  if( m_pHalBoard->get_bat().refVoltage_V > 0.f && 
      in_range(BATT_MIN_VOLTAGE, BATT_MAX_VOLTAGE, fCurVoltage) )
  {
    m_fBattComp = m_pHalBoard->get_bat().refVoltage_V / fCurVoltage;
  }
}

void M4XFrame::apply_motor_compens() {
  calc_tilt_comp();
  //calc_batt_comp(); // Do it in the scheduler, because there the battery monitor is running

  float fCurThr = m_fRCThr - RC_THR_ACRO;
  // Calculate new throttle output (tilt and battery compensated)
  float fCompThr = fCurThr * (m_fTiltComp * m_fBattComp) + RC_THR_ACRO;
  m_fRCThr = fCompThr <= RC_THR_80P ? fCompThr : RC_THR_80P;
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for autonomous flight mode with:
// * GPS auto-navigation
// * Overrides the remote control for controlling the quad
////////////////////////////////////////////////////////////////////////
void M4XFrame::auto_navigate() {
  // Break this function if there was not the proper UAV-mode set
  if(!chk_fset(m_pReceiver->get_waypoint()->mode, GPSPosition::GPS_NAVIGATN_F) ) {
    return;
  }
  // Break this function if the compass is not configured
  if(!m_pHalBoard->m_pComp->configured() ) {
    return;
  }
  
  // Set yaw in remote control
  m_pReceiver->set_channel(RC_YAW, m_pNavigation->calc_yaw() );
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void M4XFrame::altitude_hold() {
  // Do only change thrust if copter is armed!
#if !BENCH_OUT 
  if(m_fRCThr < RC_THR_ACRO) {
    return;
  }
#endif
  
  const float fScaleF_g2cmss = 100.f * INERT_G_CONST;
  
  int_fast16_t iAltZOutput = 0; // Barometer & Sonar
  int_fast16_t iAccZOutput = 0; // Accelerometer

  // Calc current g-force
  bool bOK_G;
  float fAccel_g = Device::get_accel_z_g(m_pHalBoard, bOK_G); // Get the acceleration in g
  
  if(!bOK_G) {
    return;
  }
    
  // Limit altitude control if in remote control mode
  bool bUseAltHold = m_pReceiver->get_waypoint()->mode == GPSPosition::NOTHING_F ? false : true;
  // Limit this loop to 50 Hz
  uint_fast32_t iHldAltTime_ms = m_pHalBoard->m_pHAL->scheduler->millis() - m_iAltHTimer;
  if(iHldAltTime_ms >= HLD_ALTITUDE_TIMER) {
    m_iAltHTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  } else {
    bUseAltHold = false;
  }
  
  // Height regulation using the barometer and/or sonar
  if(bUseAltHold) { 
    bool bOK_H;
    float fTargAlti_cm  = static_cast<float>(m_pReceiver->get_waypoint()->altitude_cm);
    float fCurrAlti_cm  = Device::get_altitude_cm(m_pHalBoard, bOK_H);
    float fClmbRate_cms = m_pHalBoard->m_pInertNav->get_velocity_z();
    
    if(!bOK_H) {
      return;
    }

    // Calculate the motor speed changes by the error from the height estimate and the current climb rates
    // If the quadro is going down, because of an device error, then this code is not used
    if(m_pReceiver->get_waypoint()->mode != GPSPosition::CONTRLD_DOWN_F) {
      float fAltZStabOut = constrain_float(m_pHalBoard->get_pid(PID_THR_STAB).get_pid(fTargAlti_cm - fCurrAlti_cm, 1), -250, 250);
      iAltZOutput        = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid(PID_THR_RATE).get_pid(fAltZStabOut - fClmbRate_cms, 1), -500, 500) );
    }

    if(m_pReceiver->get_channel(RC_ROL) > RC_THR_OFF) {
      // If the quad-copter is going down too fast, fAcceleration_g becomes greater
      if(m_pReceiver->get_waypoint()->mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g > HLD_ALTITUDE_ZGBIAS) {
        m_pExeption->pause_take_down();
      }

      // else: the fAcceleration_g becomes smaller
      if(m_pReceiver->get_waypoint()->mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g <= HLD_ALTITUDE_ZGBIAS) {
        m_pExeption->cont_take_down();
      }
    }
  }
  
  // Small & fast stabilization using the accelerometer
  static short iLAccSign = 0; 
  if(fabs(fAccel_g) >= HLD_ALTITUDE_ZGBIAS) {
    if(iLAccSign == 0) {
      iLAccSign = sign_f(fAccel_g);
    }
    
    // The g-force must act for a minimum time interval before the PID can be used
    uint_fast32_t iAccZTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iAccZTimer;
    if(iAccZTime < HLD_ALTITUDE_ZTBIAS) {
       return; 
    }
    
    // Check whether the direction of acceleration changed suddenly
    // If so: reset the timer
    short iCAccSign = sign_f(fAccel_g);
    if(iCAccSign != iLAccSign) {
      // Reset the switch if acceleration becomes normal again
      m_iAccZTimer = m_pHalBoard->m_pHAL->scheduler->millis();
      // Reset the PID integrator
      m_pHalBoard->get_pid(PID_ACC_RATE).reset_I();
      // Save last sign
      iLAccSign = iCAccSign;
      return;
    }
    
    // Feed the current acceleration into the PID regulator
    float fAccZ_cmss = sign_f(fAccel_g) * (fabs(fAccel_g) - HLD_ALTITUDE_ZGBIAS) * fScaleF_g2cmss;
    iAccZOutput = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid(PID_ACC_RATE).get_pid(-fAccZ_cmss, 1), -500, 500) );
    
#if DEBUG_OUT
    m_pHalBoard->m_pHAL->console->printf("altitude_hold() - iAccZOutput: %d\n", iAccZOutput);
#endif
  } else {
    // Reset the switch if acceleration becomes normal again
    m_iAccZTimer = m_pHalBoard->m_pHAL->scheduler->millis();
    // Reset the PID integrator
    m_pHalBoard->get_pid(PID_ACC_RATE).reset_I();
  }

  // Modify the speed of the motors to hold the altitude
  int_fast16_t iFL = iAltZOutput + iAccZOutput;
  int_fast16_t iBL = iAltZOutput + iAccZOutput;
  int_fast16_t iFR = iAltZOutput + iAccZOutput;
  int_fast16_t iBR = iAltZOutput + iAccZOutput;

  add(iFL, iBL, iFR, iBR);
}

/*
 * Fast and time critical loop for:
 * - controlling the quadrocopter
 * - fetching RC signals
 * - filtering and processing sensor data necessary for flight
 */
void M4XFrame::attitude_hold() {
  // additional filter or rc variables
  static float targ_yaw = 0.f; // Yaw target from RC
  
  Vector3f vAtti = m_pHalBoard->get_atti_cor_deg(); // returns the fused sensor value (gyrometer and accelerometer)
  Vector3f vGyro = m_pHalBoard->get_gyro_degps();   // returns the sensor value from the gyrometer
#if !BENCH_OUT 
  // Throttle raised, turn on stabilisation.
  if(m_fRCThr >= RC_THR_ACRO) {
#endif
    // Stabilise PIDS
    float pit_stab_output = constrain_float(m_pHalBoard->get_pid(PID_PIT_STAB).get_pid(m_fRCPit - vAtti.x, 1), -250, 250);
    float rol_stab_output = constrain_float(m_pHalBoard->get_pid(PID_ROL_STAB).get_pid(m_fRCRol - vAtti.y, 1), -250, 250);
    float yaw_stab_output = constrain_float(m_pHalBoard->get_pid(PID_YAW_STAB).get_pid(wrap180_f(targ_yaw - vAtti.z), 1), -360, 360);

    // Is pilot asking for yaw change? - If so, feed directly to rate PID (overwriting yaw stab output)
    if(fabs(m_fRCYaw ) > 5.f) {
      yaw_stab_output = m_fRCYaw;
      targ_yaw = vAtti.z; // remember this yaw for when pilot stops
    }

    // Rate PIDS
    int_fast16_t pit_output = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid(PID_PIT_RATE).get_pid(pit_stab_output - vGyro.x, 1), -500, 500) );
    int_fast16_t rol_output = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid(PID_ROL_RATE).get_pid(rol_stab_output - vGyro.y, 1), -500, 500) );
    int_fast16_t yaw_output = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->get_pid(PID_YAW_RATE).get_pid(yaw_stab_output - vGyro.z, 1), -500, 500) );

    // Apply: tilt- and battery-compensation algorithms
    apply_motor_compens();

    // Calculate the speed of the motors
    int_fast16_t iFL = m_fRCThr + rol_output + pit_output - yaw_output;
    int_fast16_t iBL = m_fRCThr + rol_output - pit_output + yaw_output;
    int_fast16_t iFR = m_fRCThr - rol_output + pit_output + yaw_output;
    int_fast16_t iBR = m_fRCThr - rol_output - pit_output - yaw_output;

    set(iFL, iBL, iFR, iBR);
#if !BENCH_OUT 
  } else {
    // Clear motor output
    clear();
    // reset yaw target so we maintain this on take-off
    targ_yaw = vAtti.z;
    // reset PID integrals whilst on the ground
    for(uint_fast8_t i = 0; i < NR_OF_PIDS; i++) {
      m_pHalBoard->get_pid(i).reset_I();
    }
  }
#endif
}
