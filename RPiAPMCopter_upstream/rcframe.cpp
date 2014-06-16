#include <AP_InertialSensor_MPU6000.h>
#include <AP_InertialNav.h>
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>

#include "rcframe.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "navigation.h"
#include "arithmetics.h"


////////////////////////////////////////////////////////////////////////
// A helper function:
// Saving the current controls from the remote control into references
////////////////////////////////////////////////////////////////////////
inline void set_channels(const Receiver *pRecvr, float &pit, float &rol, float &yaw, float &thr) {
  rol = static_cast<float>(pRecvr->m_rgChannelsRC[RC_ROL]);
  pit = static_cast<float>(pRecvr->m_rgChannelsRC[RC_PIT]);
  thr = static_cast<float>(pRecvr->m_rgChannelsRC[RC_THR] > RC_THR_80P ? RC_THR_80P : pRecvr->m_rgChannelsRC[2]);
  yaw = static_cast<float>(pRecvr->m_rgChannelsRC[RC_YAW]);
}

////////////////////////////////////////////////////////////////////////
// Abstract class implementation
////////////////////////////////////////////////////////////////////////
Frame::Frame(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) {
  m_pHalBoard   = pDev;
  m_pReceiver   = pRecv;
  m_pExeption   = pExcp;
  m_pNavigation = pUAV;
}


////////////////////////////////////////////////////////////////////////
// Class implementation
////////////////////////////////////////////////////////////////////////
M4XFrame::M4XFrame(Device *pDev, Receiver *pRecv, Exception *pExcp, UAVNav* pUAV) : Frame(pDev, pRecv, pExcp, pUAV)
{  
  m_pMotors = new AP_MotorsQuad(*(m_pReceiver->m_pRCRol), 
                                *(m_pReceiver->m_pRCPit), 
                                *(m_pReceiver->m_pRCThr), 
                                *(m_pReceiver->m_pRCYaw) );
}

void M4XFrame::init_motors() {
  // motor initialisation
  m_pMotors->set_update_rate(490);
  m_pMotors->set_frame_orientation(AP_MOTORS_X_FRAME);
  m_pMotors->set_min_throttle(RC_THR_ACRO-RC_THR_OFF);
  m_pMotors->set_mid_throttle((RC_THR_MAX-RC_THR_OFF)/2);
  m_pMotors->Init();      // initialise motors

  m_pMotors->enable();
  m_pMotors->output_min();
}

void M4XFrame::out() {  
  m_pMotors->set_pitch(_iPitOut);
  m_pMotors->set_roll(_iRolOut);
  m_pMotors->set_yaw(_iYawOut);
  m_pMotors->set_throttle(_iThrOut);
  m_pMotors->output();
}

void M4XFrame::clear() {
  _iPitOut = _iYawOut = _iRolOut = _iThrOut = 0;  
  out();
  m_pMotors->armed(false);
}

void M4XFrame::run() {
  // Must get called before altitude hold calculation
  calc_attitude_hold();
  calc_altitude_hold();
  calc_gpsnavig_hold();
  
  // Output to the motors of the model
  out();
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for autonomous flight mode with:
// * GPS auto-navigation
// * Overrides the remote control for controlling the quad
////////////////////////////////////////////////////////////////////////
void M4XFrame::calc_gpsnavig_hold() {
  // Break this function if there was not the proper UAV-mode set
  if(!chk_fset(m_pReceiver->m_Waypoint.mode, GPSPosition::GPS_NAVIGATN_F) ) {
    return;
  }
  
  // Set yaw in remote control
  m_pReceiver->m_rgChannelsRC[RC_YAW] = m_pNavigation->calc_yaw();
}

////////////////////////////////////////////////////////////////////////
// Implementations
// This function is only needed for (semi-)autonomous flight mode like:
// * Hold altitude
// * GPS auto-navigation
////////////////////////////////////////////////////////////////////////
void M4XFrame::calc_altitude_hold() {
  const float fBias_g   = 0.50f;
  const float fScaleF_g = 100.0f;

  int_fast16_t iAltZOutput = 0; // Barometer & Sonar
  int_fast16_t iAccZOutput = 0; // Accelerometer

  // return if in standard remote control mode
  if(m_pReceiver->m_Waypoint.mode == GPSPosition::NOTHING_F) {
    return;
  }

  // Return estimated altitude by GPS and barometer
  bool bOK_H, bOK_G;
  float fTargAlti_cm  = static_cast<float>(m_pReceiver->m_Waypoint.altitude_cm);
  float fCurrAlti_cm  = Device::get_altitude_cm(m_pHalBoard, bOK_H);
  float fClmbRate_cms = m_pHalBoard->m_pInertNav->get_velocity_z();
  // Get the acceleration in g
  float fAccel_g      = Device::get_accel_z_g(m_pHalBoard, bOK_G) * fScaleF_g;

  if(!bOK_H || !bOK_G) {
    return;
  }

  // Calculate the motor speed changes by the error from the height estimate and the current climb rates
  // If the quadro is going down, because of an device error, then this code is not used
  if(m_pReceiver->m_Waypoint.mode != GPSPosition::CONTRLD_DOWN_F) {
    float fAltZStabOut = m_pHalBoard->m_rgPIDS[PID_THR_STAB].get_pid(fTargAlti_cm - fCurrAlti_cm, 1);
    iAltZOutput        = m_pHalBoard->m_rgPIDS[PID_THR_RATE].get_pid(fAltZStabOut - fClmbRate_cms, 1);
  }

  if(m_pReceiver->m_rgChannelsRC[RC_ROL] > RC_THR_OFF) {
    // If the quad-copter is going down too fast, fAcceleration_g becomes greater
    if(m_pReceiver->m_Waypoint.mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g > fBias_g) {
      m_pExeption->pause_take_down();
    }

    // else: the fAcceleration_g becomes smaller
    if(m_pReceiver->m_Waypoint.mode == GPSPosition::CONTRLD_DOWN_F && fAccel_g <= fBias_g) {
      m_pExeption->continue_take_down();
    }
  }

  // Don't change the throttle if acceleration is below a certain bias
  if(fabs(fAccel_g) >= fBias_g) {
    //fAccel_g         = sign_f(fAccel_g) * (abs(fAccel_g) - fBias_g) * fScaleF_g;
    float fAccZStabOut = m_pHalBoard->m_rgPIDS[PID_ACC_STAB].get_pid(fAccel_g, 1);
    iAccZOutput        = m_pHalBoard->m_rgPIDS[PID_ACC_RATE].get_pid(fAccZStabOut, 1);
  }
  
  // Calculate new motor output
  _iThrOut += iAltZOutput + iAccZOutput;
}

/*
 * Fast and time critical loop for:
 * - controlling the quadrocopter
 * - fetching rc signals
 * - filtering and processing sensor data necessary for flight
 */
void M4XFrame::calc_attitude_hold() {
  // additional filter or rc variables
  static float targ_yaw = 0.f; // yaw target from rc

  // Wait if there is no new data (save ressources)
  while(!m_pHalBoard->m_pInert->wait_for_sample(MAIN_T_MS) );

  // Handle all defined problems (time-outs, broken gyrometer, GPS signal ..)
  m_pExeption->handle();

  // Variables to store remote control commands plus "rcalt" for the desired altitude in cm
  float rcpit, rcrol, rcyaw, _iThrOut;
  set_channels(m_pReceiver, rcpit, rcrol, rcyaw, _iThrOut);

  // Update sensor information
  m_pHalBoard->update_attitude();
  Vector3f vAtti = m_pHalBoard->get_atti_cor_deg(); // returns the fused sensor value (gyrometer and accelerometer)
  Vector3f vGyro = m_pHalBoard->get_gyro_cor_deg(); // returns the sensor value from the gyrometer

  // Throttle raised, turn on stabilisation.
  if(_iThrOut > RC_THR_ACRO) {
    // Stabilise PIDS
    float pit_stab_output = constrain_float(m_pHalBoard->m_rgPIDS[PID_PIT_STAB].get_pid(rcpit - vAtti.x, 1), -250, 250);
    float rol_stab_output = constrain_float(m_pHalBoard->m_rgPIDS[PID_ROL_STAB].get_pid(rcrol - vAtti.y, 1), -250, 250);
    float yaw_stab_output = constrain_float(m_pHalBoard->m_rgPIDS[PID_YAW_STAB].get_pid(wrap180_f(targ_yaw - vAtti.z), 1), -360, 360);

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(fabs(rcyaw ) > 5.f) {
      yaw_stab_output = rcyaw;
      targ_yaw = vAtti.z; // remember this yaw for when pilot stops
    }

    // rate PIDS
    _iPitOut = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->m_rgPIDS[PID_PIT_RATE].get_pid(pit_stab_output - vGyro.x, 1), -500, 500) );
    _iRolOut = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->m_rgPIDS[PID_ROL_RATE].get_pid(rol_stab_output - vGyro.y, 1), -500, 500) );
    _iYawOut = static_cast<int_fast16_t>(constrain_float(m_pHalBoard->m_rgPIDS[PID_YAW_RATE].get_pid(yaw_stab_output - vGyro.z, 1), -500, 500) );
    _iThrOut -= RC_THR_OFF; // Remote control output ==>  to motor input range
    m_pMotors->armed(true);
  }
  else {
    clear();

    // reset yaw target so we maintain this on take-off
    targ_yaw = vAtti.z;

    // reset PID integrals whilst on the ground
    for(uint_fast8_t i = 0; i < NR_OF_PIDS; i++) {
      m_pHalBoard->m_rgPIDS[i].reset_I();
    }
  }
}
