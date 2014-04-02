#include <AP_Math.h>
#include <AP_InertialNav.h>
#include <math.h>

#include "navigation.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "math.h"
#include "filter.h"

#define M_2PI               (2.f * M_PI)
#define M_PERIMETER_EARTH_M (40074000.f)
#define M_RADIUS_EARTH_M    (M_PERIMETER_EARTH_M / M_2PI)
#define M_AP_INT2FLOAT_DEG  (10000000.f)

/*
 * Sigmoid transfer function
 */
inline float uav_yaw_f(float x, float mod){
  float val = sign_f(x) * smaller_f(abs(mod * x), 180.f) / 180.f;
  return val / sqrt(1 + pow2_f(val) );
}

inline float uav_zero_f(float x, float mod){
  float val = (180.0 - smaller_f(abs(mod * x), 179.9f)) / 180.f;
  return val / sqrt(1 + pow2_f(val) );
}

UAVNav::UAVNav(Device *pDev, Receiver *pRecv, Exception *pExcp) {
  m_pHalBoard      = pDev;
  m_pReceiver      = pRecv;
  m_pExeption      = pExcp;
  
  m_t32YawTimer    = m_pHalBoard->m_pHAL->scheduler->millis();
  
  m_fTargetYaw_deg = 0.f;
  m_iTargetPit_deg = 0;
  m_dX = m_dY      = 0.f;
}

/*
 * Geographic calculations
 */
float UAVNav::width_of_merid_m(const float fLat_deg) const {  
  return (M_2PI * M_RADIUS_EARTH_M * cos(ToRad(fLat_deg) ) ) / 360.f;
}

float UAVNav::dist_2_equat_m(const float fLat_deg) const {
  return (M_PERIMETER_EARTH_M / 360.f) * fLat_deg;
}

float UAVNav::dist_2_greenw_m(const float fLat_deg, const float fLon_deg) const {    
  return width_of_merid_m(fLat_deg) * fLon_deg;
}

float UAVNav::calc_error_deg() { 
  float fLatHome_deg = (float)m_pHalBoard->m_pInertNav->get_latitude() / M_AP_INT2FLOAT_DEG;
  float fLonHome_deg = (float)m_pHalBoard->m_pInertNav->get_longitude() / M_AP_INT2FLOAT_DEG;
  
  float fLatTarg_deg = (float)m_pReceiver->m_Waypoint.latitude / M_AP_INT2FLOAT_DEG;
  float fLonTarg_deg = (float)m_pReceiver->m_Waypoint.longitude / M_AP_INT2FLOAT_DEG;
  
  float fXHome = dist_2_greenw_m(fLatHome_deg, fLonHome_deg);
  float fYHome = dist_2_equat_m(fLatHome_deg);
  
  float fXTarg = dist_2_greenw_m(fLatTarg_deg, fLonTarg_deg);
  float fYTarg = dist_2_equat_m(fLatTarg_deg);
  
  m_dX = fXTarg - fXHome;
  m_dY = fYTarg - fYHome;
  
  return ToDeg(atan2(m_dX, m_dY) ) - m_pHalBoard->get_comp_deg();
}

void UAVNav::run() {
  // Calculate the time since last call
  uint_fast32_t t32CurTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  float dT = (float)(t32CurTimer - m_t32YawTimer) / 1000.f;
  m_t32YawTimer = t32CurTimer;
  
  // Update position data and calculate the errors
  float fError_deg = calc_error_deg();
  
  // Calculate the error dependent modifier, using a normed sigmoid transfer function (-1 < x < 1) 
  float fCtrl = uav_yaw_f(fError_deg, YAW_CTRL_SLOPE) * YAW_ERROR_RATE * dT;
  float fZero = uav_zero_f(fError_deg, YAW_ZERO_SLOPE) * YAW_ERROR_RATE * dT;
  
  // Change the yaw of the copter if error to target is high
  m_fTargetYaw_deg += fCtrl;
  // Anneal the yaw change to zero if the error is low
  m_fTargetYaw_deg += sign_f(-m_fTargetYaw_deg) * fZero;
  // Cap the maximum yaw change
  m_fTargetYaw_deg  = abs(m_fTargetYaw_deg) > MAX_YAW ? sign_f(m_fTargetYaw_deg) * MAX_YAW : m_fTargetYaw_deg;
  
  if(fError_deg <= 1.f) {
    m_iTargetPit_deg = -10;
  } else {
    m_iTargetPit_deg = 0;
  }
  
  #if DEBUG_OUT
  m_pHalBoard->m_pHAL->console->printf("Navigation - Comp: %.3f, Err: %.3f, CTRL: %.3f, ZERO: %.3f, Yaw: %.3f\n", m_pHalBoard->get_comp_deg(), fError_deg, fCtrl, fZero, m_fTargetYaw_deg);
  #endif
  
  // Override the remote control
  // TODO Speed control
  m_pReceiver->m_rgChannelsRC[1] = m_iTargetPit_deg;
  m_pReceiver->m_rgChannelsRC[3] = (int_fast16_t)m_fTargetYaw_deg;
}