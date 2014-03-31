#include <AP_Math.h>
#include <math.h>

#include "navigation.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"
#include "math.h"


float sigm_uav_f(float x, float mod){
  float val = sign_f(x) * smaller_f(abs(mod * x), 180.f) / 180.f;
  return val / sqrt(1 + pow2_f(val) );
}

UAVNav::UAVNav(Device *pDev, Receiver *pRecv, Exception *pExcp) {
  m_pHalBoard       = pDev;
  m_pReceiver       = pRecv;
  m_pExeption       = pExcp;
  
  m_t32YawTimer     = m_pHalBoard->m_pHAL->scheduler->millis();
  
  m_fDestin_deg    = m_pHalBoard->get_comp_deg();
  m_fTargetYaw_deg = 0;
}

void UAVNav::target_angle() { 
  int_fast32_t iDX = m_pReceiver->m_Waypoint.longitude;
  int_fast32_t iDY = m_pReceiver->m_Waypoint.latitude;
  
  int_fast32_t iCX = m_pHalBoard->get_gps().longitude;
  int_fast32_t iCY = m_pHalBoard->get_gps().latitude;
  
  int_fast32_t iX = iDX - iCX;
  int_fast32_t iY = iDY - iCY;
  
  // Calculate the angle to the destination direction
  // NOTE: Here the system is inverted by 90° to align the magnetic north to 0°
  m_fDestin_deg = ToDeg(atan2((float)iX, (float)iY) );
}

int_fast32_t UAVNav::delta_angle() {
  m_fCurrentYaw_deg = m_pHalBoard->get_atti_raw_deg().z;
  m_fDelta_deg = m_fDestin_deg - m_fCurrentYaw_deg;
  return m_fDelta_deg;
}

void UAVNav::run() {
  const float fRate = 0.25f;
  const float fSlope  = 1.f;
 
  // Calculate the time since last call
  uint_fast32_t t32CurTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  float dT = (float)(t32CurTimer - m_t32YawTimer) / 1000.f;
  m_t32YawTimer = t32CurTimer;
  
  // Update position data and calculate the errors 
  target_angle();
  delta_angle();
  
  // Calculate the error dependent modifier, using a normed sigmoid transfer function (-1 < x < 1) 
  float fSigma = sigm_uav_f(m_fDelta_deg, fSlope);
  float fR = fRate * dT;
  m_fTargetYaw_deg = anneal_f(m_fTargetYaw_deg, -m_fTargetYaw_deg, 1.f/fSigma * fR); // Anneal to zero if the delta angle is low
  m_fTargetYaw_deg = anneal_f(m_fTargetYaw_deg, m_fDelta_deg, fSigma * fR);          // Anneal to target yaw if the delta angle is high
  // limit the maximum yaw change
  m_fTargetYaw_deg = abs(m_fTargetYaw_deg) > MAX_YAW ? sign_f(m_fTargetYaw_deg) * MAX_YAW : m_fTargetYaw_deg;
  
  // Override the remote control
  m_pReceiver->m_rgChannelsRC[3] = (int_fast16_t)m_fTargetYaw_deg;
}