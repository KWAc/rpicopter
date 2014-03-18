#include <AP_GPS.h>
#include <AP_Baro.h>

#include "exceptions.h"
#include "receiver.h"
#include "device.h"
#include "absdevice.h"


/*
 * Calculate time necessary to reduce the throttle until the value "THR_TAKE_OFF"
 * The higher the quadcopter is the longer it will take, because the take-down speed should be constant (MAX_FALL_SPEED_MS: ~0.833 m/s)
 */
inline float go_down_t(float altitude_m, float fThr) {
  // altitude_m should be always positive
  altitude_m = altitude_m <= 0.f ? 1.f : altitude_m;
  
  // Calc save time (in ms) to touch the ground
  float fTime2Ground_ms = altitude_m / MAX_FALL_SPEED_MS * 1000.f;
  // Delta of the current throttle and the throttle needed to take-off
  float fdThr = fThr > THR_TAKE_OFF ? (fThr - THR_TAKE_OFF) / THR_STEP_S : fThr / THR_STEP_S;

  // Time constant (in ms) which determines the velocity to reduce the throttle
  return fTime2Ground_ms / fdThr;
}

Exception::Exception(Device *pDevice, Receiver *pReceiver) {
  m_pHalBoard   = pDevice;
  m_pReceiver   = pReceiver;
  m_bRcvrOvride = false;
  m_fLastAltitude_m = 0.f;
  
  m_iAltitudeTimer = m_iInertTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  memcpy(m_rgChannelsRC, m_pReceiver->m_rgChannelsRC, sizeof(m_pReceiver->m_rgChannelsRC) );
}

void Exception::dev_take_down() {
  static bool bInertTimer = false;
  // If motors do not spin: reset & return
  if(m_pReceiver->m_rgChannelsRC[2] == RC_THR_OFF) {
    bInertTimer = false;
    m_pHalBoard->set_errors(AbsErrorDevice::NOTHING_F);
    rls_recvr();
    return;
  }
  // Set timer one time, to calculate how much the motors should be reduced
  if(bInertTimer == false) {
    m_iInertTimer = m_pHalBoard->m_pHAL->scheduler->millis();
    bInertTimer = true;
  }
  // Override the receiver, no matter what happens
  lck_recvr();
  reduce_thr(m_pHalBoard->m_pHAL->scheduler->millis() - m_iInertTimer);
}

void Exception::rcvr_take_down() {
  // Get time to calculate how much the motors should be reduced
  uint_fast32_t packet_t = m_pReceiver->timeLastSuccessfulParse();  // Measure time elapsed since last successful package from WiFi or radio
  // If motors do not spin or a new packet arrived: reset & return
  if( m_pReceiver->m_rgChannelsRC[2] == RC_THR_OFF || packet_t <= COM_PKT_TIMEOUT ) {
    m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
    rls_recvr();
    return;
  }
  // Remove the override if there was a new package within the interval
  lck_recvr();                                                      // Copy the last command into a temporary
  // Calculate the time passed since there was no new package and reduce the throttle
  reduce_thr(packet_t - COM_PKT_TIMEOUT);                           // Reduce thrust ..
}

void Exception::lck_recvr() {
  if(m_bRcvrOvride == true) {
    return;
  }
  memcpy(m_rgChannelsRC, m_pReceiver->m_rgChannelsRC, sizeof(m_pReceiver->m_rgChannelsRC) );
  m_bRcvrOvride = true;
}
  
void Exception::rls_recvr() {
  if(m_bRcvrOvride == false)  {
    return;
  }
  m_bRcvrOvride = false;
  m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
  memset(m_rgChannelsRC, 0, sizeof(m_rgChannelsRC) );
}

bool Exception::handle() {
  //////////////////////////////////////////////////////////////////////////////////////////
  // Device handler
  //////////////////////////////////////////////////////////////////////////////////////////
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GYROMETER_F) {   // Gyrometer was not healthy: Go down straight
    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::ACCELEROMETR_F) { // Accelerometer was not healthy: Go down straight
    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::BAROMETER_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::COMPASS_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GPS_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::VOLTAGE_HIGH_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::VOLTAGE_LOW_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::CURRENT_HIGH_F) {
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::CURRENT_LOW_F) { // Battery is at the end: Go down straight
    dev_take_down();
    return true;
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////
  // Remote control handler
  //////////////////////////////////////////////////////////////////////////////////////////
  if(m_pReceiver->get_errors() & AbsErrorDevice::UART_TIMEOUT_F) {
    rcvr_take_down();
    return true;
  }
  
  return false;
}

float Exception::estim_altit(bool &bOK) {
  float fAlti = 0.f;
  bOK = false;
  // Barometer and GPS usable
  if(m_pHalBoard->m_pBaro->healthy && m_pHalBoard->m_pGPS->status() == GPS::GPS_OK_FIX_3D) {
    fAlti = m_pHalBoard->read_baro().altitude_m;
    fAlti += m_pHalBoard->read_gps().altitude_m;
    fAlti /= 2.f;
    bOK = true;
  }
  // Only barometer usable
  else if(m_pHalBoard->m_pBaro->healthy && m_pHalBoard->m_pGPS->status() != GPS::GPS_OK_FIX_3D) {
    fAlti = m_pHalBoard->read_baro().altitude_m;
    bOK = true;
  }
  // Only GPS usable
  else if(!m_pHalBoard->m_pBaro->healthy && m_pHalBoard->m_pGPS->status() == GPS::GPS_OK_FIX_3D) {
    fAlti = m_pHalBoard->read_gps().altitude_m;
    bOK = true;
  }
  return fAlti;
}

void Exception::reduce_thr(float fTime) {
  static float fRho = 0.f;                                                // if model is falling too fast, this variable is used as a stopper
  float fStepC      = 15.f;                                               // Default step size
  // Do only if override was set!
  if(m_bRcvrOvride == false) {
    return;
  }
  // The speed of decreasing the throttle is dependent on the height
  uint_fast32_t iAltitudeTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iAltitudeTimer;
  if(iAltitudeTime > ALTI_MEASURE_TIME) {
    bool bSensorOK = false;
    float fAlti = estim_altit(bSensorOK);
    if(bSensorOK == true) {
      fStepC = go_down_t(fAlti, m_rgChannelsRC[2]);
    }
    // Is model falling too fast?
    if(m_fLastAltitude_m > fAlti) {
      float fFSpeed = (m_fLastAltitude_m - fAlti) / (float)iAltitudeTime;
      // If yes: increase fRho
      if(fFSpeed > MAX_FALL_SPEED_MS + 0.25f) {
        fRho += 5.f;
      }
    }
    // Save some variables and set timer
    m_fLastAltitude_m = fAlti;
    m_iAltitudeTimer  = m_pHalBoard->m_pHAL->scheduler->millis();
  }
  // Calculate how much to reduce throttle
  float fTConst = (THR_STEP_S * (fTime / fStepC) ) - fRho;
  int_fast16_t fThr = m_rgChannelsRC[2] - (int_fast16_t)fTConst;
  // reduce throttle..
  m_pReceiver->m_rgChannelsRC[2] = fThr >= RC_THR_MIN ? fThr : RC_THR_OFF; // throttle
  // reset yaw, pitch and roll
  m_pReceiver->m_rgChannelsRC[3] = 0;                                      // yaw
  m_pReceiver->m_rgChannelsRC[1] = 0;                                      // pitch
  m_pReceiver->m_rgChannelsRC[0] = 0;                                      // roll
}
