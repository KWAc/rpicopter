#include <float.h>

#include <AP_GPS.h>
#include <AP_Baro.h>

#include "exceptions.h"
#include "receiver.h"
#include "device.h"
#include "absdevice.h"
#include "extended_readouts.h"

/*
 * Calculate time necessary to reduce the throttle until the value "THR_TAKE_OFF"
 * The higher the quadcopter is the longer it will take, because the take-down speed should be constant (MAX_FALL_SPEED_MS: ~0.833 m/s)
 */
inline float go_down_t(float altitude_m, float fThr) {  
  // Calc save time (in ms) to touch the ground
  float fTime2Ground_ms = altitude_m / MAX_FALL_SPEED_MS * 1000.f;
  // Delta of the current throttle and the throttle needed to take-off
  float fdThr = fThr > THR_TAKE_OFF ? (fThr - THR_TAKE_OFF) / THR_MOD_STEP_S : fThr / THR_MOD_STEP_S;

  // Time constant (in ms) which determines the velocity to reduce the throttle
  return fTime2Ground_ms / fdThr;
}

Exception::Exception(Device *pDevice, Receiver *pReceiver) {
  m_pHalBoard    = pDevice;
  m_pReceiver    = pReceiver;
 
  m_bPauseTD     = false;
  m_iPauseTDTime = 0;
  
  m_t32Pause = m_t32Altitude = m_t32Device = m_pHalBoard->m_pHAL->scheduler->millis();
  
  memcpy(m_rgChannelsRC, m_pReceiver->m_rgChannelsRC, sizeof(m_pReceiver->m_rgChannelsRC) );
}

void Exception::dev_take_down() {
  static bool bInertTimer = false;
  static bool bIgnRcvr    = false;
  
  // If motors do not spin: reset & return
  if(m_pReceiver->m_rgChannelsRC[2] == RC_THR_OFF) {
    bInertTimer = false;
    m_pHalBoard->set_errors(AbsErrorDevice::NOTHING_F);
    m_pReceiver->m_Waypoint.mode = GPSPosition::NOTHING_F;
    rls_recvr(bIgnRcvr);
    return;
  }
  // Don't reduce speed of the motors if pause set
  if(m_bPauseTD == true) {
    return;
  }
  
  // Set timer one time, to calculate how much the motors should be reduced
  if(bInertTimer == false) {
    m_t32Device = m_pHalBoard->m_pHAL->scheduler->millis();
    bInertTimer = true;
  }
  // Override the receiver, no matter what happens
  lck_recvr(bIgnRcvr);
  if(bIgnRcvr == true) {
    reduce_thr(m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Device - m_iPauseTDTime);
  }
}

void Exception::rcvr_take_down() {
  static bool bIgnRcvr    = false;
  // Get time to calculate how much the motors should be reduced
  uint_fast32_t packet_t = m_pReceiver->last_parse_t32(); // Measure time elapsed since last successful package from WiFi or radio
  // If motors do not spin or a new packet arrived: reset & return
  if( m_pReceiver->m_rgChannelsRC[2] == RC_THR_OFF || packet_t <= COM_PKT_TIMEOUT ) {
    m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
    m_pReceiver->m_Waypoint.mode = GPSPosition::NOTHING_F;
    rls_recvr(bIgnRcvr);
    return;
  }
  // Don't reduce speed of the motors if pause set
  if(m_bPauseTD == true) {
    return;
  }
  // Remove the override if there was a new package within the interval
  lck_recvr(bIgnRcvr); // Copy the last command into a temporary
  if(bIgnRcvr == true) {
    reduce_thr(packet_t - COM_PKT_TIMEOUT - m_iPauseTDTime);
  }
}

void Exception::lck_recvr(bool &bSwitch) {
  if(bSwitch == true) {
    return;
  }
  memcpy(m_rgChannelsRC, m_pReceiver->m_rgChannelsRC, sizeof(m_pReceiver->m_rgChannelsRC) );
  bSwitch = true;
}
  
void Exception::rls_recvr(bool &bSwitch) {
  if(bSwitch != true)  {
    return;
  }
  bSwitch = false;
  m_pReceiver->set_errors(AbsErrorDevice::NOTHING_F);
  memset(m_rgChannelsRC, 0, sizeof(m_rgChannelsRC) );
}

void Exception::disable_alti_hold() {
  if(m_pReceiver->m_Waypoint.mode == GPSPosition::HLD_ALTITUDE_F) {
    m_pReceiver->m_Waypoint.mode = GPSPosition::NOTHING_F;
  }
}

void Exception::disable_gps_navigation() {
  if(m_pReceiver->m_Waypoint.mode == GPSPosition::GPS_NAVIGATN_F) {
    m_pReceiver->m_Waypoint.mode = GPSPosition::NOTHING_F;
  }
}

bool Exception::handle() {
  //////////////////////////////////////////////////////////////////////////////////////////
  // Device handler
  //////////////////////////////////////////////////////////////////////////////////////////
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GYROMETER_F) {     // Gyrometer was not healthy: Go down straight
    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::ACCELEROMETR_F) {  // Accelerometer was not healthy: Go down straight
    dev_take_down();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::BAROMETER_F) {     // If barometer is not working, the height calculation would be likely unreliable (GPS probably not stable)
    disable_alti_hold();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::COMPASS_F) {       // If Compass not working, the GPS navigation shouldn't be used
    disable_gps_navigation();
    return true;
  }
  if(m_pHalBoard->get_errors() & AbsErrorDevice::GPS_F) {           // And if GPS not working, the GPS navigation shouldn't be used at all :D
    disable_gps_navigation();
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
  if(m_pHalBoard->get_errors() & AbsErrorDevice::CURRENT_LOW_F) {   // Battery is at the end: Go down straight
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

void Exception::pause_take_down() {
  m_bPauseTD = true;
  m_t32Pause = m_pHalBoard->m_pHAL->scheduler->millis();
}

void Exception::continue_take_down() {
  if(m_bPauseTD == false) {
    return;
  }

  m_bPauseTD = false;
  m_iPauseTDTime += m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Pause;
}

void Exception::reduce_thr(float fTime) {
  static float fStepC           = 15.f;   // Default step size

  m_pReceiver->m_Waypoint.mode = GPSPosition::CONTRLD_DOWN_F;
  
  // The speed of decreasing the throttle is dependent on the height
  uint_fast32_t iAltitudeTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_t32Altitude;
  if(iAltitudeTime > AHRS_T_MS) {
    bool bOK = false;
    float fAlti_m = altitude_cm(m_pHalBoard, bOK) / 100.f;
    if(bOK == true) {
      fStepC = go_down_t(fAlti_m, m_rgChannelsRC[2]);
      fStepC = fStepC < THR_MIN_STEP_S ? THR_MIN_STEP_S : fStepC;
    }
    // Save some variables and set timer
    m_t32Altitude   = m_pHalBoard->m_pHAL->scheduler->millis();
  }
  
  // Calculate how much to reduce throttle
  float fTConst = (THR_MOD_STEP_S * (fTime / fStepC) );
  int_fast16_t fThr = m_rgChannelsRC[2] - (int_fast16_t)fTConst;
  
  // reduce throttle..
  m_pReceiver->m_rgChannelsRC[2] = fThr >= RC_THR_MIN ? fThr : RC_THR_OFF; // throttle
  // reset yaw, pitch and roll
  m_pReceiver->m_rgChannelsRC[3] = 0;                                      // yaw
  m_pReceiver->m_rgChannelsRC[1] = 0;                                      // pitch
  m_pReceiver->m_rgChannelsRC[0] = 0;                                      // roll
}
