#include <stdint.h>
#include <stddef.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_Math.h>
////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_GPS.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>

#include "extended_readouts.h"
#include "config.h"
#include "math.h"
#include "device.h"


float climbrate_ms(Device * pDev, bool &bOK) {
  static uint_fast32_t iClmbTimer = 0;
  static float fFusedClimbRate = 0.f;

  bOK = false;
  if(!pDev) {
    return 0.f; 
  }
 
  if(pDev->m_pBaro->healthy && pDev->m_pInert->healthy() ) {
    fFusedClimbRate  = pDev->get_baro().climb_rate_ms;
    float fAcclCRate = pDev->get_accel_mg_ms().z;
    
    // Calculate time since last update time
    uint_fast32_t time_ms = pDev->m_pHAL->scheduler->millis() - iClmbTimer;
    time_ms = time_ms <= INERT_TIMEOUT ? time_ms : INERT_TIMEOUT;
    float time_s = (float)time_ms / 1000.f;
    iClmbTimer = pDev->m_pHAL->scheduler->millis();
    fFusedClimbRate = anneal_f(fFusedClimbRate, fAcclCRate, time_s, CLIMB_ANNEAL_SLOPE, CLIMB_FUSION_RATE, &sigm_climb_f);
    
    bOK = true;
  }
  
  return fFusedClimbRate;
}

float altitude_m(Device * pDev, bool &bOK) {
  float fAltitude_m = 0.f;
    
  bOK = false;
  if(!pDev) {
    return 0.f; 
  }
  
  // Barometer and GPS usable
  if(pDev->m_pBaro->healthy && pDev->m_pGPS->status() == GPS::GPS_OK_FIX_3D) {
    fAltitude_m = pDev->get_baro().altitude_m;
    fAltitude_m += pDev->get_gps().altitude_m;
    fAltitude_m /= 2.f;
    bOK = true;
  }
  // Only barometer usable
  else if(pDev->m_pBaro->healthy && pDev->m_pGPS->status() != GPS::GPS_OK_FIX_3D) {
    fAltitude_m = pDev->get_baro().altitude_m;
    bOK = true;
  }
  // Only GPS usable
  else if(!pDev->m_pBaro->healthy && pDev->m_pGPS->status() == GPS::GPS_OK_FIX_3D) {
    fAltitude_m = pDev->get_gps().altitude_m;
    bOK = true;
  }
  
  // Use the range finder for smaller altitudes
  #ifdef SONAR_TYPE
    float fAltitudeRF_m = pDev->get_rf_m();
    if(fAltitudeRF_m <= 6) {
      fAltitude_m = fAltitudeRF_m;
    }
  #endif
  
  return fAltitude_m;
}

float altitude_cm(Device * pDev, bool &bOK) {
  return altitude_m(pDev, bOK) * 100.f;
}
