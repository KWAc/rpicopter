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
#include <AP_InertialNav.h>
//#include <AP_InertialNav_NavEKF.h>
#include <AP_GPS.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_BattMonitor.h>

#include "extended_readouts.h"
#include "config.h"
#include "math.h"
#include "device.h"


float zaccel_g(Device *pDev, bool &bOK) {
  static float fGForce = 0.f;
  
  bOK = false;
  // Break when no device was found
  if(!pDev) {
    return fGForce; 
  }
  // -45 < Pitch < +45
  if(pDev->get_atti_raw_deg().x > 45.f || pDev->get_atti_raw_deg().x < -45.f) {
    return fGForce;
  }
  // -45 < Roll < +45
  if(pDev->get_atti_raw_deg().y > 45.f || pDev->get_atti_raw_deg().y < -45.f) {
    return fGForce;
  }
  
  float fCFactor = 100.f * INERT_G_CONST;
  float fG       = -pDev->get_accel_mg_cmss().z / fCFactor;
  fGForce        = low_pass_filter_f(fG, fGForce, ZACCL_LOWPATH_FILT_f);
  
  bOK = true;
  return fGForce;
}

float climbrate_cms(Device *pDev, bool &bOK) {
  int_fast32_t iAltitude_cm = 0.f;
    
  bOK = false;
  if(!pDev) {
    return iAltitude_cm; 
  }
  
  if(pDev->m_pInertNav->altitude_ok() ) {
    iAltitude_cm = pDev->m_pInertNav->get_velocity_z();
    bOK = true;
  }

  return pDev->m_pInertNav->get_velocity_z();
}

int_fast32_t altitude_cm(Device * pDev, bool &bOK) {
  int_fast32_t iAltitude_cm = 0.f;

  bOK = false;
  if(!pDev) {
    return 0.f; 
  }
  
  // Barometer and GPS usable
  if(pDev->m_pInertNav->altitude_ok() ) {
    iAltitude_cm = (int_fast32_t)pDev->m_pInertNav->get_altitude();
    bOK = true;
  }

#ifdef SONAR_TYPE
  // Use the range finder for smaller altitudes
  int_fast32_t iAltitudeRF_cm = pDev->get_rf_cm();
  if(iAltitudeRF_cm <= 600) {
    iAltitude_cm = iAltitudeRF_cm;
  }
#endif
  
  return iAltitude_cm;
}
