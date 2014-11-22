#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_RangeFinder_MaxsonarI2CXL.h>
#include <AP_GPS.h>

#include <AP_RangeFinder.h>
#include <AP_BoardLED.h>

#include "arithmetics.h"
#include "BattMonitor.h"
#include "config.h"
#include "device.h"
#include "filter.h"

// create board led object
AP_BoardLED board_led;

const AP_Param::GroupInfo DeviceInit::var_info[] PROGMEM = {
    AP_GROUPEND
};

inline void display_offsets_and_scaling(const AP_HAL::HAL *pHAL, AP_InertialSensor *pInert) {
    Vector3f accel_offsets = pInert->get_accel_offsets();
    Vector3f accel_scale = pInert->get_accel_scale();
    Vector3f gyro_offsets = pInert->get_gyro_offsets();

    // display results
    pHAL->console->printf("\nAccel Offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    pHAL->console->printf("Accel Scale \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    pHAL->console->printf("Gyro Offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

///////////////////////////////////////////////////////////
// DeviceInit
///////////////////////////////////////////////////////////
void DeviceInit::load_pids() {
  for(int i = 0; i < NR_OF_PIDS; i++) {
    m_pPIDs[i].load_gains();
    m_pHAL->console->printf("Load PID P %f  I %f  D %f  imax %f\n", (double)m_pPIDs[i].kP(), (double)m_pPIDs[i].kI(), (double)m_pPIDs[i].kD(), (double)m_pPIDs[i].imax());
  }
}

void DeviceInit::save_pids() {
  for(int i = 0; i < NR_OF_PIDS; i++) {
    m_pPIDs[i].save_gains();
  }
}

void DeviceInit::init_rf() {
  AP_Param::set_object_value(m_pRF, m_pRF->var_info, "_TYPE",     RangeFinder::RangeFinder_TYPE_ANALOG);
  AP_Param::set_object_value(m_pRF, m_pRF->var_info, "_PIN",      RANGE_FINDER_PIN);
  AP_Param::set_object_value(m_pRF, m_pRF->var_info, "_SCALING",  RANGE_FINDER_SCALE);
  m_pRF->init();
}

void DeviceInit::init_inertial_nav() {
  m_pAHRS->set_compass(m_pComp);

  m_pInertNav->init();
  m_pInertNav->set_velocity_xy(0.f, 0.f);
  m_pInertNav->set_velocity_z(0.f);

  m_pInertNav->setup_home_position();
  m_pInertNav->set_altitude(0.f);

  m_t32Compass = m_t32Inertial = m_t32InertialNav = m_pHAL->scheduler->millis();
}

void DeviceInit::init_barometer() {
  m_pBaro->init();
  m_pBaro->calibrate();
}

void DeviceInit::init_compass() {
  if(!m_pComp->init() ) {
    m_pHAL->console->printf("Init compass failed!\n");
    return;
  }

  if(BATT_ATTO_3DR) {
    m_pComp->motor_compensation_type(2); // current sensing
  } 
  else {
    m_pComp->motor_compensation_type(1); // throttle as input
  }

  m_pHAL->console->print("Compass auto-detected as: ");
  switch( m_pComp->product_id ) {
  case AP_COMPASS_TYPE_HIL:
    m_pHAL->console->printf("HIL\n");
    break;
  case AP_COMPASS_TYPE_HMC5843:
    m_pHAL->console->printf("HMC5843\n");
    break;
  case AP_COMPASS_TYPE_HMC5883L:
    m_pHAL->console->printf("HMC5883L\n");
    break;
  case AP_COMPASS_TYPE_PX4:
    m_pHAL->console->printf("PX4\n");
    break;
  default:
    m_pHAL->console->printf("unknown\n");
    break;
  }

  m_t32Compass = m_pHAL->scheduler->millis();
}

void DeviceInit::init_inertial() {
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  m_pHAL->gpio->pinMode(40, HAL_GPIO_OUTPUT);
  m_pHAL->gpio->write(40, HIGH);
#endif

  // Turn on MPU6050
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
//  #if DEBUG_OUT
  display_offsets_and_scaling(m_pHAL, m_pInert);
//  #endif
  
  m_t32Inertial = m_pHAL->scheduler->millis();
}

void DeviceInit::init_gps() {
  // Init the GPS without logging
  m_pGPS->init(NULL);
  // Initialise the LEDs
  board_led.init();
}

void DeviceInit::init_batterymon() {
  m_pBat->setup_type(BattMonitor::ATTO180);
  m_pBat->setup_cap(BATT_CAP_mAh);
  m_pBat->init();
}

DeviceInit::DeviceInit( const AP_HAL::HAL *pHAL, AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, AP_GPS *pGPS, BattMonitor *pBat, RangeFinder *pRF, AP_AHRS_DCM *pAHRS, AP_InertialNav *pInertNav, AC_PID *pPIDs) 
{
  m_pHAL              = pHAL;
  m_pInert            = pInert;
  m_pComp             = pComp;
  m_pBaro             = pBar;
  m_pGPS              = pGPS;
  m_pBat              = pBat;
  m_pRF               = pRF;
  m_pAHRS             = pAHRS;
  m_pInertNav         = pInertNav;
  m_eErrors           = NOTHING_F;
  m_t32Compass = m_t32InertialNav = m_t32Inertial = m_pHAL->scheduler->millis();
  
  m_pPIDs = pPIDs;
  
  // Rate PIDs
  m_pPIDs[PID_PIT_RATE] = AC_PID(0.65, 0.35, 0.015, 50);
  m_pPIDs[PID_ROL_RATE] = AC_PID(0.65, 0.35, 0.015, 50);
  m_pPIDs[PID_YAW_RATE] = AC_PID(0.75, 0.50, 0.f, 50);
  m_pPIDs[PID_THR_RATE] = AC_PID(0.25, 0.50, 0.f, 100);
  m_pPIDs[PID_ACC_RATE] = AC_PID(0.50, 0.10, 0.f, 100);
  // STAB PIDs
  m_pPIDs[PID_PIT_STAB] = AC_PID(4.25, 0.f, 0.f, 0);
  m_pPIDs[PID_ROL_STAB] = AC_PID(4.25, 0.f, 0.f, 0);
  m_pPIDs[PID_YAW_STAB] = AC_PID(4.25, 0.f, 0.f, 0);
  m_pPIDs[PID_THR_STAB] = AC_PID(5.50, 0.f, 0.f, 0);
  m_pPIDs[PID_ACC_STAB] = AC_PID(4.25, 0.f, 0.f, 0);
}

AC_PID &DeviceInit::get_pid(uint_fast8_t index) {
  if(index >= NR_OF_PIDS) {
    return m_pPIDs[NR_OF_PIDS-1];
  }
  return m_pPIDs[index];
}

void DeviceInit::set_pid(uint_fast8_t index, const AC_PID &pid) {
  if(index >= NR_OF_PIDS) {
    m_pPIDs[NR_OF_PIDS-1] = pid;
  }
  m_pPIDs[index] = pid;
}

///////////////////////////////////////////////////////////
// Device
///////////////////////////////////////////////////////////
void Device::update_attitude() {
  // Update the inertial and calculate attitude
  m_pAHRS->update();
  // Calculate the attitude based on the accelerometer
  calc_acceleration();

#if BENCH_OUT
  static int iBCounter = 0;
  static int iTimer = 0;
  int iBCurTime = m_pHAL->scheduler->millis();
  ++iBCounter;
  if(iBCurTime - iTimer >= 1000) {
    m_pHAL->console->printf("Benchmark - update_attitude(): %d Hz\n", iBCounter);
    iBCounter = 0;
    iTimer = iBCurTime;
  }
#endif

  m_vAtti_deg.x = ToDeg(m_pAHRS->pitch);
  m_vAtti_deg.y = ToDeg(m_pAHRS->roll);
  m_vAtti_deg.z = ToDeg(m_pAHRS->yaw);
}

Device::Device( const AP_HAL::HAL *pHAL, AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, AP_GPS *pGPS, BattMonitor *pBat, RangeFinder *pRF, AP_AHRS_DCM *pAHRS, AP_InertialNav *pInertNav, AC_PID *pPIDs) : 
DeviceInit(pHAL, pInert, pComp, pBar, pGPS,  pBat, pRF, pAHRS, pInertNav, pPIDs) 
{
  m_iAltitude_cm      = 0;

  m_fInertRolCor      = 0.f;
  m_fInertPitCor      = 0.f;

  m_vAtti_deg.x       = 0.f;
  m_vAtti_deg.y       = 0.f;
  m_vAtti_deg.z       = 0.f;

  m_fCmpH             = 0.f;
  m_fGpsH             = 0.f;
}

int_fast32_t Device::read_rf_cm() {
  m_pRF->update();
  if(m_pRF->healthy() ) {
    m_iAltitude_cm = m_pRF->distance_cm();
  }
  return m_iAltitude_cm;
}

int_fast32_t Device::get_rf_cm() {
  return m_iAltitude_cm;
}

void Device::set_trims(float fRoll_deg, float fPitch_deg) {
  m_fInertRolCor = fRoll_deg;
  m_fInertPitCor = fPitch_deg;
}

Vector3f Device::get_atti_cor_deg() {
  return Vector3f(m_vAtti_deg.x - m_fInertPitCor, // Pitch correction for imbalances
                  m_vAtti_deg.y - m_fInertRolCor, // Roll correction for imbalances
                  m_vAtti_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_atti_raw_deg() {
  return m_vAtti_deg;
}

Vector3f Device::get_gyro_degps() {
  if(!m_pInert->healthy() ) {
    m_pHAL->console->printf("read_gyro_deg(): Inertial not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, GYROMETER_F) );
    return m_vGyro_deg;
  }
  
  // Read the current gyrometer value
  m_vGyro_deg = m_pInert->get_gyro();
  
  // Save values
  float fRol = ToDeg(m_vGyro_deg.x); // in comparison to the accelerometer data swapped
  float fPit = ToDeg(m_vGyro_deg.y); // in comparison to the accelerometer data swapped
  float fYaw = ToDeg(m_vGyro_deg.z);
  
  // Put them into the right order
  m_vGyro_deg.x = fPit; // PITCH
  m_vGyro_deg.y = fRol; // ROLL
  m_vGyro_deg.z = fYaw; // YAW

  return m_vGyro_deg;
}

void Device::update_inav() {
  if(!m_pGPS) {
    return;
  }

  read_gps();
  read_comp_deg();

  uint_fast32_t t32CurrentTime = m_pHAL->scheduler->millis();
  float fTime_s = (t32CurrentTime - m_t32InertialNav) / 1000.f;
  m_pInertNav->update(fTime_s);
  m_t32InertialNav = t32CurrentTime;
}

void Device::calc_acceleration() {
  if(!m_pInert->healthy() ) {
    m_pHAL->console->printf("calc_acceleration(): Inertial not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, ACCELEROMETR_F) );
  }
  // Low Pass SFilter
  Vector3f vAccelCur_cmss = m_pAHRS->get_accel_ef() * 100.f;
  m_vAccelPG_cmss = SFilter::low_pass_filt_V3f(vAccelCur_cmss, m_vAccelPG_cmss, INERT_LOWPATH_FILT_f);
  // Calculate G-const. corrected acceleration
  m_vAccelMG_cmss = vAccelCur_cmss - m_vAccelPG_cmss;
}

Vector3f Device::get_accel_mg_cmss() {
  return m_vAccelMG_cmss;
}

Vector3f Device::get_accel_pg_cmss() {
  return m_vAccelPG_cmss;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
float Device::read_comp_deg() {
  if (!m_pComp->use_for_yaw() ) {
    //m_pHAL->console->printf("read_comp_deg(): Compass not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, COMPASS_F) );
    return m_fCmpH;
  }
  
  // accumulate compass values
  m_pComp->accumulate();
  
  // Update the compass readout maximally ten times a second
  if(m_pHAL->scheduler->millis() - m_t32Compass <= COMPASS_UPDATE_T) {
    return m_fCmpH;
  }
  
  // After some time read the compass
  m_pComp->read();
  m_fCmpH = m_pComp->calculate_heading(m_pAHRS->get_dcm_matrix() );
  m_fCmpH = ToDeg(m_fCmpH);
  m_pComp->learn_offsets();
  
  return m_fCmpH;
}

GPSData Device::read_gps() {
  static bool bSetCompLocation = false;

  if(!m_pGPS) {
    return m_ContGPS;
  }

  m_pGPS->update();
  m_ContGPS.status = static_cast<uint_fast32_t>(m_pGPS->status() );
  
  if(m_ContGPS.status > AP_GPS::NO_FIX) {
    m_ContGPS.latitude    = m_pGPS->location().lat;
    m_ContGPS.longitude   = m_pGPS->location().lng;
    m_ContGPS.altitude_cm = m_pGPS->location().alt;

    m_ContGPS.gspeed_cms  = m_pGPS->ground_speed_cm();

    m_ContGPS.gcourse_cd  = m_pGPS->ground_course_cd();
    m_ContGPS.satelites   = m_pGPS->num_sats();
    m_ContGPS.time_week   = m_pGPS->time_week();
    m_ContGPS.time_week_s = m_pGPS->time_week_ms() / 1000.0;
    
    // If the GPS is working and the compass initiated, 
    // set the location once
    if(m_pComp->healthy() && !bSetCompLocation) {
      m_pComp->set_initial_location(m_ContGPS.latitude, m_ContGPS.longitude);
      bSetCompLocation = true;
    }
  } else {
    //m_pHAL->console->printf("read_gps(): GPS not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, GPS_F) );
  }

  return m_ContGPS;
}

BaroData Device::read_baro() {
  if (!m_pBaro->healthy() ) {
    m_pHAL->console->printf("read_baro(): Barometer not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, BAROMETER_F) );
    return m_ContBaro;
  }

  m_pBaro->read();

  m_ContBaro.pressure_pa      = SFilter::low_pass_filt_f(m_pBaro->get_pressure(), m_ContBaro.pressure_pa, BAROM_LOWPATH_FILT_f);
  m_ContBaro.temperature_deg  = SFilter::low_pass_filt_f(m_pBaro->get_temperature(), m_ContBaro.temperature_deg, BAROM_LOWPATH_FILT_f);

  m_ContBaro.altitude_cm      = static_cast<int_fast32_t>(SFilter::low_pass_filt_f(m_pBaro->get_altitude(), m_ContBaro.altitude_cm, BAROM_LOWPATH_FILT_f) * 100);
  m_ContBaro.climb_rate_cms   = static_cast<int_fast32_t>(SFilter::low_pass_filt_f(m_pBaro->get_climb_rate(), m_ContBaro.climb_rate_cms, BAROM_LOWPATH_FILT_f) * 100);

  m_ContBaro.pressure_samples = m_pBaro->get_pressure_samples();

  return m_ContBaro;
}

BattData Device::read_bat() {
  const unsigned int iRefVoltSamples = 25;
  static unsigned int iRefVoltCounter = 0;
  static float fRefVolt_V = 0.f;

  m_pBat->read();

  m_ContBat.voltage_V    = m_pBat->voltage();
  m_ContBat.current_A    = m_pBat->current_amps();
  m_ContBat.power_W      = m_ContBat.voltage_V * m_ContBat.current_A;
  m_ContBat.consumpt_mAh = m_pBat->current_total_mah();

  // Only perform current sensing for the compass
  // if there is a ATTO/3DR sensor in use
  if(m_pComp->healthy() ) {
    // set the current in A
    m_pComp->set_current(m_ContBat.current_A);
  }
  
  if(m_ContBat.voltage_V < BATT_MIN_VOLTAGE) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, VOLTAGE_LOW_F) );
  }
  if(m_ContBat.voltage_V > BATT_MAX_VOLTAGE) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, VOLTAGE_HIGH_F) );
  }

  // Collect samples (iRefVoltSamples) for reference voltage (but only if readouts are valid)
  if(iRefVoltCounter < iRefVoltSamples) {
    if(m_ContBat.voltage_V <= BATT_MAX_VOLTAGE && m_ContBat.voltage_V >= BATT_MIN_VOLTAGE) {
      iRefVoltCounter++;
      fRefVolt_V += m_ContBat.voltage_V;
    }
  } 
  // Only update reference voltage if necessary
  else if(m_ContBat.refVoltage_V < 0) {
    m_ContBat.refVoltage_V = fRefVolt_V / iRefVoltSamples;
  }
  
  return m_ContBat;
}

float Device::get_comp_deg() {
  return m_fCmpH;
}

BaroData Device::get_baro() {
  return m_ContBaro;
}

GPSData Device::get_gps() {
  return m_ContGPS;
}

BattData Device::get_bat() {
  return m_ContBat;
}

float Device::get_altitude_cm(Device *pDev, bool &bOK) {
  bOK = true;
  if(!pDev) {
    bOK = false;
    return 0.f;
  }

  float fAltitude_cm = 0.f;
  // Barometer and GPS usable
  if(pDev->m_pInertNav->altitude_ok() ) {
    fAltitude_cm = static_cast<float>(pDev->m_pInertNav->get_altitude() );
  }

  // Use the range finder for smaller altitudes
  if(!pDev->m_pRF->healthy() ) {
    bOK = false;
    return fAltitude_cm;
  }
  
  float iAltitudeRF_cm = static_cast<float>(pDev->get_rf_cm() );
  return iAltitudeRF_cm <= AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE ? iAltitudeRF_cm : fAltitude_cm;
}

float Device::get_accel_x_g(Device *pDev, bool &bOK) {
  static float fGForce = 0.f;

  bOK = false;
  // Break when no device was found
  if(!pDev) {
    return fGForce;
  }
  // Sanity check
  if(abs(pDev->get_atti_raw_deg().x) > INERT_ANGLE_BIAS || abs(pDev->get_atti_raw_deg().y) > INERT_ANGLE_BIAS) {
    return fGForce;
  }

  float fCFactor = 100.f * INERT_G_CONST;
  fGForce       = pDev->get_accel_mg_cmss().x / fCFactor;
  bOK = true;
  return fGForce;
}

float Device::get_accel_y_g(Device *pDev, bool &bOK) {
  static float fGForce = 0.f;

  bOK = false;
  // Break when no device was found
  if(!pDev) {
    return fGForce;
  }
  // Sanity check
  if(abs(pDev->get_atti_raw_deg().x) > INERT_ANGLE_BIAS || abs(pDev->get_atti_raw_deg().y) > INERT_ANGLE_BIAS) {
    return fGForce;
  }

  float fCFactor = 100.f * INERT_G_CONST;
  fGForce       = pDev->get_accel_mg_cmss().y / fCFactor;
  bOK = true;
  return fGForce;
}

float Device::get_accel_z_g(Device *pDev, bool &bOK) {
  static float fGForce = 0.f;

  bOK = false;
  // Break when no device was found
  if(!pDev) {
    return fGForce;
  }
  // Sanity check
  if(abs(pDev->get_atti_raw_deg().x) > INERT_ANGLE_BIAS || abs(pDev->get_atti_raw_deg().y) > INERT_ANGLE_BIAS) {
    return fGForce;
  }

  float fCFactor = 100.f * INERT_G_CONST;
  fGForce       = pDev->get_accel_mg_cmss().z / fCFactor;
  bOK = true;
  return fGForce;
}
