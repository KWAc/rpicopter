#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
//#include <AP_InertialNav_NavEKF.h>
#include <AP_GPS.h>             // ArduPilot GPS library

#include <AP_BattMonitor.h>
#include <RangeFinder_Backend.h>
#include <AP_BoardLED.h>

#include "device.h"
#include "config.h"
#include "BattMonitor.h"
#include "arithmetics.h"
#include "filter.h"


// create board led object
AP_BoardLED board_led;

/*
 * Gaussian bell function
 */
inline float atti_f(float fX, float fSlope) {
  // Calculate the slope of the function ..
  // .. dependent on the angular range which is allowed (for the accelerometer)
  fSlope = 180.f / INERT_ANGLE_BIAS;

  // Calculate the output rating
  float fVal = (180.f - fabs(fSlope * fX) ) / 180.f;
  fVal /= sqrt(1.f + pow2_f(fVal) );
  
  // Limit the function: be always >= zero
  fVal = fVal < 0.f ? 0.f : pow2_f(fVal);
  return (4.f * pow2_f(fVal) );  // x^4; 0 <= y <= 1.0
}

void Device::update_attitude() {
  m_pAHRS->update();

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

#if SIGM_FOR_ATTITUDE
  // Use "m_pInert->update()" only if "m_pAHRS->update()" is not used
  //m_pInert->update();

  // Calculate time (in s) passed
  uint_fast32_t t32CurrentTime = m_pHAL->scheduler->millis();
  float dT = static_cast<float>((t32CurrentTime - m_t32Inertial) ) / 1000.f;
  m_t32Inertial = t32CurrentTime;

  // Calculate attitude from relative gyrometer changes
  m_vAtti_deg  += read_gyro_deg() * dT;
  m_vAtti_deg   = wrap180_V3f(m_vAtti_deg);
  m_vAtti_deg.z = ToDeg(m_pAHRS->yaw); // Use AHRS for the yaw

  // Use a temporary instead of the member variable for acceleration data
  Vector3f vRef_deg = read_accl_deg();

  #if DEBUG_OUT
  static int iDTimer = 0;
  int iDCurTime = m_pHAL->scheduler->millis();
  if(iDCurTime - iDTimer >= 35) {
    iDTimer = iDCurTime;
  
    m_pHAL->console->printf("Attitude - x: %.3f/%.3f, y: %.3f/%.3f, z: %.1f\n", m_vAtti_deg.x, vRef_deg.x, m_vAtti_deg.y, vRef_deg.y, m_vAtti_deg.z);
    //m_pHAL->console->printf("Acceleration - x: %.3f, y: %.3f, z: %.3f\n", m_vAccelPG_cmss.x, m_vAccelPG_cmss.y, m_vAccelPG_cmss.z);
  }
  #endif
 
  // Some sanity checks, before annealing to the accelerometer readouts
  if( abs(m_vAccelPG_cmss.z) < INERT_FFALL_BIAS ||        // Free fall
      abs(vRef_deg.x)        > INERT_ANGLE_BIAS ||        // Out of range (roll  > 60°)
      abs(vRef_deg.y)        > INERT_ANGLE_BIAS)          // Out of range (pitch > 60°)
  {
    #if DEBUG_OUT
    m_pHAL->console->printf("Attitude estimation out of range or free fall: Don't anneal to accelerometer.\n");
    #endif
    return;
  }
 
  m_vAtti_deg.x = SFilter::transff_filt_f(m_vAtti_deg.x, vRef_deg.x-m_vAtti_deg.x, dT*INERT_FUSION_RATE, Functor_f(&atti_f, vRef_deg.x) );
  m_vAtti_deg.y = SFilter::transff_filt_f(m_vAtti_deg.y, vRef_deg.y-m_vAtti_deg.y, dT*INERT_FUSION_RATE, Functor_f(&atti_f, vRef_deg.y) );
#else
  m_vAtti_deg.x = ToDeg(m_pAHRS->pitch);
  m_vAtti_deg.y = ToDeg(m_pAHRS->roll);
  m_vAtti_deg.z = ToDeg(m_pAHRS->yaw);
#endif
}

Device::Device( const AP_HAL::HAL *pHAL,
                AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, AP_GPS *pGPS, BattMonitor *pBat, AP_RangeFinder_Backend *pRF, AP_AHRS_DCM *pAHRS, AP_InertialNav *pInertNav )
{
  m_iAltitude_cm      = 0;

  m_fInertRolCor      = 0.f;
  m_fInertPitCor      = 0.f;

  m_vAtti_deg.x       = 0.f;
  m_vAtti_deg.y       = 0.f;
  m_vAtti_deg.z       = 0.f;

  m_fCmpH             = 0.f;
  m_fGpsH             = 0.f;

  m_iUpdateRate       = MAIN_T_MS;
  m_eErrors           = NOTHING_F;

  // HAL
  m_pHAL              = pHAL;
  // Sensors
  m_pInert            = pInert;
  m_pComp             = pComp;
  m_pBaro             = pBar;
  m_pGPS              = pGPS;
  m_pBat              = pBat;
  m_pRF               = pRF;
  m_pAHRS             = pAHRS;
  m_pInertNav         = pInertNav;

  // Timers
  m_t32Compass = m_t32InertialNav = m_t32Inertial = m_pHAL->scheduler->millis();

  // PIDs
  memset(m_rgPIDS, 0, sizeof(m_rgPIDS) );
}
/*
void Device::init_rf() {
  #ifdef SONAR_TYPE
    #if SONAR_TYPE <= AP_RANGEFINDER_MAXSONARI2CXL
      // type conversion
      AP_RangeFinder_MaxsonarXL *pRF = (AP_RangeFinder_MaxsonarXL*)m_pRF;
      // init scaler
      pRF->calculate_scaler(SONAR_TYPE, SONAR_SCALING);
    #elif SONAR_TYPE == AP_RANGEFINDER_PULSEDLIGHT
        // type conversion
      AP_RangeFinder_PulsedLightLRF *pRF = (AP_RangeFinder_PulsedLightLRF*)m_pRF;
      // ensure i2c is slow
      hal.i2c->setHighSpeed(false);
      // initialise sensor
      pRF->init();
      // kick off one reading
      pRF->take_reading();
      // check health
      if (!pRF->healthy) {
          m_pHAL->console->printf("Initialisation failed\n");
      }
    #endif
  #else
    m_pHAL->console->printf("No range finder installed\n");
  #endif
}
*/
void Device::init_inertial_nav() {
  m_pAHRS->set_compass(m_pComp);

  m_pInertNav->init();
  m_pInertNav->set_velocity_xy(0.f, 0.f);
  m_pInertNav->set_velocity_z(0.f);

  m_pInertNav->setup_home_position();
  m_pInertNav->set_altitude(0.f);

  m_t32Compass = m_t32Inertial = m_t32InertialNav = m_pHAL->scheduler->millis();
}
/*
#ifdef SONAR_TYPE
int_fast32_t Device::read_rf_cm() {
  m_iAltitude_cm = m_pRF->read();
  return m_iAltitude_cm;
}
#endif

#ifdef SONAR_TYPE
int_fast32_t Device::get_rf_cm() {
  return m_iAltitude_cm;
}
#endif
*/
float Device::get_pit_cor() {
  return m_fInertPitCor;
}

float Device::get_rol_cor() {
  return m_fInertRolCor;
}

void Device::set_pit_cor(float fValDeg) {
  m_fInertPitCor = fValDeg;
}

void Device::set_rol_cor(float fValDeg) {
  m_fInertRolCor = fValDeg;
}

Vector3f Device::get_atti_cor_deg() {
  return Vector3f(m_vAtti_deg.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vAtti_deg.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vAtti_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_atti_raw_deg() {
  return m_vAtti_deg;
}

Vector3f Device::get_gyro_cor_deg() {
  return Vector3f(m_vGyro_deg.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vGyro_deg.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vGyro_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_gyro_raw_deg() {
  return m_vGyro_deg;
}

Vector3f Device::get_accel_cor_deg() {
  return Vector3f(m_vAccel_deg.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vAccel_deg.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vAccel_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_accel_raw_deg() {
  return m_vAccel_deg;
}

void Device::init_barometer() {
  m_pBaro->init();
  m_pBaro->calibrate();

#ifdef APM2_HARDWARE
  // we need to stop the barometer from holding the SPI bus
  m_pHAL->gpio->pinMode(40, GPIO_OUTPUT);
  m_pHAL->gpio->write(40, HIGH);
#endif
}

void Device::update_inav() {
  if(!m_pGPS) {
    return;
  }

  read_gps();
  read_comp_deg();
  //m_pAHRS->update(); // AHRS system is already updated at 100 Hz in the attitude update function

  uint_fast32_t t32CurrentTime = m_pHAL->scheduler->millis();
  float fTime_s = (t32CurrentTime - m_t32InertialNav) / 1000.f;
  m_pInertNav->update(fTime_s);
  m_t32InertialNav = t32CurrentTime;
}

void Device::init_pids() {
  // Rate PIDs
  m_rgPIDS[PID_PIT_RATE].kP(0.65);
  m_rgPIDS[PID_PIT_RATE].kI(0.35);
  m_rgPIDS[PID_PIT_RATE].kD(0.015);
  m_rgPIDS[PID_PIT_RATE].imax(50);

  m_rgPIDS[PID_ROL_RATE].kP(0.65);
  m_rgPIDS[PID_ROL_RATE].kI(0.35);
  m_rgPIDS[PID_ROL_RATE].kD(0.015);
  m_rgPIDS[PID_ROL_RATE].imax(50);

  m_rgPIDS[PID_YAW_RATE].kP(0.75);
  m_rgPIDS[PID_YAW_RATE].kI(0.15);
  m_rgPIDS[PID_YAW_RATE].kD(0.0f);
  m_rgPIDS[PID_YAW_RATE].imax(50);

  m_rgPIDS[PID_THR_RATE].kP(0.75);  // For altitude hold
  m_rgPIDS[PID_THR_RATE].kI(0.25);  // For altitude hold
  m_rgPIDS[PID_THR_RATE].kD(0.0f);  // For altitude hold
  m_rgPIDS[PID_THR_RATE].imax(100); // For altitude hold

  m_rgPIDS[PID_ACC_RATE].kP(1.50);  // For altitude hold
  m_rgPIDS[PID_ACC_RATE].kI(0.75);  // For altitude hold
  m_rgPIDS[PID_ACC_RATE].kD(0.0f);  // For altitude hold
  m_rgPIDS[PID_ACC_RATE].imax(100); // For altitude hold

  // STAB PIDs
  m_rgPIDS[PID_PIT_STAB].kP(4.25);
  m_rgPIDS[PID_ROL_STAB].kP(4.25);
  m_rgPIDS[PID_YAW_STAB].kP(4.25);
  m_rgPIDS[PID_THR_STAB].kP(5.50);  // For altitude hold
  m_rgPIDS[PID_ACC_STAB].kP(15.50); // For altitude hold
}

void Device::init_compass() {
  if(!m_pComp->init() ) {
    m_pHAL->console->printf("Init compass failed!\n");
  }

  m_pComp->accumulate();
  m_pComp->motor_compensation_type(1);                              // throttle
  m_pComp->set_and_save_offsets(0, 0, 0, 0);                        // set offsets to account for surrounding interference
  m_pComp->set_declination(ToRad(0.f) );                            // set local difference between magnetic north and true north

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

void Device::init_inertial() {
  // Turn on MPU6050
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_200HZ);

  // Calibrate the inertial
  m_t32Inertial = m_pHAL->scheduler->millis();
}

void Device::init_gps() {
  // Init the GPS without logging
  m_pGPS->init(NULL);
  // Initialise the LEDs
  board_led.init();
}

void Device::init_batterymon() {
  // initialise the battery monitor for ATTO180 sensor by default :D
  m_pBat->setup_source(ATTO180);
}

/*
 * Reads the current altitude changes from the gyroscope in degrees and returns it as a 3D vector
 */
Vector3f Device::read_gyro_deg() {
  if(!m_pInert->healthy() ) {
    m_pHAL->console->printf("read_gyro_deg(): Inertial not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, GYROMETER_F) );
    return m_vGyro_deg;
  }

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

/*
 * Reads the current attitude from the accelerometer in degrees and returns it as a 3D vector
 * From: "Tilt Sensing Using a Three-Axis Accelerometer"
 */
Vector3f Device::read_accl_deg() {
  if(!m_pInert->healthy() ) {
    m_pHAL->console->printf("read_accl_deg(): Inertial not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, ACCELEROMETR_F) );
    return m_vAccel_deg;
  }

  // Low Pass SFilter
  Vector3f vAccelCur_cmss = m_pInert->get_accel() * 100.f;
  m_vAccel_deg = m_vAccelPG_cmss = SFilter::low_pass_filt_V3f(vAccelCur_cmss, m_vAccelPG_cmss, INERT_LOWPATH_FILT_f);

  // Calculate G-const. corrected acceleration
  m_vAccelMG_cmss = vAccelCur_cmss - m_vAccelPG_cmss;

  // Calculate roll and pitch in degrees from the filtered acceleration readouts (attitude)
  float fpYZ = sqrt(pow2_f(m_vAccel_deg.y) + pow2_f(m_vAccel_deg.z) );
  // Pitch
  m_vAccel_deg.x = ToDeg(atan2(m_vAccel_deg.x, fpYZ) );
  // Roll
  m_vAccel_deg.y = ToDeg(atan2(-m_vAccel_deg.y, -m_vAccel_deg.z) );

  return m_vAccel_deg;
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

  // Update the compass readout maximally ten times a second
  if(m_pHAL->scheduler->millis() - m_t32Compass <= COMPASS_UPDATE_T) {
    return m_fCmpH;
  }

  m_pComp->read();

  m_fCmpH = m_pComp->calculate_heading(m_pAHRS->get_dcm_matrix() );
  m_fCmpH = ToDeg(m_fCmpH);
  m_pComp->learn_offsets();

  return m_fCmpH;
}

GPSData Device::read_gps() {
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
  } else {
    //m_pHAL->console->printf("read_gps(): GPS not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, GPS_F) );
  }

  return m_ContGPS;
}

BaroData Device::read_baro() {
  if (!m_pBaro->healthy) {
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
  bOK = false;
  if(!pDev) {
    return 0.f;
  }

  float fAltitude_cm = 0.f;
  // Barometer and GPS usable
  if(pDev->m_pInertNav->altitude_ok() ) {
    fAltitude_cm = static_cast<float>(pDev->m_pInertNav->get_altitude() );
    bOK = true;
  }
/*
#ifdef SONAR_TYPE
  // Use the range finder for smaller altitudes
  float iAltitudeRF_cm = static_cast<float>(pDev->get_rf_cm() );
  if(iAltitudeRF_cm <= 600) {
    fAltitude_cm = iAltitudeRF_cm;
  }
#endif
*/
  return fAltitude_cm;
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
  float fG       = pDev->get_accel_mg_cmss().x / fCFactor;
  fGForce        = SFilter::low_pass_filt_f(fG, fGForce, ACCEL_LOWPATH_FILT_f);

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
  float fG       = pDev->get_accel_mg_cmss().y / fCFactor;
  fGForce        = SFilter::low_pass_filt_f(fG, fGForce, ACCEL_LOWPATH_FILT_f);

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
  float fG       = -pDev->get_accel_mg_cmss().z / fCFactor;
  fGForce        = SFilter::low_pass_filt_f(fG, fGForce, ACCEL_LOWPATH_FILT_f);

  bOK = true;
  return fGForce;
}

void Device::set_update_rate_ms(const uint_fast8_t rate) {
  m_iUpdateRate = rate;
}

uint_fast8_t Device::get_update_rate_ms() const {
  return m_iUpdateRate;
}
