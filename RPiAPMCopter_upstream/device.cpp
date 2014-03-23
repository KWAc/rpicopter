#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>


#include "device.h"
#include "config.h"
#include "BattMonitor.h"
#include "math.h"


Device::Device( const AP_HAL::HAL *pHAL,
                AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, GPS *pGPS, BattMonitor *pBat, RangeFinder *pRF )
{
  m_fAltitude_m   = 0.f;

  m_fInertRolOffs = 0.f;
  m_fInertPitOffs = 0.f;
  m_fInertYawOffs = 0.f;

  m_fInertRolCor  = 0.f;
  m_fInertPitCor  = 0.f;

  m_vAttitude_deg.x   = 0.f;
  m_vAttitude_deg.y   = 0.f;
  m_vAttitude_deg.z   = 0.f;

  m_fCmpH         = 0.f;
  m_fGpsH         = 0.f;
  
  m_eErrors       = NOTHING_F;
  
  if(CMP_FOR_YAW) {
    m_fCmpH = -read_comp_deg(0, 0);
  } else {
    m_fCmpH = 0.f;
  }
  
  // HAL
  m_pHAL     = pHAL;
  // Sensors
  m_pInert   = pInert;
  m_pComp    = pComp;
  m_pBaro    = pBar;
  m_pGPS     = pGPS;
  m_pBat     = pBat;
  m_pRF      = pRF;

  // Timers
  m_iInrtTimer = m_iAcclTimer = m_pHAL->scheduler->millis();
  
  // PIDs
  memset(m_rgPIDS, 0, sizeof(m_rgPIDS) );
}

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
          m_pHAL->console->println("Initialisation failed\n");
      }
    #endif
  #else
    m_pHAL->console->println("No range finder installed\n");
  #endif
}
#ifdef SONAR_TYPE
float Device::read_rf_m() {
  m_fAltitude_m = (float)m_pRF->read() / 100.f;
  return m_fAltitude_m;
}
#endif

#ifdef SONAR_TYPE
float Device::get_rf_m() {
  return m_fAltitude_m;
}
#endif

void Device::update_inertial() {
  m_pInert->update();
  
  // Calculate time (in s) passed
  float time_s = (float)(m_pHAL->scheduler->millis() - m_iInrtTimer) / 1000.f;
  m_iInrtTimer = m_pHAL->scheduler->millis();
  
  // Calculate attitude from relative gyrometer changes
  m_vAttitude_deg += read_gyro_deg() * time_s;
  m_vAttitude_deg = wrap180_V3f(m_vAttitude_deg);
  // Use a temporary instead of the member variable for acceleration data
  Vector3f vAnneal = read_accl_deg();

  // Use accelerometer on in +/-45° range
  // Pitch
  if(vAnneal.x > 45.f || vAnneal.x < -45.f) {
    return;
  }
  // Roll
  if(vAnneal.y > 45.f || vAnneal.y < -45.f) {
    return;
  }
/*
  // .. alternatively the compass or GPS could be used ..
  // First read the sensors
  if(CMP_FOR_YAW) {
    m_fCmpH = -read_comp_deg(0, 0);
  }
  if(GPS_FOR_YAW) { // TODO TEST this code
    GPSData gps = read_gps();
    float fHyp = sqrt(pow2_f(gps.heading_x) + pow2_f(gps.heading_y) );  // Calculate hypotenuse
    m_fGpsH = atan2(gps.heading_y, fHyp);                               // Calculate the heading in degrees (from X and Y heading)
  }
  // Then calculate the heading, dependent on the devices used
  // NO compass or GPS is used:
  if(!CMP_FOR_YAW && !GPS_FOR_YAW) {
    vAnneal.z = m_vAttitude_deg.z; // Take the gyrometer
  }
  // Only compass used:
  if(CMP_FOR_YAW && !GPS_FOR_YAW) {
    vAnneal.z = m_fCmpH;
  }
  // Only GPS used:
  if(!CMP_FOR_YAW && GPS_FOR_YAW) {
    vAnneal.z = m_fGpsH;
  }
  // Compass and GPS used:
  if(CMP_FOR_YAW && GPS_FOR_YAW) {
    vAnneal.z = (m_fGpsH + m_fCmpH) / 2.f;
  }
*/
  // Anneal both sensors
  m_vAttitude_deg = anneal_V3f(m_vAttitude_deg, vAnneal, time_s, INERT_ANNEAL_SLOPE, INERT_FUSION_RATE, &sigm_atti_f);
}

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
  return Vector3f(m_vAttitude_deg.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vAttitude_deg.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vAttitude_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_atti_raw_deg() {
  return m_vAttitude_deg;
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
}

void Device::init_pids() {
  // Rate PIDs
  m_rgPIDS[PID_PIT_RATE].kP(0.50);
  m_rgPIDS[PID_PIT_RATE].kI(0.25);
  m_rgPIDS[PID_PIT_RATE].imax(50);

  m_rgPIDS[PID_ROL_RATE].kP(0.50);
  m_rgPIDS[PID_ROL_RATE].kI(0.25);
  m_rgPIDS[PID_ROL_RATE].imax(50);

  m_rgPIDS[PID_YAW_RATE].kP(1.25);
  m_rgPIDS[PID_YAW_RATE].kI(0.25);
  m_rgPIDS[PID_YAW_RATE].imax(50);

  m_rgPIDS[PID_THR_RATE].kP(0.35);  // For altitude hold
  m_rgPIDS[PID_THR_RATE].kI(0.25);  // For altitude hold
  m_rgPIDS[PID_THR_RATE].imax(100); // For altitude hold
  
  // ACCELERATION PIDs
  m_rgPIDS[PID_THR_ACCL].kP(1.25);  // For altitude hold
  m_rgPIDS[PID_THR_ACCL].kI(0.00);  // For altitude hold
  m_rgPIDS[PID_THR_ACCL].imax(0);   // For altitude hold
  
  // STAB PIDs
  m_rgPIDS[PID_PIT_STAB].kP(5.50);
  m_rgPIDS[PID_ROL_STAB].kP(5.50);
  m_rgPIDS[PID_YAW_STAB].kP(5.50);
  m_rgPIDS[PID_THR_STAB].kP(5.50);  // For altitude hold
}

void Device::init_compass() {
  if(!m_pComp->init() ) {
    m_pHAL->console->printf("Init compass failed!\n");
  }

  m_pComp->accumulate();
  m_pComp->motor_compensation_type(1);                              // throttle
  m_pComp->set_offsets(0, 0, 0);                                    // set offsets to account for surrounding interference
  m_pComp->set_declination(ToRad(0.f) );                            // set local difference between magnetic north and true north

  m_pHAL->console->print("Compass auto-detected as: ");
  switch( m_pComp->product_id ) {
  case AP_COMPASS_TYPE_HIL:
    m_pHAL->console->printf("HIL\n");
    break;
  case AP_COMPASS_TYPE_HMC5843:
    m_pHAL->console->println("HMC5843\n");
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
}

void Device::init_inertial() {
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_200HZ);
  // Calibrate the inertial
  calibrate_inertial();
}

void Device::init_gps() {
  m_pGPS->init(m_pHAL->uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
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
    m_pHAL->console->println("read_gyro_deg(): Inertial not healthy\n");
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
    m_pHAL->console->println("read_accl_deg(): Inertial not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, ACCELEROMETR_F) );
    return m_vAccel_deg;
  }

  // Low Pass Filter
  Vector3f vAccelT_mss = m_pInert->get_accel();
  m_vAccel_deg = m_vAccelPG_mss = low_pass_filter_V3f(vAccelT_mss, m_vAccelPG_mss);
  
  // Calculate G-const. corrected acceleration
  m_vAccelMG_mss = vAccelT_mss - m_vAccelPG_mss;
  
  // Calculate the G-const. corrected derivative of the acceleration (speed)
  float time_s = (float)(m_pHAL->scheduler->millis() - m_iAcclTimer) / 1000.f;
  // v = v0 + a * t
  Vector3f vCurVelocity = m_vAccelMG_mss * time_s; // a * t
  m_vAccelMG_ms = low_pass_filter_V3f(vCurVelocity, m_vAccelMG_ms) + vCurVelocity; // v0 + a*t
  m_iAcclTimer = m_pHAL->scheduler->millis();
  
  // Calculate roll and pitch in degrees from the filtered acceleration readouts (attitude)
  float fpYZ = sqrt(pow2_f(m_vAccel_deg.y) + pow2_f(m_vAccel_deg.z) );
  //float fuXZ = sign_f(m_vAccel_deg.z) * sqrt(0.1f * pow2_f(m_vAccel_deg.x) + pow2_f(m_vAccel_deg.z) );
  m_vAccel_deg.x = ToDeg(atan2(m_vAccel_deg.x, fpYZ) )   - m_fInertPitOffs;   // PITCH
  //m_vAccel_deg.y = ToDeg(atan2(-m_vAccel_deg.y, -fuXZ) ) - m_fInertRolOffs; // ROLL
  m_vAccel_deg.y = ToDeg(atan2(-m_vAccel_deg.y, -m_vAccel_deg.z) ) - m_fInertRolOffs;
  m_vAccel_deg.z = 0.f;                                                       // YAW:   Cannot be calculated because accelerometer is aligned with the gravitational field vector

  return m_vAccel_deg;
}

Vector3f Device::get_accel_mg_mss() {
  return m_vAccelMG_mss;
}

Vector3f Device::get_accel_mg_ms() {
  return m_vAccelMG_ms;
}

Vector3f Device::get_accel_pg_mss() {
  return m_vAccelPG_mss;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
float Device::read_comp_deg(const float roll, const float pitch) {
  if (!m_pComp->healthy() ) {
    m_pHAL->console->println("read_comp_deg(): Compass not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, COMPASS_F) );
    return m_fCmpH;
  }
  
  m_pComp->read();
  
  Matrix3f dcm_matrix;
  dcm_matrix.from_euler(roll, pitch, 0);
  m_fCmpH = m_pComp->calculate_heading(dcm_matrix);
  m_fCmpH = ToDeg(m_fCmpH);
  m_pComp->learn_offsets();

  return m_fCmpH;
}

GPSData Device::read_gps() {
  if(m_pGPS->status() == GPS::NO_GPS) {
    m_pHAL->console->println("read_gps(): GPS not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, GPS_F) );
    return m_ContGPS;
  }

  m_pGPS->update();
  
  if(m_pGPS->new_data) {
    if(m_pGPS->fix) {
      m_ContGPS.latitude    = m_pGPS->latitude;
      m_ContGPS.longitude   = m_pGPS->longitude;
      m_ContGPS.altitude_m  = (float)m_pGPS->altitude_cm / 100.0;

      m_ContGPS.gspeed_ms   = (float)m_pGPS->ground_speed_cm / 100.0;
      m_ContGPS.espeed_ms   = m_pGPS->velocity_east();
      m_ContGPS.nspeed_ms   = m_pGPS->velocity_north();
      m_ContGPS.dspeed_ms   = m_pGPS->velocity_down();

      // The fucking avr_g++ does NOT support dynamic C++ with class like objects in structs declared as static :(
      // Hate this permittivity
      m_ContGPS.heading_x   = m_pGPS->velocity_vector().x;
      m_ContGPS.heading_y   = m_pGPS->velocity_vector().y;
      m_ContGPS.heading_z   = m_pGPS->velocity_vector().z;

      m_ContGPS.gcourse_cd  = (int)m_pGPS->ground_course_cd / 100;
      m_ContGPS.status_fix  = m_pGPS->fix;
      m_ContGPS.satelites   = m_pGPS->num_sats;
      m_ContGPS.time_week   = m_pGPS->time_week;
      m_ContGPS.time_week_s = m_pGPS->time_week_ms / 1000.0;
    } else {
      // Dunno atm
    }
    m_pGPS->new_data = false;
  }
  return m_ContGPS;
}

BaroData Device::read_baro() {
  if (!m_pBaro->healthy) {
    m_pHAL->console->println("read_baro(): Barometer not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, BAROMETER_F) );
    return m_ContBaro;
  }
  
  m_pBaro->read();
  m_ContBaro.pressure_pa      = low_pass_filter_f(m_pBaro->get_pressure(),    m_ContBaro.pressure_pa);
  m_ContBaro.altitude_m       = low_pass_filter_f(m_pBaro->get_altitude(),    m_ContBaro.altitude_m);
  m_ContBaro.temperature_deg  = low_pass_filter_f(m_pBaro->get_temperature(), m_ContBaro.temperature_deg);
  m_ContBaro.climb_rate_ms    = low_pass_filter_f(m_pBaro->get_climb_rate(),  m_ContBaro.climb_rate_ms);
  m_ContBaro.pressure_samples = m_pBaro->get_pressure_samples();

  return m_ContBaro;
}

BattData Device::read_bat() {
  m_pBat->read();

  m_ContBat.voltage_V    = m_pBat->voltage();
  m_ContBat.current_A    = m_pBat->current_amps();
  m_ContBat.consumpt_mAh = m_pBat->current_total_mah();
  
  if(m_ContBat.voltage_V < 6.0f) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, VOLTAGE_LOW_F) );
  }
  if(m_ContBat.voltage_V > 25.2f) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, VOLTAGE_HIGH_F) );
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

/*
 * Find the offset from total equilibrium.
 * Because the vehicle is never totally horizontally,
 * we will measure the discrepancy to compensate unequal motor thrust at start.
 */
Vector3f Device::calibrate_inertial() {
  Vector3f offset;
  float samples_acc[ATTITUDE_SAMPLE_CNT];
  float samples_avg = 0;
  float samples_dev = 0;

  m_fInertRolOffs = 0;
  m_fInertPitOffs = 0;
  m_fInertYawOffs = 0;

  m_fInertPitCor = 0;
  m_fInertRolCor = 0;

  float fInertRolOffs = 0;
  float fInertPitOffs = 0;
  float fInertYawOffs = 0;

  // initially switch on LEDs
  //leds_on(); bool led = true;

  while(true) {
    samples_avg = 0;
    samples_dev = 0;

    fInertRolOffs = 0;
    fInertPitOffs = 0;
    fInertYawOffs = 0;

    // Take 10 samples in less than one second
    for(uint_fast8_t i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
      m_pInert->update();
      offset = read_accl_deg();

      m_pHAL->console->printf("Gyroscope calibration - Offsets are roll:%f, pitch:%f, yaw:%f.\n",
                              (double)offset.y, (double)offset.x, (double)offset.z);

      fInertPitOffs += offset.x / (float)ATTITUDE_SAMPLE_CNT;
      fInertRolOffs += offset.y / (float)ATTITUDE_SAMPLE_CNT;
      fInertYawOffs += offset.z / (float)ATTITUDE_SAMPLE_CNT;

      // Check whether the data set is useable
      float cur_sample = sqrt(pow2_f(offset.x) + pow2_f(offset.y) + pow2_f(offset.z) );
      samples_acc[i] = cur_sample;
      samples_avg += cur_sample / ATTITUDE_SAMPLE_CNT;

      //flash_leds(led); led = !led;  // Let LEDs blink
      m_pHAL->scheduler->delay(50);     // Wait 50ms
    }

    // Calc standard deviation
    for(uint_fast8_t i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
      samples_dev += pow2_f(samples_acc[i] - samples_avg) / (float)ATTITUDE_SAMPLE_CNT;
    }
    samples_dev = sqrt(samples_dev);
    // If std dev is low: exit loop
    if(samples_dev < samples_avg / 16.f)
      break;
  }

  // Save offsets
  m_fInertPitOffs = offset.x = fInertPitOffs;
  m_fInertRolOffs = offset.y = fInertRolOffs;
  m_fInertYawOffs = offset.z = fInertYawOffs;

  //leds_off();   // switch off leds
  m_pHAL->console->printf("Gyroscope calibrated - Offsets are roll:%f, pitch:%f, yaw:%f.\nAverage euclidian distance:%f, standard deviation:%f\n",
                          (double)m_fInertRolOffs, (double)m_fInertPitOffs, (double)m_fInertYawOffs, (double)samples_avg, (double)samples_dev);
  return offset;
}
