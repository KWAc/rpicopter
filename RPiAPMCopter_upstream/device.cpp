#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <AP_BattMonitor.h>

#include "device.h"
#include "config.h"
#include "BattMonitor.h"
#include "math.h"


Device::Device( const AP_HAL::HAL *pHAL,
                AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, GPS *pGPS, BattMonitor *pBat )
{
  m_fInertRolOffs = 0.f;
  m_fInertPitOffs = 0.f;
  m_fInertYawOffs = 0.f;

  m_fInertRolCor  = 0.f;
  m_fInertPitCor  = 0.f;

  m_vAttitude.x   = 0.f;
  m_vAttitude.y   = 0.f;
  m_vAttitude.z   = 0.f;

  m_fCmpH         = 0.f;
  m_fGpsH         = 0.f;
  
  if(CMP_FOR_YAW) {
    m_fCmpH = -read_comp(0, 0);
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

  // Initialize all members
  // Timer for sensor fusion
  m_iTimer   = m_pHAL->scheduler->millis();
  // PIDs
  memset(m_pPIDS, 0, sizeof(m_pPIDS) );
  /*
  // Don't! Sensors are not initiated!
  m_ContBaro = read_baro();
  m_ContGPS  = read_gps();
  m_ContBat  = read_bat();
  */
}
  
uint_fast32_t Device::time_elapsed_ms() {
  uint_fast32_t time_ms = m_pHAL->scheduler->millis() - m_iTimer;
  m_iTimer = m_pHAL->scheduler->millis();
  return time_ms;
}

float Device::time_elapsed_s() {
  float time_s = (float)time_elapsed_ms() / 1000.f;
  return time_s;
}

void Device::update_inertial() {
  m_pInert->update();
  
  // Calculate time (in s) passed
  float time_s = time_elapsed_s();
  // Calculate attitude from relative gyrometer changes
  m_vAttitude += read_gyro() * time_s;
  m_vAttitude = wrap180_V3f(m_vAttitude);
  // Use a temporary instead of the member variable for acceleration data
  Vector3f vAnneal = read_accel();

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
    m_fCmpH = -read_comp(0, 0);
  }
  if(GPS_FOR_YAW) { // TODO TEST this code
    GPSData gps = read_gps();
    float fHyp = sqrt(pow2_f(gps.heading_x) + pow2_f(gps.heading_y) );  // Calculate hypotenuse
    m_fGpsH = atan2(gps.heading_y, fHyp);                               // Calculate the heading in degrees (from X and Y heading)
  }
  // Then calculate the heading, dependent on the devices used
  // NO compass or GPS is used:
  if(!CMP_FOR_YAW && !GPS_FOR_YAW) {
    vAnneal.z = m_vAttitude.z; // Take the gyrometer
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
  m_vAttitude = anneal_V3f(m_vAttitude, vAnneal, time_s, 20.f, 5.f);
}

float Device::getInertPitCor() {
  return m_fInertPitCor;
}

float Device::getInertRolCor() {
  return m_fInertRolCor;
}

void Device::setInertPitCor(float fValDeg) {
  m_fInertPitCor = fValDeg;
}

void Device::setInertRolCor(float fValDeg) {
  m_fInertRolCor = fValDeg;
}

Vector3f Device::get_atti_cor() {
  return Vector3f(m_vAttitude.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vAttitude.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vAttitude.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_atti_raw() {
  return m_vAttitude;
}

Vector3f Device::get_gyro_cor() {
  return Vector3f(m_vGyro.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vGyro.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vGyro.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_gyro_raw() {
  return m_vGyro;
}

Vector3f Device::get_accel_cor() {
  return Vector3f(m_vAccel.x - m_fInertPitCor, // Pitch correction for inbalances
                  m_vAccel.y - m_fInertRolCor, // Roll correction for inbalances
                  m_vAccel.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_accel_raw() {
  return m_vAccel;
}

void Device::init_barometer() {
  m_pBaro->init();
  m_pBaro->calibrate();
}

void Device::init_pids() {
  m_pPIDS[PID_PIT_RATE].kP(0.50);
  m_pPIDS[PID_PIT_RATE].kI(0.25);
  m_pPIDS[PID_PIT_RATE].imax(50);

  m_pPIDS[PID_ROL_RATE].kP(0.50);
  m_pPIDS[PID_ROL_RATE].kI(0.25);
  m_pPIDS[PID_ROL_RATE].imax(50);

  m_pPIDS[PID_YAW_RATE].kP(1.25);
  m_pPIDS[PID_YAW_RATE].kI(0.25);
  m_pPIDS[PID_YAW_RATE].imax(50);

  m_pPIDS[PID_PIT_STAB].kP(5.5);
  m_pPIDS[PID_ROL_STAB].kP(5.5);
  m_pPIDS[PID_YAW_STAB].kP(5.5);
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
Vector3f Device::read_gyro() {
  if(!m_pInert->healthy() ) {
    m_pHAL->console->println("read_gyro(): Inertial not healthy\n");
    return m_vGyro;
  }
  
  m_vGyro = m_pInert->get_gyro();
  // Save values
  float fRol = ToDeg(m_vGyro.x); // in comparison to the accelerometer data swapped
  float fPit = ToDeg(m_vGyro.y); // in comparison to the accelerometer data swapped
  float fYaw = ToDeg(m_vGyro.z);
  // Put them into the right order
  m_vGyro.x = fPit; // PITCH
  m_vGyro.y = fRol; // ROLL
  m_vGyro.z = fYaw; // YAW
  
  return m_vGyro;
}

/*
 * Reads the current attitude from the accelerometer in degrees and returns it as a 3D vector
 * From: "Tilt Sensing Using a Three-Axis Accelerometer"
 */
Vector3f Device::read_accel() { 
  if(!m_pInert->healthy() ) {
    m_pHAL->console->println("read_accel(): Inertial not healthy\n");
    return m_vAccel;
  }

  // Low Pass Filter
  Vector3f vAccelTmp_mss = m_pInert->get_accel();
  m_vAccel = m_vAccelLast_mss = vAccelTmp_mss * LOWPATH_FILT + (m_vAccelLast_mss * (1.0 - LOWPATH_FILT));
  
  // Calculate roll and pitch in degrees from the filtered acceleration readouts
  float fpYZ = sqrt(pow2_f(m_vAccel.y) + pow2_f(m_vAccel.z) );
  //float fuXZ = sign_f(m_vAccel.z) * sqrt(0.1f * pow2_f(m_vAccel.x) + pow2_f(m_vAccel.z) );
  m_vAccel.x = ToDeg(atan2(m_vAccel.x, fpYZ) )   - m_fInertPitOffs; // PITCH
  //m_vAccel.y = ToDeg(atan2(-m_vAccel.y, -fuXZ) ) - m_fInertRolOffs; // ROLL
  m_vAccel.y = ToDeg(atan2(-m_vAccel.y, -m_vAccel.z) ) - m_fInertRolOffs;
  m_vAccel.z = 0.f;                                                 // YAW:   Cannot be calculated because accelerometer is aligned with the gravitational field vector

  return m_vAccel;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
float Device::read_comp(const float roll, const float pitch) {
  if (!m_pComp->healthy() ) {
    m_pHAL->console->println("read_comp(): Compass not healthy\n");
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
    return m_ContBaro;
  }

  m_pBaro->read();
  
  m_ContBaro.pressure = m_pBaro->get_pressure();
  m_ContBaro.altitude = m_pBaro->get_altitude();
  m_ContBaro.temperature = m_pBaro->get_temperature();
  m_ContBaro.climb_rate = m_pBaro->get_climb_rate();
  m_ContBaro.pressure_samples = m_pBaro->get_pressure_samples();

  return m_ContBaro;
}

BattData Device::read_bat() {
  m_pBat->read();

  m_ContBat.voltage_V    = m_pBat->voltage();
  m_ContBat.current_A    = m_pBat->current_amps();
  m_ContBat.consumpt_mAh = m_pBat->current_total_mah();

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
      offset = read_accel();

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
