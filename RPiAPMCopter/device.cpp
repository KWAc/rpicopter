#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_GPS.h>
#include <AP_BattMonitor.h>

#include "device.h"
#include "BattMonitor.h"
#include "math.h"


float delta_angle(float fCurVal, float fOldVal) {  
  float fVal = fCurVal - fOldVal;
  
  if(fVal < -180.f) {
    fVal = (fCurVal + 360) - fOldVal;
  }
  if(fVal > 180.f) {
    fVal = (fCurVal - 360) - fOldVal;
  }
  
  return fVal;
}

/*
 * Function from a LiPo charging chart:
 * 4,20 V  100%
 * 4,13 V	90%
 * 4,06 V	80%
 * 3,99 V	70%
 * 3,92 V	60%
 * 3,85 V	50%
 * 3,78 V	40%
 * 3,71 V	30%
 * 3,64 V	20%
 * 3,57 V	10%
 * 3,50 V  0%
 * Return: 0 - 1 (0%-100%) if in voltage range of this table :D
 */
float batt_rescapa(float voltage_V, unsigned int num_cells) {
  float fCap =  1.4286 * (voltage_V / (float)num_cells) - 5.f;
  return fCap < 0.f ? 0.f : fCap > 1.f ? 1.f : fCap;
}

/*
 * Fuses two sensor values together by annealing angle_1 to angle_2 
 * in every time step by a given rate value.
 */
float sensor_fuse(float angle_cor, float angle_fix, float time, float rate) {  
  return angle_cor += wrap_180(angle_fix-angle_cor)*(time/1000)*rate;
}

Device::Device(const AP_HAL::HAL *pHAL,
        AP_InertialSensor_MPU6000 *pInert, 
        AP_Compass_HMC5843 *pComp, 
        AP_Baro_MS5611 *pBar, 
        AP_GPS_UBLOX *pGPS, 
        BattMonitor *pBat,
        PID *pPids) 
{
  m_fInertRolOffs = 0.f;
  m_fInertPitOffs = 0.f;
  m_fInertYawOffs = 0.f;
  
  // HAL
  m_pHAL   = pHAL;
  // Sensors
  m_pInert = pInert;
  m_pComp  = pComp;
  m_pBaro  = pBar;
  m_pGPS   = pGPS;
  m_pBat   = pBat;
  // PIDs
  m_pids = pPids;
}

/* 
 * Find the offset from total equilibrium.
 * Because the vehicle is never totally horizontally, 
 * we will measure the discrepancy to compensate unequal motor thrust at start.
 */
Vector3f Device::attitude_calibration() {
  Vector3f offset;
  float samples_acc[ATTITUDE_SAMPLE_CNT];
  float samples_avg = 0;
  float samples_dev = 0;
  
  m_fInertRolOffs = 0;
  m_fInertPitOffs = 0;
  m_fInertYawOffs = 0;
  
  float fInertRolOffs = 0;
  float fInertPitOffs = 0;
  float fInertYawOffs = 0;
  
  // initially switch on LEDs
  //leds_on(); bool led = true;
  
  // RC_CHANNELS[2] == thrust
  // run calibration only if motors _don't_ spin
  // otherwise model would probably not calibrate properly or crash while flying
  while(true) {
    samples_avg = 0;
    samples_dev = 0;
    
    fInertRolOffs = 0;
    fInertPitOffs = 0;
    fInertYawOffs = 0;
  
    // Take 10 samples in less than one second
    for(int i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
      m_pInert->update();
      offset = read_atti();

      m_pHAL->console->printf("Gyroscope calibration - Offsets are roll:%f, pitch:%f, yaw:%f.\n", 
                          offset.x, offset.y, offset.z);

      fInertRolOffs += offset.y / (float)ATTITUDE_SAMPLE_CNT;
      fInertPitOffs += offset.x / (float)ATTITUDE_SAMPLE_CNT;
      fInertYawOffs += offset.z / (float)ATTITUDE_SAMPLE_CNT;
      
      // Check whether the data set is useable
      float cur_sample = sqrt(pow(offset.x, 2) + pow(offset.y, 2) + pow(offset.z, 2) );
      samples_acc[i] = cur_sample;
      samples_avg += cur_sample / ATTITUDE_SAMPLE_CNT;
    
      //flash_leds(led); led = !led;  // Let LEDs blink
      m_pHAL->scheduler->delay(50);     // Wait 50ms
    }
    
    // Calc standard deviation
    for(int i = 0; i < ATTITUDE_SAMPLE_CNT; i++) {
    samples_dev += pow(samples_acc[i] - samples_avg, 2) / (float)ATTITUDE_SAMPLE_CNT;
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
                      m_fInertRolOffs, m_fInertPitOffs, m_fInertYawOffs, samples_avg, samples_dev);
  return offset;
}

void Device::init_barometer() {
  m_pBaro->init();
  m_pBaro->calibrate();
}

void Device::init_pids() {
  m_pids[PID_PIT_RATE].kP(0.50);
  m_pids[PID_PIT_RATE].kI(0.25);
  m_pids[PID_PIT_RATE].imax(50);

  m_pids[PID_ROL_RATE].kP(0.50);
  m_pids[PID_ROL_RATE].kI(0.25);
  m_pids[PID_ROL_RATE].imax(50);

  m_pids[PID_YAW_RATE].kP(1.25);
  m_pids[PID_YAW_RATE].kI(0.25);
  m_pids[PID_YAW_RATE].imax(50);

  m_pids[PID_PIT_STAB].kP(5.5);
  m_pids[PID_ROL_STAB].kP(5.5);
  m_pids[PID_YAW_STAB].kP(5.5);
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
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
  // Calibrate the inertial
  attitude_calibration();
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
  m_vGyro = m_pInert->get_gyro();
  
  // In comparision to read_atti() x and y are exchanged. Why?
  float fRol = ToDeg(m_vGyro.x);
  float fPitch = ToDeg(m_vGyro.y);
  
  m_vGyro.x = fPitch;
  m_vGyro.y = fRol;
  m_vGyro.z = ToDeg(m_vGyro.z);
  
  return m_vGyro;
}

/* 
 * Reads the current attitude from the accelerometer in degrees and returns it as a 3D vector 
 */
Vector3f Device::read_atti() {
  m_vAtti = m_pInert->get_accel();  
  float r = sqrt(pow(m_vAtti.x, 2) + pow(m_vAtti.y, 2) + pow(m_vAtti.z, 2) );
  
  m_vAtti.x  = -(ToDeg(acos(m_vAtti.x/r) ) - 90.f) - m_fInertPitOffs;
  m_vAtti.y  = ToDeg(acos(m_vAtti.y/r) ) - 90.f - m_fInertRolOffs;
  m_vAtti.z  = ToDeg(acos(m_vAtti.z/r) ) - 180.f - m_fInertYawOffs;
  
  return m_vAtti;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
float Device::read_comp(float roll, float pitch) {
  m_fComp = 999;
  
  m_pComp->read();
  if (!m_pComp->healthy() ) {
    m_pHAL->console->println("Compass not healthy\n");
    return m_fComp;
  }
  Matrix3f dcm_matrix;
  dcm_matrix.from_euler(roll, pitch, 0);
  m_fComp = m_pComp->calculate_heading(dcm_matrix);
  m_fComp = ToDeg(m_fComp);
  m_pComp->null_offsets();
  return m_fComp;
}

GPSData Device::read_gps() {  
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
      
      // The fucking avr_g++ does NOT support dynamic C++ with class like obects in structs declared as static :(
      // Hate this primitivity
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
  m_pBaro->read();

  if (!m_pBaro->healthy) {
    m_pHAL->console->println("Barometer not healthy\n");
    return m_ContBaro;
  }
  
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
 * bias: Maximum countable drift in degrees
 */
void Device::gyro_drift(Vector3f &drift, Vector3f &offset, int &samples, float bias) {
  static int   counter  = -1;
  static uint32_t timer = 0;
  uint32_t time = m_pHAL->scheduler->millis() - timer;
    
  static float lst_rol  = 0;
  static float lst_pit  = 0;
  static float lst_yaw  = 0;
  
  static float lst_drol = 0; 
  static float lst_dpit = 0; 
  static float lst_dyaw = 0;
  
  static float sum_rol  = 0;
  static float sum_pit  = 0;
  static float sum_yaw  = 0;
  
  float rol = 0, pit = 0, yaw = 0;
  float drol = 0, dpit = 0, dyaw = 0;

  if(time >= 2000) {
    offset = read_atti();
    
    rol = offset.x;
    pit = offset.y;
    yaw = offset.z;
    
    rol = wrap_360(rol);
    pit = wrap_360(pit);
    yaw = wrap_360(yaw);

    drol = delta_angle(rol, lst_rol);
    dpit = delta_angle(pit, lst_pit);
    dyaw = delta_angle(yaw, lst_yaw);
    
    lst_rol = rol;
    lst_pit = pit;
    lst_yaw = yaw;

    if(abs(drol-lst_drol) > 0.05 || abs(dpit-lst_dpit) > 0.05 || abs(dyaw-lst_dyaw) > 0.025) {                       
      lst_drol = drol;
      lst_dpit = dpit;
      lst_dyaw = dyaw;
      
      timer = m_pHAL->scheduler->millis();
      return;
    }
    counter++;

    if(counter > 0) {
      sum_rol += drol * 1000/time;
      sum_pit += dpit * 1000/time;
      sum_yaw += dyaw * 1000/time;
      
      drift.x = sum_rol/counter;
      drift.y = sum_pit/counter;
      drift.z = sum_yaw/counter;
    } else {
      m_pHAL->console->printf("Ignore first cycle for calibration: ");
    }
    
    samples = counter;
    timer = m_pHAL->scheduler->millis();
  }
}

