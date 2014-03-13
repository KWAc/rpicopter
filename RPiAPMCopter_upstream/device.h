#ifndef DEVICE_h
#define DEVICE_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "containers.h"

class AP_InertialSensor;
class Compass;
class AP_Baro;
class GPS;
class BattMonitor;

class BaroData;
class BattData;
class GPSData;

class PID;


///////////////////////////////////////////////////////////
// Container for sensor data and sensor configuration
///////////////////////////////////////////////////////////
class Device {
private:
  uint_fast32_t m_iTimer;
  // Used for low path filter
  Vector3f m_vAccelLast_mss;
  // Set by inertial calibration
  float m_fInertRolOffs;
  float m_fInertPitOffs;
  float m_fInertYawOffs;
  
protected:
  float     m_fCmpH; // Compass heading
  float     m_fGpsH; // GPS heading
  // x = pitch, y = roll, z = yaw
  Vector3f  m_vGyro;
  Vector3f  m_vAccel;
  Vector3f  m_vAttitude;
  // misc
  BaroData  m_ContBaro;
  GPSData   m_ContGPS;
  BattData  m_ContBat;

  float m_fInertPitCor; // +/-: left to right or right to left
  float m_fInertRolCor; // +/-: front to back or back to front
  
  /* Not updating the intertial, to avoid double updates on other spots in the code :( Not elegant so far */
  Vector3f  read_gyro();        // converts sensor relative readout to absolute attitude in degrees and saves in m_vGyro
  Vector3f  read_accel();       // converts sensor relative readout to absolute attitude and saves in m_vAccel
  
  inline float time_elapsed_s();
  inline uint_fast32_t time_elapsed_ms();
  
public:
  // PID configuration and remote contro
  PID m_pPIDS[6];
  // Hardware abstraction library interface
  const AP_HAL::HAL  *m_pHAL;
  // MPU6050 accel/gyro chip
  AP_InertialSensor  *m_pInert;
  // Magnetometer aka compass
  Compass            *m_pComp;
  // Barometer
  AP_Baro            *m_pBaro;
  // GPS
  GPS                *m_pGPS;
  // battery monitor
  BattMonitor        *m_pBat;

public:
  // Accepts pointers to abstract base classes to handle different sensor types
  Device( const AP_HAL::HAL *,
          AP_InertialSensor *, Compass *, AP_Baro *, GPS *, BattMonitor * );

  void init_barometer();
  void init_pids();
  void init_compass();
  
  Vector3f calibrate_inertial();
  void init_inertial();
  
  void init_gps();
  void init_batterymon();

  /* Updating the inertial and calculates the attitude from fused sensor values */
  void update_inertial();   // Calls: read_gyro() and read_accel() and saves results to m_vAttitude, m_vGyro and m_vAccel
  
  /* updating the sensors */
  BaroData  read_baro();
  float     read_comp(const float roll = 0.f, const float pitch = 0.f);
  GPSData   read_gps();
  BattData  read_bat();

  /* Return the Vector3f Inertial readouts */
  Vector3f get_atti_cor();  // fused sensor values from accelerometer/gyrometer with m_fInertPitCor/m_fInertRolCor
  Vector3f get_atti_raw();  // fused sensor values from accelerometer/gyrometer without m_fInertPitCor/m_fInertRolCor
  
  Vector3f get_gyro_cor();  // gyrometer sensor readout with m_fInertPitCor/m_fInertRolCor
  Vector3f get_gyro_raw();  // gyrometer sensor readout without m_fInertPitCor/m_fInertRolCor
  
  Vector3f get_accel_cor(); // accelerometer sensor readout with m_fInertPitCor/m_fInertRolCor
  Vector3f get_accel_raw(); // accelerometer sensor readout without m_fInertPitCor/m_fInertRolCor
  
  // Setter and getter for inertial adjustments
  float getInertPitCor();
  float getInertRolCor();
  
  void setInertPitCor(float fValDeg);
  void setInertRolCor(float fValDeg);
};

#endif
