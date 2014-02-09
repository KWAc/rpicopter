#ifndef DEVICE_h
#define DEVICE_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "config.h"
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


float sensor_fuse(float angle_cor, float angle_fix, float time, float rate = 0.025);
float delta_angle(float fCurVal, float fOldVal);
float batt_rescapa(float voltage_V, unsigned int num_cells);

///////////////////////////////////////////////////////////
// Container for sensor data and sensor configuration
///////////////////////////////////////////////////////////
class Device {
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

  float m_fInertRolOffs;
  float m_fInertPitOffs;
  float m_fInertYawOffs;
  
  float m_fInertPitCor; // left to right or right to left
  float m_fInertRolCor; // front to back or back to front

// Maybe make it protected ..
public:
  float     m_fComp;
  Vector3f  m_vGyro;
  Vector3f  m_vAtti;
  BaroData  m_ContBaro;
  GPSData   m_ContGPS;
  BattData  m_ContBat;
  
  float m_fRol;
  float m_fPit;
  float m_fYaw;

public:
  // Accepts pointers to abstract base classes to handle different sensor types
  Device( const AP_HAL::HAL *,
          AP_InertialSensor *, Compass *, AP_Baro *, GPS *, BattMonitor * );

  Vector3f attitude_calibration();
  void gyro_drift(Vector3f &drift, Vector3f &offset, int &samples, float bias = 20);
  
  void init_barometer();
  void init_pids();
  void init_compass();
  void init_inertial();
  void init_gps();
  void init_batterymon();
  
  Vector3f  read_gyro();
  Vector3f  read_atti();
  BaroData  read_baro();
  float     read_comp(float roll = 0.f, float pitch = 0.f);
  GPSData   read_gps();
  BattData  read_bat();
};

#endif
