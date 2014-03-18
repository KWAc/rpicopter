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


///////////////////////////////////////////////////////////
// Container for sensor data and sensor configuration
///////////////////////////////////////////////////////////
class Device {
private:
  uint32_t m_iTimer;

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
  // x = pitch, y = roll, z = yaw
  Vector3f  m_vGyro;
  Vector3f  m_vAccel;
  Vector3f  m_vAttitude;
  // misc
  BaroData  m_ContBaro;
  GPSData   m_ContGPS;
  BattData  m_ContBat;

public:
  // Accepts pointers to abstract base classes to handle different sensor types
  Device( const AP_HAL::HAL *,
          AP_InertialSensor *, Compass *, AP_Baro *, GPS *, BattMonitor * );

  Vector3f attitude_calibration();

  void init_barometer();
  void init_pids();
  void init_compass();
  void init_inertial();
  void init_gps();
  void init_batterymon();

  /* Updating the inertial and calculates the attitude from fused sensor values */
  void update_inertial();       // saves result to m_vAttitude
  /* Not updating the intertial, to avoid double updates on other spots in the code :( Not elegant so far */
  Vector3f  read_gyro();        // converts sensor readout to degrees and saves in m_vGyro
  Vector3f  read_accel();       // converts sensor readout absolute attitude and saves in m_vAccel
  /* updating the sensors */
  BaroData  read_baro();
  float     read_comp(const float roll = 0.f, const float pitch = 0.f);
  GPSData   read_gps();
  BattData  read_bat();

  static float get_resbatcap(const float voltage_V, const uint8_t num_cells);

  /* Return the Vector3f Inertial readouts */
  Vector3f get_atti();  // fused sensor values from accelerometer/gyrometer and maybe compass/GPS later
  Vector3f get_gyro();  // gyrometer sensor readout
  Vector3f get_accel(); // accelerometer sensor readout
};

#endif
