#ifndef DEVICE_h
#define DEVICE_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "containers.h"
#include "absdevice.h"
#include "config.h"

class AP_InertialSensor;
class Compass;
class AP_Baro;
class GPS;
class BattMonitor;
class RangeFinder;

class BaroData;
class BattData;
class GPSData;

class PID;


///////////////////////////////////////////////////////////
// Container for sensor data and sensor configuration
///////////////////////////////////////////////////////////
class Device : public AbsErrorDevice {
private:
  uint_fast32_t m_iInrtTimer; // For calculating the derivative of the angular changes
  uint_fast32_t m_iAcclTimer; // for calculating the derivative of acceleration
  
  // Used with low path filter
  Vector3f m_vAccelPG_mss; // acceleration readout
  Vector3f m_vAccelMG_mss; // acceleration readout minus G constant (~9.81)
  Vector3f m_vAccelMG_ms;  // acceleration readout minus G constant (~9.81)

  // Set by inertial calibration
  float m_fInertRolOffs;
  float m_fInertPitOffs;
  float m_fInertYawOffs;
  
protected:
  float     m_fCmpH; // Compass heading
  float     m_fGpsH; // GPS heading
  // x = pitch, y = roll, z = yaw
  Vector3f  m_vGyro_deg;
  Vector3f  m_vAccel_deg;
  Vector3f  m_vAttitude_deg;
  float     m_fAltitude_m;
  // misc
  BaroData  m_ContBaro;
  GPSData   m_ContGPS;
  BattData  m_ContBat;
  
  // User set correction variables
  float m_fInertPitCor; // +/-: left to right or right to left
  float m_fInertRolCor; // +/-: front to back or back to front
  
  /* Not updating the intertial, to avoid double updates on other spots in the code :( Not elegant so far */
  Vector3f  read_gyro_deg();        // converts sensor relative readout to absolute attitude in degrees and saves in m_vGyro_deg
  Vector3f  read_accl_deg();       // converts sensor relative readout to absolute attitude and saves in m_vAccel_deg
  
public:
  // PID configuration and remote contro
  PID m_rgPIDS[NR_OF_PIDS];
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
  // Sonar
  RangeFinder        *m_pRF;

public:
  // Accepts pointers to abstract base classes to handle different sensor types
  Device( const AP_HAL::HAL *,
          AP_InertialSensor *, Compass *, AP_Baro *, GPS *, BattMonitor *, RangeFinder * );

  Vector3f calibrate_inertial();

  void init_barometer();
  void init_pids();
  void init_compass();
  void init_inertial();
  void init_gps();
  void init_batterymon();
  void init_rf();

  /* Updating the inertial and calculates the attitude from fused sensor values */
  void update_inertial();   // Calls: read_gyro_deg() and read_accl_deg() and saves results to m_vAttitude_deg, m_vGyro_deg and m_vAccel_deg
  /* updating the sensors */
  BaroData  read_baro();
  float     read_comp_deg(const float roll = 0.f, const float pitch = 0.f);
  GPSData   read_gps();
  BattData  read_bat();
// Ensure, there is a compiler error if sonar is not installed, but function used 
#ifdef SONAR_TYPE
  float     read_rf_m();        // Altitude estimate using range finder
#endif

  /* Return the Vector3f Inertial readouts */
  Vector3f get_atti_cor_deg();  // fused sensor values from accelerometer/gyrometer with m_fInertPitCor/m_fInertRolCor
  Vector3f get_atti_raw_deg();  // fused sensor values from accelerometer/gyrometer without m_fInertPitCor/m_fInertRolCor
  
  Vector3f get_gyro_cor_deg();  // gyrometer sensor readout with m_fInertPitCor/m_fInertRolCor
  Vector3f get_gyro_raw_deg();  // gyrometer sensor readout without m_fInertPitCor/m_fInertRolCor
  
  Vector3f get_accel_cor_deg(); // accelerometer sensor readout with m_fInertPitCor/m_fInertRolCor
  Vector3f get_accel_raw_deg(); // accelerometer sensor readout without m_fInertPitCor/m_fInertRolCor
  
  Vector3f get_accel_pg_mss();  // Acceleration with the G-const
  Vector3f get_accel_mg_mss();  // Acceleration without the G-const (filtered out)
  Vector3f get_accel_mg_ms();   // Speed estimation filtered out the G-const
  
  float    get_comp_deg();      // Just return the last estimated compass
  BaroData get_baro();          // Just return the filtered barometer data
  GPSData  get_gps();           // Just return the last estimated gps data
  BattData get_bat();           // Just return the last estimated battery data
// Ensure, there is a compiler error if sonar is not installed, but function used 
#ifdef SONAR_TYPE
  float    get_rf_m();          // Altitude estimate using range finder
#endif
  // Setter and getter for inertial adjustments
  float get_pit_cor();
  float get_rol_cor();
  
  void set_pit_cor(float fValDeg);
  void set_rol_cor(float fValDeg);
};

#endif
