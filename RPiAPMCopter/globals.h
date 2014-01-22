#ifndef GLOB_h
#define GLOB_h

#include "defines.h"


// barometer data container 
struct BaroData {
  float pressure;
  float altitude;
  float temperature;
  float climb_rate;
  float pressure_samples;
};

// gps data container
struct GPSData {
  int latitude;         // in degrees * 10,000,000
  int longitude;        // in degrees * 10,000,000
  float altitude_m;     // altitude in m
  
  float gspeed_ms;      // ground speed in m/sec
  float espeed_ms;      // velocity east
  float nspeed_ms;      // velocity north
  float dspeed_ms;      // velocity down
  
  float heading_x;
  float heading_y;
  float heading_z;
  
  int gcourse_cd;       // ground course in degree
  int satelites;
  int status_fix;
  int time_week;
  int time_week_s;
};

// battery monitor
struct BattData {
  float voltage_V;
  float current_A;
  float consumpt_mAh;
};

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 inertial;

// Magnetometer aka compass
AP_Compass_HMC5843 compass;
bool COMPASS_INITIALIZED = 0;

// Barometer
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
bool BAROMETER_INITIALIZED = 0;

// GPS
AP_GPS_UBLOX gps;

// battery monitor
AP_BattMonitor battery;

// Motor control 
// PID array (6 pids, two for each axis)
PID PIDS[6];

float     OUT_HEADING   = 0.f;
BaroData  OUT_BARO;
GPSData   OUT_GPS;
BattData  OUT_BATT;

float OUT_PIT           = 0.f;
float OUT_ROL           = 0.f;
float OUT_YAW           = 0.f;

// Gyrometer calibration:
// After boot of the APM and before flight the gyrometer is calibrated on (even) ground
float GYRO_ROL_OFFS     = 0.f;
float GYRO_PIT_OFFS     = 0.f;
float GYRO_YAW_OFFS     = 0.f;

// On flight correction in case the quadrocopter is drifting (values _signed_)
// e.g. because of imbalances of the built, gyrometer not nice calibrated
float GYRO_ROL_COR      = 0.f; // left to right or right to left
float GYRO_PIT_COR      = 0.f; // front to back or back to front

// Remote control
uint32_t RC_PACKET_T = 0;
int16_t  RC_CHANNELS[APM_IOCHANNEL_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

#endif
