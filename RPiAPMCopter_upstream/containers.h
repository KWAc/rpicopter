#ifndef CONTAINER_h
#define CONTAINER_h

#include <stdint.h>
#include <stddef.h>


// barometer data container
struct BaroData {
  float   pressure_pa;
  float   altitude_m;
  float   temperature_deg;
  float   climb_rate_ms;
  uint_fast8_t pressure_samples;
  
  BaroData();
};

// gps data container
struct GPSData {
  int_fast16_t latitude;     // in degrees * 10,000,000
  int_fast16_t longitude;    // in degrees * 10,000,000
  float   altitude_m;        // altitude in m

  float   gspeed_ms;         // ground speed in m/sec
  float   espeed_ms;         // velocity east
  float   nspeed_ms;         // velocity north
  float   dspeed_ms;         // velocity down

  float   heading_x;
  float   heading_y;
  float   heading_z;

  int_fast16_t gcourse_cd;   // ground course in degree
  int_fast16_t satelites;
  int_fast16_t status_fix;
  int_fast16_t time_week;
  int_fast16_t time_week_s;
  
  GPSData();
};

struct GPSPosition {
  enum UAV_TYPE {
    NOTHING_F = 1 << 0,
    HLD_ALTITUDE_F = 1 << 1,
    GPS_NAVIGATN_F = 1 << 2
  };

  int_fast16_t latitude;     // in degrees * 10,000,000
  int_fast16_t longitude;    // in degrees * 10,000,000
  int_fast16_t altitude_m;   // altitude in m
  
  UAV_TYPE m_eMode;
  
  GPSPosition();
  GPSPosition(int_fast16_t, int_fast16_t, int_fast16_t, GPSPosition::UAV_TYPE);
};

// battery monitor
struct BattData {
  float   voltage_V;
  float   current_A;
  float   consumpt_mAh;
  
  BattData();
};

#endif
