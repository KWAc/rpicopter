#ifndef CONTAINER_h
#define CONTAINER_h

#include <stdint.h>
#include <stddef.h>


// barometer data container
struct BaroData {
  float   pressure;
  float   altitude;
  float   temperature;
  float   climb_rate;
  uint8_t pressure_samples;
};

// gps data container
struct GPSData {
  int16_t latitude;     // in degrees * 10,000,000
  int16_t longitude;    // in degrees * 10,000,000
  float   altitude_m;   // altitude in m

  float   gspeed_ms;    // ground speed in m/sec
  float   espeed_ms;    // velocity east
  float   nspeed_ms;    // velocity north
  float   dspeed_ms;    // velocity down

  float   heading_x;
  float   heading_y;
  float   heading_z;

  int16_t gcourse_cd;   // ground course in degree
  int16_t satelites;
  int16_t status_fix;
  int16_t time_week;
  int16_t time_week_s;
};

// battery monitor
struct BattData {
  float   voltage_V;
  float   current_A;
  float   consumpt_mAh;
};

#endif
