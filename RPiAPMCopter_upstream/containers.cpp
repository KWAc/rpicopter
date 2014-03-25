#include "containers.h"


BaroData::BaroData() {
  pressure_pa      = 0;
  altitude_cm      = 0;
  temperature_deg  = 0;
  climb_rate_cms   = 0;
  pressure_samples = 0;
}

GPSData::GPSData() {
  latitude    = 0;
  longitude   = 0;
  altitude_cm = 0;

  gspeed_cms  = 0;
  espeed_cms  = 0;
  nspeed_cms  = 0;
  dspeed_cms  = 0;

  heading_x   = 0;
  heading_y   = 0;
  heading_z   = 0;

  gcourse_cd  = 0;
  satelites   = 0;
  status_fix  = 0;
  time_week   = 0;
  time_week_s = 0;
}

GPSPosition::GPSPosition() {
  latitude    = 0;
  longitude   = 0;
  altitude_cm = 0;
  
  mode     = GPSPosition::NOTHING_F;
}

GPSPosition::GPSPosition(int_fast32_t lat, int_fast32_t lon, int_fast32_t alt, GPSPosition::UAV_TYPE flag) {
  latitude    = lat;
  longitude   = lon;
  altitude_cm = alt;
  
  mode        = flag;
}

BattData::BattData() {
  voltage_V   = 0;
  current_A   = 0;
  consumpt_mAh = 0;
}
