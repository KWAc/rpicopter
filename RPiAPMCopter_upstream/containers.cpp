#include "containers.h"


BaroData::BaroData() {
  pressure_pa      = 0;
  altitude_m       = 0;
  temperature_deg  = 0;
  climb_rate_ms    = 0;
  pressure_samples = 0;
}

GPSData::GPSData() {
  latitude    = 0;
  longitude   = 0;
  altitude_m  = 0;

  gspeed_ms   = 0;
  espeed_ms   = 0;
  nspeed_ms   = 0;
  dspeed_ms   = 0;

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
  altitude_m  = 0;
  
  m_eMode     = GPSPosition::NOTHING_F;
}

GPSPosition::GPSPosition(int_fast16_t lat, int_fast16_t lon, int_fast16_t alt, GPSPosition::UAV_TYPE flag) {
  latitude    = lat;
  longitude   = lon;
  altitude_m  = alt;
  
  m_eMode     = flag;
}

BattData::BattData() {
  voltage_V   = 0;
  current_A   = 0;
  consumpt_mAh = 0;
}
