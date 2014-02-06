#ifndef CONTAINER_h
#define CONTAINER_h

#include <stdint.h>
#include <stddef.h>


class Emitter {
public:
  Emitter(void (*pf_foo)(), int delay = 0);
  
  bool emit();
  void reset();
  uint32_t getDelay(uint16_t iNum);
  
private:
  bool bSend;
  int iDelay;
  void (*pfEmitter)();
};

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

#endif
