#ifndef OUT_h
#define OUT_h

#include "parser.h"


inline
void flash_leds(bool on) {
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

inline
void leds_off() {
    hal.gpio->write(A_LED_PIN, LED_OFF);
    hal.gpio->write(C_LED_PIN, LED_OFF);
}

inline
void leds_on() {
    hal.gpio->write(A_LED_PIN, LED_ON);
    hal.gpio->write(C_LED_PIN, LED_ON);
}

// remote control
void send_rc() { 
  int rcthr = RC_CHANNELS[2]; 
  int rcyaw = RC_CHANNELS[3];
  int rcpit = RC_CHANNELS[1];
  int rcrol = RC_CHANNELS[0];

  hal.console->printf("{\"type\":\"rc_input\",\"roll\":%d,\"pitch\":%d,\"thr\":%d,\"yaw\":%d}\n",
    rcrol, rcpit, rcthr, rcyaw);
}

// compass
void send_comp(float heading) {
  hal.console->printf("{\"type\":\"sens_comp\",\"heading\":%.4f}\n",
    heading);
}

// attitude in degrees
void send_attitude(float roll, float pitch, float yaw) {
  hal.console->printf("{\"type\":\"sens_attitude\",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f}\n",
    roll, pitch, yaw);
}

// barometer
void send_baro(BaroData data) {
  hal.console->printf("{\"type\":\"sens_baro\",\"pressure\":%.4f,\"altitude\":%.4f,\"temperature\":%.4f,\"climb_rate\":%.4f,\"samples\":%d}\n",
    data.pressure, data.altitude, data.temperature, data.climb_rate, (unsigned int)data.pressure_samples);
}

// gps
void send_gps(GPSData data) {
  hal.console->printf("{\"type\":\"sens_gps\",\"latitude\":%d,\"longitude\":%d,\"altitude_m\":%.4f,\"gspeed_ms\":%.4f,\"espeed_ms\":%.4f,\"nspeed_ms\":%.4f,\"dspeed_ms\":%.4f,\"h_x\":%.4f,\"h_y\":%.4f,\"h_z\":%.4f,\"gcourse_cd\":%d,\"satelites\":%d,\"tweek\":%d,\"tweek_s\":%d}\n",
    data.latitude, 
    data.longitude, 
    data.altitude_m, 
    
    data.gspeed_ms, 
    data.espeed_ms, 
    data.nspeed_ms, 
    data.dspeed_ms,
    
    data.heading_x,
    data.heading_y,
    data.heading_z,
    
    data.gcourse_cd, 
    data.satelites, 
    data.time_week, 
    data.time_week_s);
}

// battery monitor
void send_battery(BattData data) {
  hal.console->printf("{\"type\":\"sens_battery\",\"voltage_V\":%.4f,\"current_A\":%.4f,\"consumpt_mAh\":%.4f}\n",
    data.voltage_V, data.current_A, data.consumpt_mAh);
}

// PID configuration
void send_pids() {
  // Capture values
  float pit_rkp   = PIDS[PID_PIT_RATE].kP();
  float pit_rki   = PIDS[PID_PIT_RATE].kI();
  float pit_rimax = PIDS[PID_PIT_RATE].imax();

  float rol_rkp   = PIDS[PID_ROL_RATE].kP();
  float rol_rki   = PIDS[PID_ROL_RATE].kI();
  float rol_rimax = PIDS[PID_ROL_RATE].imax();
  
  float yaw_rkp   = PIDS[PID_YAW_RATE].kP();
  float yaw_rki   = PIDS[PID_YAW_RATE].kI();
  float yaw_rimax = PIDS[PID_YAW_RATE].imax();
  
  float pit_skp   = PIDS[PID_PIT_STAB].kP();
  float rol_skp   = PIDS[PID_ROL_STAB].kP();
  float yaw_skp   = PIDS[PID_YAW_STAB].kP();

  hal.console->printf("{\"type\":\"pid_config\",\"pit_rkp\":%.4f,\"pit_rki\":%.4f,\"pit_rimax\":%.4f,\"rol_rkp\":%.4f,\"rol_rki\":%.4f,\"rol_rimax\":%.4f,\"yaw_rkp\":%.4f,\"yaw_rki\":%.4f,\"yaw_rimax\":%.4f,\"pit_skp\":%.4f,\"rol_skp\":%.4f,\"yaw_skp\":%.4f}\n",
    pit_rkp, pit_rki, pit_rimax,
    rol_rkp, rol_rki, rol_rimax,
    yaw_rkp, yaw_rki, yaw_rimax,
    pit_skp, rol_skp, yaw_skp);
}

#endif
