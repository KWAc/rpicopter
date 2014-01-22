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

// compass
void send_comp(float heading) {
  hal.console->printf("{\"type\":\"s_cmp\",\"h\":%.1f}\n",
    heading);
}

// attitude in degrees
void send_attitude(float roll, float pitch, float yaw) {
  hal.console->printf("{\"type\":\"s_att\",\"r\":%.1f,\"p\":%.1f,\"y\":%.1f}\n",
    roll, pitch, yaw);
}

// barometer
void send_baro(BaroData data) {
    hal.console->printf("{\"type\":\"s_bar\",\"p\":%.1f,\"a\":%.1f,\"t\":%.1f,\"c\":%.1f,\"s\":%d}\n",
    data.pressure, data.altitude, data.temperature, data.climb_rate, (unsigned int)data.pressure_samples);
}

// gps
void send_gps(GPSData data) {
  hal.console->printf("{\"type\":\"s_gps\",\"lat\":%d,\"lon\":%d,\"a_m\":%.1f,\"g_ms\":%.1f,\"e_ms\":%.1f,\"n_ms\":%.1f,\"d_ms\":%.1f,\"h_x\":%.1f,\"h_y\":%.1f,\"h_z\":%.1f,\"g_cd\":%d,\"sat\":%d,\"tw\":%d,\"tw_s\":%d}\n",
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
  hal.console->printf("{\"type\":\"s_bat\",\"V\":%.1f,\"A\":%.1f,\"c_mAh\":%.1f}\n",
    data.voltage_V, data.current_A, data.consumpt_mAh);
}

// remote control
void send_rc() { 
  int rcthr = RC_CHANNELS[2]; 
  int rcyaw = RC_CHANNELS[3];
  int rcpit = RC_CHANNELS[1];
  int rcrol = RC_CHANNELS[0];

  hal.console->printf("{\"type\":\"rc_in\",\"r\":%d,\"p\":%d,\"t\":%d,\"y\":%d}\n",
    rcrol, rcpit, rcthr, rcyaw);
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

  hal.console->printf("{\"type\":\"pid_cnf\",\"pit_rkp\":%.2f,\"pit_rki\":%.2f,\"pit_rimax\":%.2f,\"rol_rkp\":%.2f,\"rol_rki\":%.2f,\"rol_rimax\":%.2f,\"yaw_rkp\":%.2f,\"yaw_rki\":%.2f,\"yaw_rimax\":%.2f,\"pit_skp\":%.2f,\"rol_skp\":%.2f,\"yaw_skp\":%.2f}\n",
    pit_rkp, pit_rki, pit_rimax,
    rol_rkp, rol_rki, rol_rimax,
    yaw_rkp, yaw_rki, yaw_rimax,
    pit_skp, rol_skp, yaw_skp);
}

#endif
