#ifndef OUT_h
#define OUT_h

#include "global.h"
#include "containers.h"
#include "emitter.h"


void send_comp();
void send_atti();
void send_baro();
void send_gps();
void send_bat();
void send_rc();
void send_pids();

Emitter emitComp(&send_comp, 44);
Emitter emitAtti(&send_atti, 3);
Emitter emitBaro(&send_baro, 66);
Emitter emitGPS(&send_gps, 66);
Emitter emitBat(&send_bat, 75);
Emitter emitRC(&send_rc, 37);
Emitter emitPID(&send_pids, 75);

///////////////////////////////////////////////////////////
// LED OUT
///////////////////////////////////////////////////////////
void flash_leds(bool on) {
  hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
  hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void leds_off() {
  hal.gpio->write(A_LED_PIN, LED_OFF);
  hal.gpio->write(C_LED_PIN, LED_OFF);
}

void leds_on() {
  hal.gpio->write(A_LED_PIN, LED_ON);
  hal.gpio->write(C_LED_PIN, LED_ON);
}
///////////////////////////////////////////////////////////
// compass
///////////////////////////////////////////////////////////
void send_comp() {
  hal.console->printf("{\"type\":\"s_cmp\",\"h\":%.1f}\n",
  hal_board.read_comp() );
}
///////////////////////////////////////////////////////////
// attitude in degrees
///////////////////////////////////////////////////////////
void send_atti() {
  Vector3f atti = hal_board.read_atti(); 
  hal.console->printf("{\"type\":\"s_att\",\"r\":%.1f,\"p\":%.1f,\"y\":%.1f}\n",
  hal_board.m_fRol, hal_board.m_fPit, hal_board.m_fYaw);
}
///////////////////////////////////////////////////////////
// barometer
///////////////////////////////////////////////////////////
void send_baro() {
  BaroData baro = hal_board.read_baro();
  hal.console->printf("{\"type\":\"s_bar\",\"p\":%.1f,\"a\":%.1f,\"t\":%.1f,\"c\":%.1f,\"s\":%d}\n",
  baro.pressure, baro.altitude, baro.temperature, baro.climb_rate, (unsigned int)baro.pressure_samples);
}
///////////////////////////////////////////////////////////
// gps
///////////////////////////////////////////////////////////
void send_gps() {
  GPSData gps = hal_board.read_gps();
  hal.console->printf("{\"type\":\"s_gps\",\"lat\":%d,\"lon\":%d,\"a_m\":%.1f,\"g_ms\":%.1f,\"e_ms\":%.1f,\"n_ms\":%.1f,\"d_ms\":%.1f,\"h_x\":%.1f,\"h_y\":%.1f,\"h_z\":%.1f,\"g_cd\":%d,\"sat\":%d,\"tw\":%d,\"tw_s\":%d}\n",
  gps.latitude, 
  gps.longitude, 
  gps.altitude_m, 

  gps.gspeed_ms, 
  gps.espeed_ms, 
  gps.nspeed_ms, 
  gps.dspeed_ms,

  gps.heading_x,
  gps.heading_y,
  gps.heading_z,

  gps.gcourse_cd, 
  gps.satelites, 
  gps.time_week, 
  gps.time_week_s);
}
///////////////////////////////////////////////////////////
// battery monitor
///////////////////////////////////////////////////////////
void send_bat() {
  BattData bat = hal_board.read_bat();
  hal.console->printf("{\"type\":\"s_bat\",\"V\":%.1f,\"A\":%.1f,\"c_mAh\":%.1f,\"r_cap\":%.1f}\n",
  bat.voltage_V, bat.current_A, bat.consumpt_mAh, batt_rescapa(bat.voltage_V, AP_BATT_CELL_COUNT) );
}
///////////////////////////////////////////////////////////
// remote control
///////////////////////////////////////////////////////////
void send_rc() { 
  int rcthr = RC_CHANNELS[2]; 
  int rcyaw = RC_CHANNELS[3];
  int rcpit = RC_CHANNELS[1];
  int rcrol = RC_CHANNELS[0];

  hal.console->printf("{\"type\":\"rc_in\",\"r\":%d,\"p\":%d,\"t\":%d,\"y\":%d}\n",
  rcrol, rcpit, rcthr, rcyaw);
}
///////////////////////////////////////////////////////////
// PID configuration
///////////////////////////////////////////////////////////
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

