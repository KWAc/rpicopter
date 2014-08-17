#ifndef OUT_h
#define OUT_h

#include "global.h"
#include "containers.h"
#include "scheduler.h"


void send_comp();
void send_atti();
void send_baro();
void send_gps();
void send_bat();
void send_rc();
void send_pids_attitude();
void send_pids_altitude();

// function, delay, multiplier of the delay
Task outAtti   (&send_atti,          3,   1);
Task outRC     (&send_rc,            37,  1);
Task outComp   (&send_comp,          44,  1);
Task outBaro   (&send_baro,          66,  1);
Task outGPS    (&send_gps,           66,  2);
Task outBat    (&send_bat,           75,  1);
Task outPIDAtt (&send_pids_attitude, 133, 1);
Task outPIDAlt (&send_pids_altitude, 133, 2);

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
  if(!_HAL_BOARD.m_pComp->healthy() ) {
    return;
  }

  hal.console->printf("{\"type\":\"s_cmp\",\"h\":%.1f}\n",
  static_cast<double>(_HAL_BOARD.read_comp_deg() ) );
}
///////////////////////////////////////////////////////////
// attitude in degrees
///////////////////////////////////////////////////////////
void send_atti() {
  hal.console->printf("{\"type\":\"s_att\",\"r\":%.1f,\"p\":%.1f,\"y\":%.1f}\n",
  static_cast<double>(_HAL_BOARD.get_atti_cor_deg().y), 
  static_cast<double>(_HAL_BOARD.get_atti_cor_deg().x), 
  static_cast<double>(_HAL_BOARD.get_atti_cor_deg().z) );
}
///////////////////////////////////////////////////////////
// barometer
///////////////////////////////////////////////////////////
void send_baro() {
  if(!_HAL_BOARD.m_pBaro->healthy() ) {
    return;
  }

  BaroData baro = _HAL_BOARD.read_baro();
  hal.console->printf("{\"type\":\"s_bar\",\"p\":%.1f,\"a\":%ld,\"t\":%.1f,\"c\":%.1f,\"s\":%d}\n",
  static_cast<double>(baro.pressure_pa), 
  baro.altitude_cm, 
  static_cast<double>(baro.temperature_deg), 
  static_cast<double>(baro.climb_rate_cms), 
  static_cast<uint_fast16_t>(baro.pressure_samples) );
}
///////////////////////////////////////////////////////////
// gps
///////////////////////////////////////////////////////////
void send_gps() {
  // Has fix?
  if(!_HAL_BOARD.m_pGPS->status() > 1) {
    return;
  }

  GPSData gps = _HAL_BOARD.get_gps();
  hal.console->printf("{\"type\":\"s_gps\",\"lat_dege7\":%ld,\"lon_dege7\":%ld,\"a_cm\":%ld,\"g_cms\":%ld,\"g_cd\":%ld,\"sat\":%d,\"tw\":%d,\"tw_s\":%.2f}\n",
                      gps.latitude,
                      gps.longitude,
                      gps.altitude_cm,
                      
                      gps.gspeed_cms,
                      
                      gps.gcourse_cd,
                      gps.satelites,
                      gps.time_week,
                      static_cast<double>(gps.time_week_s) );
}
///////////////////////////////////////////////////////////
// battery monitor
///////////////////////////////////////////////////////////
void send_bat() {
  BattData bat = _HAL_BOARD.read_bat();
  hal.console->printf("{\"type\":\"s_bat\",\"R\":%.1f,\"V\":%.1f,\"A\":%.1f,\"P\":%.1f,\"c_mAh\":%.1f}\n",
                      static_cast<double>(bat.refVoltage_V),
                      static_cast<double>(bat.voltage_V), 
                      static_cast<double>(bat.current_A),
                      static_cast<double>(bat.power_W), 
                      static_cast<double>(bat.consumpt_mAh) );
}
///////////////////////////////////////////////////////////
// remote control
///////////////////////////////////////////////////////////
void send_rc() {
  int_fast16_t rcthr = _RECVR.get_channel(RC_THR);
  int_fast16_t rcyaw = _RECVR.get_channel(RC_YAW);
  int_fast16_t rcpit = _RECVR.get_channel(RC_PIT);
  int_fast16_t rcrol = _RECVR.get_channel(RC_ROL);

  hal.console->printf("{\"type\":\"rc_in\",\"r\":%d,\"p\":%d,\"t\":%d,\"y\":%d}\n",
                      rcrol, rcpit, rcthr, rcyaw);
}
///////////////////////////////////////////////////////////
// PID configuration
///////////////////////////////////////////////////////////
void send_pids_attitude() {
  // Capture values
  float pit_rkp   = _HAL_BOARD.get_pid(PID_PIT_RATE).kP();
  float pit_rki   = _HAL_BOARD.get_pid(PID_PIT_RATE).kI();
  float pit_rkd   = _HAL_BOARD.get_pid(PID_PIT_RATE).kD();
  float pit_rimax = _HAL_BOARD.get_pid(PID_PIT_RATE).imax();

  float rol_rkp   = _HAL_BOARD.get_pid(PID_ROL_RATE).kP();
  float rol_rki   = _HAL_BOARD.get_pid(PID_ROL_RATE).kI();
  float rol_rkd   = _HAL_BOARD.get_pid(PID_ROL_RATE).kD();
  float rol_rimax = _HAL_BOARD.get_pid(PID_ROL_RATE).imax();

  float yaw_rkp   = _HAL_BOARD.get_pid(PID_YAW_RATE).kP();
  float yaw_rki   = _HAL_BOARD.get_pid(PID_YAW_RATE).kI();
  float yaw_rkd   = _HAL_BOARD.get_pid(PID_YAW_RATE).kD();
  float yaw_rimax = _HAL_BOARD.get_pid(PID_YAW_RATE).imax();

  float pit_skp   = _HAL_BOARD.get_pid(PID_PIT_STAB).kP();
  float rol_skp   = _HAL_BOARD.get_pid(PID_ROL_STAB).kP();
  float yaw_skp   = _HAL_BOARD.get_pid(PID_YAW_STAB).kP();

  hal.console->printf("{\"type\":\"pid_cnf\","
                      "\"p_rkp\":%.2f,\"p_rki\":%.2f,\"p_rkd\":%.4f,\"p_rimax\":%.2f,"
                      "\"r_rkp\":%.2f,\"r_rki\":%.2f,\"r_rkd\":%.4f,\"r_rimax\":%.2f,"
                      "\"y_rkp\":%.2f,\"y_rki\":%.2f,\"y_rkd\":%.4f,\"y_rimax\":%.2f,"
                      "\"p_skp\":%.2f,\"r_skp\":%.2f,\"y_skp\":%.4f}\n",
                      static_cast<double>(pit_rkp), static_cast<double>(pit_rki), static_cast<double>(pit_rkd), static_cast<double>(pit_rimax),
                      static_cast<double>(rol_rkp), static_cast<double>(rol_rki), static_cast<double>(rol_rkd), static_cast<double>(rol_rimax),
                      static_cast<double>(yaw_rkp), static_cast<double>(yaw_rki), static_cast<double>(yaw_rkd), static_cast<double>(yaw_rimax),
                      static_cast<double>(pit_skp), static_cast<double>(rol_skp), static_cast<double>(yaw_skp) );
}

void send_pids_altitude() {
  // Capture values
  float thr_rkp   = _HAL_BOARD.get_pid(PID_THR_RATE).kP();
  float thr_rki   = _HAL_BOARD.get_pid(PID_THR_RATE).kI();
  float thr_rkd   = _HAL_BOARD.get_pid(PID_THR_RATE).kD();
  float thr_rimax = _HAL_BOARD.get_pid(PID_THR_RATE).imax();

  float acc_rkp   = _HAL_BOARD.get_pid(PID_ACC_RATE).kP();
  float acc_rki   = _HAL_BOARD.get_pid(PID_ACC_RATE).kI();
  float acc_rkd   = _HAL_BOARD.get_pid(PID_ACC_RATE).kD();
  float acc_rimax = _HAL_BOARD.get_pid(PID_ACC_RATE).imax();

  float thr_skp   = _HAL_BOARD.get_pid(PID_THR_STAB).kP();
  float acc_skp   = _HAL_BOARD.get_pid(PID_ACC_STAB).kP();

  hal.console->printf("{\"type\":\"pid_cnf\","
                      "\"t_rkp\":%.2f,\"t_rki\":%.2f,\"t_rkd\":%.4f,\"t_rimax\":%.2f,"
                      "\"a_rkp\":%.2f,\"a_rki\":%.2f,\"a_rkd\":%.4f,\"a_rimax\":%.2f,"
                      "\"t_skp\":%.2f,\"a_skp\":%.2f}\n",
                      static_cast<double>(thr_rkp), static_cast<double>(thr_rki), static_cast<double>(thr_rkd), static_cast<double>(thr_rimax),
                      static_cast<double>(acc_rkp), static_cast<double>(acc_rki), static_cast<double>(acc_rkd), static_cast<double>(acc_rimax),
                      static_cast<double>(thr_skp), static_cast<double>(acc_skp) );
}

#endif

