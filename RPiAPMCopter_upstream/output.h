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
void send_pids();

// function, delay, multiplier of the delay 
Task outAtti(&send_atti, 3,  1);
Task outRC  (&send_rc,   37, 1);
Task outComp(&send_comp, 44, 1);
Task outBaro(&send_baro, 66, 1);
Task outGPS (&send_gps,  66, 2);
Task outBat (&send_bat,  75, 1);
Task outPID (&send_pids, 75, 2);

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
  if(!_HAL_BOARD.m_pComp->use_for_yaw() ) {
    return;
  }
  
  hal.console->printf("{\"type\":\"s_cmp\",\"h\":%.1f}\n",
  (double)_HAL_BOARD.read_comp_deg() );
}
///////////////////////////////////////////////////////////
// attitude in degrees
///////////////////////////////////////////////////////////
void send_atti() {
  hal.console->printf("{\"type\":\"s_att\",\"r\":%.1f,\"p\":%.1f,\"y\":%.1f}\n",
  (double)_HAL_BOARD.get_atti_raw_deg().y, (double)_HAL_BOARD.get_atti_raw_deg().x, (double)_HAL_BOARD.get_atti_raw_deg().z);
}
///////////////////////////////////////////////////////////
// barometer
///////////////////////////////////////////////////////////
void send_baro() {
  if(!_HAL_BOARD.m_pBaro->healthy) {
    return;
  }
  
  BaroData baro = _HAL_BOARD.read_baro();
  hal.console->printf("{\"type\":\"s_bar\",\"p\":%.1f,\"a\":%ld,\"t\":%.1f,\"c\":%.1f,\"s\":%d}\n",
  (double)baro.pressure_pa, baro.altitude_cm, (double)baro.temperature_deg, (double)baro.climb_rate_cms, (uint_fast16_t)baro.pressure_samples);
}
///////////////////////////////////////////////////////////
// gps
///////////////////////////////////////////////////////////
void send_gps() {
  if(!(*_HAL_BOARD.m_pGPS)->fix) {
    return;
  }
  
  GPSData gps = _HAL_BOARD.get_gps();
  hal.console->printf("{\"type\":\"s_gps\",\"lat\":%ld,\"lon\":%ld,\"a_cm\":%ld,\"g_cms\":%.1f,\"e_cms\":%.1f,\"n_cms\":%.1f,\"d_cms\":%.1f,\"h_x\":%.1f,\"h_y\":%.1f,\"h_z\":%.1f,\"g_cd\":%ld,\"sat\":%d,\"tw\":%d,\"tw_s\":%ld}\n",
  gps.latitude,
  gps.longitude,
  gps.altitude_cm,

  (double)gps.gspeed_cms,
  (double)gps.espeed_cms,
  (double)gps.nspeed_cms,
  (double)gps.dspeed_cms,

  (double)gps.heading_x,
  (double)gps.heading_y,
  (double)gps.heading_z,

  gps.gcourse_cd,
  gps.satelites,
  gps.time_week,
  gps.time_week_s);
}
///////////////////////////////////////////////////////////
// battery monitor
///////////////////////////////////////////////////////////
/*
 * Function from a LiPo charging chart:
 * 4,20 V  100%
 * 4,13 V       90%
 * 4,06 V       80%
 * 3,99 V       70%
 * 3,92 V       60%
 * 3,85 V       50%
 * 3,78 V       40%
 * 3,71 V       30%
 * 3,64 V       20%
 * 3,57 V       10%
 * 3,50 V  0%
 * Return: 0 - 1 (0%-100%) if in voltage range of this table :D
 */
/*
inline float remain_lipocap(const float voltage_V, const uint_fast8_t num_cells) {
  float fCap =  1.4286 * (voltage_V / (float)num_cells) - 5.f;
  return fCap < 0.f ? 0.f : fCap > 1.f ? 1.f : fCap;
}
*/
void send_bat() {
  BattData bat = _HAL_BOARD.read_bat();
  // TODO Add support for other battery types (NiMH, ..)
  //float fCapPerc = remain_lipocap(bat.voltage_V+AP_BATT_VOLT_OFFSET, AP_BATT_CELL_COUNT);
  /*
  hal.console->printf("{\"type\":\"s_bat\",\"V\":%.1f,\"A\":%.1f,\"c_mAh\":%.1f,\"r_cap\":%.1f}\n",
  (double)(bat.voltage_V+AP_BATT_VOLT_OFFSET), (double)bat.current_A, (double)bat.consumpt_mAh, (double)fCapPerc);
  */
  hal.console->printf("{\"type\":\"s_bat\",\"V\":%.1f,\"A\":%.1f,\"c_mAh\":%.1f}\n",
  (double)bat.voltage_V, (double)bat.current_A, (double)bat.consumpt_mAh);
}
///////////////////////////////////////////////////////////
// remote control
///////////////////////////////////////////////////////////
void send_rc() {
  int_fast16_t rcthr = _RECVR.m_rgChannelsRC[2];
  int_fast16_t rcyaw = _RECVR.m_rgChannelsRC[3];
  int_fast16_t rcpit = _RECVR.m_rgChannelsRC[1];
  int_fast16_t rcrol = _RECVR.m_rgChannelsRC[0];

  hal.console->printf("{\"type\":\"rc_in\",\"r\":%d,\"p\":%d,\"t\":%d,\"y\":%d}\n",
  rcrol, rcpit, rcthr, rcyaw);
}
///////////////////////////////////////////////////////////
// PID configuration
///////////////////////////////////////////////////////////
void send_pids() {
  // Capture values
  float pit_rkp   = _HAL_BOARD.m_rgPIDS[PID_PIT_RATE].kP();
  float pit_rki   = _HAL_BOARD.m_rgPIDS[PID_PIT_RATE].kI();
  float pit_rimax = _HAL_BOARD.m_rgPIDS[PID_PIT_RATE].imax();

  float rol_rkp   = _HAL_BOARD.m_rgPIDS[PID_ROL_RATE].kP();
  float rol_rki   = _HAL_BOARD.m_rgPIDS[PID_ROL_RATE].kI();
  float rol_rimax = _HAL_BOARD.m_rgPIDS[PID_ROL_RATE].imax();

  float yaw_rkp   = _HAL_BOARD.m_rgPIDS[PID_YAW_RATE].kP();
  float yaw_rki   = _HAL_BOARD.m_rgPIDS[PID_YAW_RATE].kI();
  float yaw_rimax = _HAL_BOARD.m_rgPIDS[PID_YAW_RATE].imax();

  float thr_rkp   = _HAL_BOARD.m_rgPIDS[PID_THR_RATE].kP();
  float thr_rki   = _HAL_BOARD.m_rgPIDS[PID_THR_RATE].kI();
  float thr_rimax = _HAL_BOARD.m_rgPIDS[PID_THR_RATE].imax();
  
  float acc_rkp   = _HAL_BOARD.m_rgPIDS[PID_ACC_RATE].kP();
  float acc_rki   = _HAL_BOARD.m_rgPIDS[PID_ACC_RATE].kI();
  float acc_rimax = _HAL_BOARD.m_rgPIDS[PID_ACC_RATE].imax();
  
  float pit_skp   = _HAL_BOARD.m_rgPIDS[PID_PIT_STAB].kP();
  float rol_skp   = _HAL_BOARD.m_rgPIDS[PID_ROL_STAB].kP();
  float yaw_skp   = _HAL_BOARD.m_rgPIDS[PID_YAW_STAB].kP();
  float thr_skp   = _HAL_BOARD.m_rgPIDS[PID_ROL_STAB].kP();
  float acc_skp   = _HAL_BOARD.m_rgPIDS[PID_YAW_STAB].kP();
  
  hal.console->printf("{\"type\":\"pid_cnf\","
                      "\"pit_rkp\":%.2f,\"pit_rki\":%.2f,\"pit_rimax\":%.2f,"
                      "\"rol_rkp\":%.2f,\"rol_rki\":%.2f,\"rol_rimax\":%.2f,"
                      "\"yaw_rkp\":%.2f,\"yaw_rki\":%.2f,\"yaw_rimax\":%.2f,"
                      "\"thr_rkp\":%.2f,\"thr_rki\":%.2f,\"thr_rimax\":%.2f,"
                      "\"acc_rkp\":%.2f,\"acc_rki\":%.2f,\"acc_rimax\":%.2f,"
                      "\"pit_skp\":%.2f,\"rol_skp\":%.2f,\"yaw_skp\":%.2f,\"thr_skp\":%.2f,\"acc_skp\":%.2f}\n",
                      (double)pit_rkp, (double)pit_rki, (double)pit_rimax,
                      (double)rol_rkp, (double)rol_rki, (double)rol_rimax,
                      (double)yaw_rkp, (double)yaw_rki, (double)yaw_rimax,
                      (double)thr_rkp, (double)thr_rki, (double)thr_rimax,
                      (double)acc_rkp, (double)acc_rki, (double)acc_rimax,
                      (double)pit_skp, (double)rol_skp, (double)yaw_skp, (double)thr_skp, (double)acc_skp);
}

#endif

