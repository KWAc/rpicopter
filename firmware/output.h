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
char* send_rc() {
  char buffer[512]; 
  memset(buffer, 0, sizeof(buffer));
  
  int rcthr = RC_CHANNELS[2]; 
  int rcyaw = RC_CHANNELS[3];
  int rcpit = RC_CHANNELS[1];
  int rcrol = RC_CHANNELS[0];
  /*
  // Make json string
  sprintf(buffer, "{\"type\":\"rc_input\",\
    \"roll\":%d,\
    \"pitch\":%d,\
    \"thr\":%d,\
    \"yaw\":%d}\r\n",
    rcrol, rcpit, rcthr, rcyaw);
  
  hal.console->printf(buffer);
  */
  hal.console->printf("{\"type\":\"rc_input\",\"roll\":%d,\"pitch\":%d,\"thr\":%d,\"yaw\":%d}\r\n",
    rcrol, rcpit, rcthr, rcyaw);
  
  return buffer;
}

// compass
char* send_comp(float heading) {
  char buffer[512]; 
  memset(buffer, 0, sizeof(buffer));
  /*
  // Make json string
  sprintf(buffer, "{\"type\":\"sens_comp\",\
    \"heading\":%.4f}\r\n",
    heading);
  
  hal.console->printf(buffer);
  */
  hal.console->printf( "{\"type\":\"sens_comp\",\"heading\":%.4f}\r\n",
    heading);

  return buffer;
}

// attitude in degrees
char* send_attitude(float roll, float pitch, float yaw) {
  char buffer[512]; 
  memset(buffer, 0, sizeof(buffer));
  
  // Make json string
  /*
  sprintf(buffer, "{\"type\":\"sens_attitude\",\
    \"roll\":%f,\
    \"pitch\":%f,\
    \"yaw\":%f}\r\n",
    roll, pitch, yaw);

  hal.console->printf(buffer);
  */
  hal.console->printf("{\"type\":\"sens_attitude\",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f}\r\n",
                      roll, pitch, yaw);
   
  return buffer;
}

// barometer
char* send_baro(bdata data) {
  char buffer[512]; 
  memset(buffer, 0, sizeof(buffer));
  /*
  // Make json string
  sprintf(buffer, "{\"type\":\"sens_baro\",\
    \"pressure\":%.4f,\
    \"altitude\":%.4f,\
    \"temperature\":%.4f,\
    \"climb_rate\":%.4f,\
    \"samples\":%d}\r\n",
    data.pressure, data.altitude, data.temperature, data.climb_rate, (unsigned int)data.pressure_samples);
  
  hal.console->printf(buffer);
  */
  hal.console->printf("{\"type\":\"sens_baro\",\"pressure\":%.4f,\"altitude\":%.4f,\"temperature\":%.4f,\"climb_rate\":%.4f,\"samples\":%d}\r\n",
    data.pressure, data.altitude, data.temperature, data.climb_rate, (unsigned int)data.pressure_samples);

  return buffer;
}

// PID configuration
char* send_pids() {
  char buffer[512]; 
  memset(buffer, 0, sizeof(buffer));

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
  float rol_skp   = PIDS[PID_PIT_STAB].kI();
  float yaw_skp   = PIDS[PID_PIT_STAB].imax();
  /*
  // Make json string
  sprintf(buffer, "{\"type\":\"pid_config\",\"pit_rkp\":%.4f,\"pit_rki\":%.4f,\"pit_rimax\":%.4f,\"rol_rkp\":%.4f,\"rol_rki\":%.4f,\"rol_rimax\":%.4f,\"yaw_rkp\":%.4f,\"yaw_rki\":%.4f,\"yaw_rimax\":%.4f,\"pit_skp\":%.4f,\"rol_skp\":%.4f,\"yaw_skp\":%.4f}\r\n",
    pit_rkp, pit_rki, pit_rimax,
    rol_rkp, rol_rki, rol_rimax,
    yaw_rkp, yaw_rki, yaw_rimax,
    pit_skp, rol_skp, yaw_skp);
  
  hal.console->printf(buffer);
  */
  hal.console->printf("{\"type\":\"pid_config\",\"pit_rkp\":%.4f,\"pit_rki\":%.4f,\"pit_rimax\":%.4f,\"rol_rkp\":%.4f,\"rol_rki\":%.4f,\"rol_rimax\":%.4f,\"yaw_rkp\":%.4f,\"yaw_rki\":%.4f,\"yaw_rimax\":%.4f,\"pit_skp\":%.4f,\"rol_skp\":%.4f,\"yaw_skp\":%.4f}\r\n",
    pit_rkp, pit_rki, pit_rimax,
    rol_rkp, rol_rki, rol_rimax,
    yaw_rkp, yaw_rki, yaw_rimax,
    pit_skp, rol_skp, yaw_skp);
  
  return buffer;
}

#endif
