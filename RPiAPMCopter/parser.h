#ifndef PARSER_h
#define PARSER_h

#include "setup.h"


uint8_t calc_chksum(char *str) {
  uint8_t nc = 0;
  for(int i=0; i < strlen(str); i++) 
    nc = (nc + str[i]) << 1;

  return nc;
}

//checksum verifier
bool verify_chksum(char *str, char *chk) {
  uint8_t nc = calc_chksum(str);

  long chkl = strtol(chk, NULL, 16);                    // supplied chksum to long
  if(chkl == (long)nc)                                  // compare
    return true;

  return false;
}

inline
uint16_t map(uint16_t x, uint16_t out_min, uint16_t out_max, uint16_t in_min = 1100, uint16_t in_max = 1900) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool radio_rc() {
  // array for raw channel values
  uint16_t channels[APM_IOCHANNEL_COUNT];
  memset(channels, 0, sizeof(channels) );
  
  if(hal.rcin->valid_channels() > 0) {
    hal.rcin->read(channels, 8);
    // Copy from channels array to something human readable - array entry 0 = input 1, etc.
    RC_CHANNELS[2] = channels[2];                                 // throttle
    RC_CHANNELS[3] = map(channels[3], -180, 180/*, 1068, 1915*/); // yaw
    RC_CHANNELS[1] = map(channels[1], -45, 45/*, 1077, 1915*/);   // pitch
    RC_CHANNELS[0] = map(channels[0], -45, 45/*, 1090, 1913*/);   // roll
    return true;
  }
  return true;
}

// remote control stuff
void parse_rc(char* buffer) {
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verify_chksum(str, chk) ) {                     // if chksum OK
    char *ch = strtok(str, ",");                    // first channel
    RC_CHANNELS[0] = (uint16_t)strtol(ch, NULL, 10);// parse       
    for(int i = 1; i < APM_IOCHANNEL_COUNT; i++) {  // loop through final 3 RC_CHANNELS
      char *ch = strtok(NULL, ",");
      RC_CHANNELS[i] = (uint16_t)strtol(ch, NULL, 10);   
    }
    iWiFiTimer = hal.scheduler->millis();          // update last valid packet
  } 
  memset(buffer, 0, sizeof(buffer));                // flush buffer after everything
}

// drift compensation
void parse_driftcomp(char* buffer) {
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verify_chksum(str, chk) ) {                    // if chksum OK
    char *cstr;

    for(int i = 0; i < COMP_ARGS; i++) {      // loop through final 3 RC_CHANNELS
      if(i == 0)
        cstr = strtok (buffer, ",");
      else cstr = strtok (NULL, ",");

      switch(i) {
        case 0:
          GYRO_ROL_COR = atof(cstr);
          GYRO_ROL_COR = GYRO_ROL_COR  > 10.f ? 10.f : GYRO_ROL_COR < -10.f ? -10.f : GYRO_ROL_COR;
        break;
        case 1:
          GYRO_PIT_COR = atof(cstr);
          GYRO_PIT_COR = GYRO_PIT_COR  > 10.f ? 10.f : GYRO_PIT_COR < -10.f ? -10.f : GYRO_PIT_COR;
        break;
      }
    }
  } 
  memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
}

void parse_gyrocalib(char* buffer) {
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verify_chksum(str, chk) ) {                    // if chksum OK
    char *cstr = strtok (buffer, ",");
    bool bcalib = (bool)atoi(cstr);

    // only if quadro is _not_ armed
    if(bcalib) {
      // This functions checks whether model is ready for a calibration
      attitude_calibration();
    }
  }

  memset(buffer, 0, sizeof(buffer));
}

/*
 * Changes the sensor type used for the battery monitor
 */
void parse_batterymon(char* buffer) {
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verify_chksum(str, chk) ) {                    // if chksum OK
    char *cstr = strtok (buffer, ",");
    int type = atoi(cstr);
    
    battery.setup_source(type);
  }

  memset(buffer, 0, sizeof(buffer));
}

float *parse_PID(char* buffer) {
  static float pids[PID_SIZE];
  memset(pids, 0, 3*sizeof(float) );

  if(buffer == NULL)
    return pids;

  char ckP[32], ckI[32], cimax[32];
  unsigned int i = 0, c = 0, p = 0;
  for(; i < strlen(buffer); i++) {
    if(buffer[i] == '\0') {
      break;
    } 
    else if(buffer[i] != ',') {
      switch(p) {
      case 0:
        ckP[c] = buffer[i];
        ckP[c+1] = '\0';
        break;
      case 1:
        ckI[c] = buffer[i];
        ckI[c+1] = '\0';
        break;
      case 2:
        cimax[c] = buffer[i];
        cimax[c+1] = '\0';
        break;
      }
      c++;
    } 
    else {
      p++;
      c = 0;
      continue;
    }
  }

  pids[0] = atof(ckP);
  pids[1] = atof(ckI);
  pids[2] = atof(cimax);

  return pids;
}

bool config_pids(char* buffer) { 
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verify_chksum(str, chk) ) {                    // if chksum OK
    float *pids = NULL;
    char *cstr;

    for(int i = 0; i < PID_ARGS; i++) {
      if(i == 0)
        cstr = strtok (buffer, ";");
      else cstr = strtok (NULL, ";");

      pids = parse_PID(cstr);      
      switch(i) {
      case 0:
        PIDS[PID_PIT_RATE].kP(pids[0]);
        PIDS[PID_PIT_RATE].kI(pids[1]);
        PIDS[PID_PIT_RATE].imax(pids[2]);
        break;
      case 1:
        PIDS[PID_ROL_RATE].kP(pids[0]);
        PIDS[PID_ROL_RATE].kI(pids[1]);
        PIDS[PID_ROL_RATE].imax(pids[2]);
        break;
      case 2:
        PIDS[PID_YAW_RATE].kP(pids[0]);
        PIDS[PID_YAW_RATE].kI(pids[1]);
        PIDS[PID_YAW_RATE].imax(pids[2]);
        break;
      case 3: 
        PIDS[PID_PIT_STAB].kP(pids[0]);
        PIDS[PID_ROL_STAB].kP(pids[1]);
        PIDS[PID_YAW_STAB].kP(pids[2]);
        break;
      }
    }
  }
  memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  return true;
}

// Parse incoming text
// str = "%d,%d,%d,%d * checksum" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
// str = "%d,%d,%d; %d,%d,%d; %d,%d,%d * checksum"
void parse_input(uint32_t bytesAvail) {
  static uint32_t offset = 0;
  char buffer[512];
  memset(buffer, 0, sizeof(buffer) );

  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)hal.console->read();                 // read next byte
    if(c == '\n') {                                     // new line reached - process cmd
      buffer[offset] = '\0';                            // null terminator

      char *ctype = strtok(buffer, "#");                // type of string
      char *command = strtok(NULL, "#");                // command string

      // process cmd
      if(strcmp(ctype, "RC") == 0) {
        parse_rc(command);
      }
      if(strcmp(ctype, "PID") == 0) {
        config_pids(command);
      }
      if(strcmp(ctype, "CMP") == 0) {
        parse_driftcomp(command);
      }
      if(strcmp(ctype, "GYR") == 0) {
        parse_gyrocalib(command);
      }
      if(strcmp(ctype, "BAT") == 0) {
        parse_batterymon(command);
      }

      offset = 0;
    } 
    else if(c != '\r' && offset < sizeof(buffer)-1) {
      buffer[offset++] = c;                             // store in buffer and continue until newline
    }
  }
}

#endif

