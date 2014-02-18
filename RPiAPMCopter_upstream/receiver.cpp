#include "receiver.h"
#include "device.h"
#include "BattMonitor.h"


Receiver::Receiver(Device *pHalBoard) {
  memset(m_cBuffer, 0, sizeof(m_cBuffer) );

  m_pHalBoard   = pHalBoard;

  memset(m_pChannelsRC, 0, sizeof(m_pChannelsRC) );
  m_iSerialTimer = m_pHalBoard->m_pHAL->scheduler->millis();
}

uint8_t Receiver::calc_chksum(char *str) {
  uint8_t nc = 0;
  for(int i = 0; i < strlen(str); i++) {
    nc = (nc + str[i]) << 1;
  }

  return nc;
}

//checksum verifier
bool Receiver::verf_chksum(char *str, char *chk) {
  uint8_t nc = calc_chksum(str);

  long chkl = strtol(chk, NULL, 16);                // supplied chksum to long
  if(chkl == (long)nc)                              // compare
    return true;

  return false;
}

// remote control stuff
bool Receiver::parse_ctrl_com(char* buffer) {
  if(m_pChannelsRC == NULL) {
    return false;
  }
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    char *ch = strtok(str, ",");                    // first channel
    m_pChannelsRC[0] = (uint16_t)strtol(ch, NULL, 10);// parse
    for(int i = 1; i < APM_IOCHANNEL_COUNT; i++) {  // loop through final 3 RC_CHANNELS
      char *ch = strtok(NULL, ",");
      m_pChannelsRC[i] = (uint16_t)strtol(ch, NULL, 10);
    }
    m_iSerialTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
  }
  return true;
}

// drift compensation
// maximum value is between -10 and 10 degrees
bool Receiver::parse_gyr_cor(char* buffer) {
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    char *cstr;

    for(int i = 0; i < COMP_ARGS; i++) {            // loop through final 3 RC_CHANNELS
      if(i == 0)
        cstr = strtok (buffer, ",");
      else cstr = strtok (NULL, ",");

      switch(i) {
        case 0:
          m_pHalBoard->m_fInertRolCor = atof(cstr);
          m_pHalBoard->m_fInertRolCor = m_pHalBoard->m_fInertRolCor  > 10.f ? 10.f : m_pHalBoard->m_fInertRolCor < -10.f ? -10.f : m_pHalBoard->m_fInertRolCor;
        break;
        case 1:
          m_pHalBoard->m_fInertPitCor = atof(cstr);
          m_pHalBoard->m_fInertPitCor = m_pHalBoard->m_fInertPitCor  > 10.f ? 10.f : m_pHalBoard->m_fInertPitCor < -10.f ? -10.f : m_pHalBoard->m_fInertPitCor;
        break;
      }
    }
  }
}

bool Receiver::parse_gyr_cal(char* buffer) {
  // If motors run: Do nothing!
  if(m_pChannelsRC == NULL || m_pHalBoard == NULL) {
    return false;
  } else if (m_pChannelsRC[2] > RC_THR_MIN) {
    return false;
  }

  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    char *cstr = strtok (buffer, ",");
    bool bcalib = (bool)atoi(cstr);

    // only if quadro is _not_ armed
    if(bcalib) {
      // This functions checks whether model is ready for a calibration
      m_pHalBoard->attitude_calibration();
    }
  }
  return true;
}

/*
 * Changes the sensor type used for the battery monitor
 */
bool Receiver::parse_bat_type(char* buffer) {
  if(m_pHalBoard == NULL) {
    return false;
  }
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    char *cstr = strtok (buffer, ",");
    int type = atoi(cstr);

    m_pHalBoard->m_pBat->setup_source(type);
  }
  return true;
}

float *Receiver::parse_pid_substr(char* buffer) {
  static float pids[PID_SIZE];
  memset(pids, 0, 3*sizeof(float) );

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

bool Receiver::parse_pid_conf(char* buffer) {
  if(m_pHalBoard == NULL) {
    return false;
  } else if(m_pChannelsRC[2] > RC_THR_MIN) {        // If motors run: Do nothing!
    return false;
  }

  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    float *pids = NULL;
    char *cstr;

    for(int i = 0; i < PID_ARGS; i++) {
      if(i == 0)
        cstr = strtok (buffer, ";");
      else cstr = strtok (NULL, ";");

      pids = parse_pid_substr(cstr);
      switch(i) {
      case 0:
        m_pHalBoard->m_pPIDS[PID_PIT_RATE].kP(pids[0]);
        m_pHalBoard->m_pPIDS[PID_PIT_RATE].kI(pids[1]);
        m_pHalBoard->m_pPIDS[PID_PIT_RATE].imax(pids[2]);
        break;
      case 1:
        m_pHalBoard->m_pPIDS[PID_ROL_RATE].kP(pids[0]);
        m_pHalBoard->m_pPIDS[PID_ROL_RATE].kI(pids[1]);
        m_pHalBoard->m_pPIDS[PID_ROL_RATE].imax(pids[2]);
        break;
      case 2:
        m_pHalBoard->m_pPIDS[PID_YAW_RATE].kP(pids[0]);
        m_pHalBoard->m_pPIDS[PID_YAW_RATE].kI(pids[1]);
        m_pHalBoard->m_pPIDS[PID_YAW_RATE].imax(pids[2]);
        break;
      case 3:
        m_pHalBoard->m_pPIDS[PID_PIT_STAB].kP(pids[0]);
        m_pHalBoard->m_pPIDS[PID_ROL_STAB].kP(pids[1]);
        m_pHalBoard->m_pPIDS[PID_YAW_STAB].kP(pids[2]);
        break;
      }
    }
  }
  return true;
}

bool Receiver::read_uartA(uint16_t bytesAvail) {
  static uint16_t offset = 0;

  bool bRet = false;
  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)m_pHalBoard->m_pHAL->console->read();// read next byte
    if(c == '\n') {                                     // new line reached - process cmd
      m_cBuffer[offset] = '\0';                         // null terminator
      bRet = parse(m_cBuffer);
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
    }
    else if(c != '\r' && offset < sizeof(m_cBuffer)-1) {
      m_cBuffer[offset++] = c;                          // store in buffer and continue until newline
    }
  }
  return bRet;
}
/* 
 * Compact remote control packet system for the radio on Uart2,
 * Everything fits into 7 bytes 
 */
bool Receiver::parse_radio(char *buffer) {
  int16_t throttle_high  = (uint8_t)buffer[0];       // 1. 1 - 9
  int16_t throttle_low   = (uint8_t)buffer[1];       // 2. 0 - 99
  int16_t pit            = (uint8_t)buffer[2] - 127; // 3. -45° - 45°
  int16_t rol            = (uint8_t)buffer[3] - 127; // 4. -45° - 45°
  int16_t yaw_pm         = (uint8_t)buffer[4] - 127; // 5. -1 || +1
  int16_t yaw_val        = (uint8_t)buffer[5];       // 6. yaw angle in degrees 180°
  
  int16_t chk            = (uint8_t)buffer[6];       // 7. checksum
  
  int16_t thr            = 1000 + (throttle_high * 100) + throttle_low;
  int16_t yaw            = yaw_val * yaw_pm;
  
  // Calculate checksum
  uint8_t checksum = 0;
  for(int i = 0; i < 6; i++) {
    checksum = (checksum + buffer[i]) << 1;
  }
  // First check
  if(checksum == chk) {  
    // Second check
    m_pChannelsRC[0] = rol > 45 ? 45 : rol < -45 ? -45 : rol;
    m_pChannelsRC[1] = pit > 45 ? 45 : pit < -45 ? -45 : pit;
    m_pChannelsRC[2] = thr > RC_THR_80P ? RC_THR_80P : thr < RC_THR_MIN ? RC_THR_MIN : thr;
    m_pChannelsRC[3] = yaw > 180 ? 180 : yaw < -180 ? -180 : yaw;
    
    m_iSerialTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
    return true;
  }
  return false;
}

bool Receiver::read_uartC(uint16_t bytesAvail) {
  static uint16_t offset = 0;

  bool bRet = false;
  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)m_pHalBoard->m_pHAL->uartC->read();  // read next byte
    if(c == (char)254) {                                // this control char is not used for any other symbol
      m_cBuffer[offset] = '\0';                         // null terminator
      
      bRet = parse_radio(m_cBuffer);
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
    }
    else if(offset < sizeof(m_cBuffer)-1) {
      m_cBuffer[offset++] = c;                          // store in buffer and continue until newline
    }
  }
  return bRet;
}

// Parse incoming text
// str = "%d,%d,%d,%d * checksum" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
// str = "%d,%d,%d; %d,%d,%d; %d,%d,%d * checksum"
bool Receiver::parse(char *buffer) {
  if(buffer == NULL) {
    return false;
  }

  char *ctype = strtok(buffer, "#");                // type of string
  char *command = strtok(NULL, "#");                // command string

  // process cmd
  if(strcmp(ctype, "RC") == 0) {
    return parse_ctrl_com(command);
  }
  if(strcmp(ctype, "PID") == 0) {
    return parse_pid_conf(command);
  }
  if(strcmp(ctype, "CMP") == 0) {
    return parse_gyr_cor(command);
  }
  if(strcmp(ctype, "GYR") == 0) {
    return parse_gyr_cal(command);
  }
  if(strcmp(ctype, "BAT") == 0) {
    return parse_bat_type(command);
  }
}

uint32_t Receiver::time_elapsed() {
  return m_pHalBoard->m_pHAL->scheduler->millis() - m_iSerialTimer;
}
