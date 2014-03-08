#include "receiver.h"
#include "device.h"
#include "BattMonitor.h"


Receiver::Receiver(Device *pHalBoard) {
  memset(m_cBuffer, 0, sizeof(m_cBuffer) );
  m_pHalBoard = pHalBoard;
  memset(m_pChannelsRC, 0, sizeof(m_pChannelsRC) );
  m_iSParseTimer_A = m_iSParseTimer_C = m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();
}

uint_fast8_t Receiver::calc_chksum(char *str) {
  uint_fast8_t nc = 0;
  for(uint_fast16_t i = 0; i < strlen(str); i++) {
    nc = (nc + str[i]) << 1;
  }
  return nc;
}

//checksum verifier
bool Receiver::verf_chksum(char *str, char *chk) {
  uint_fast8_t nc = calc_chksum(str);

  uint_fast32_t chkl = strtol(chk, NULL, 16);                // supplied chksum to long
  if(chkl == (uint_fast32_t)nc)                              // compare
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
    m_pChannelsRC[0] = (uint_fast16_t)strtol(ch, NULL, 10);// parse
    for(uint_fast8_t i = 1; i < APM_IOCHANNEL_COUNT; i++) {  // loop through final 3 RC_CHANNELS
      char *ch = strtok(NULL, ",");
      m_pChannelsRC[i] = (uint_fast16_t)strtol(ch, NULL, 10);
    }
    m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
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

    for(uint_fast8_t i = 0; i < COMP_ARGS; i++) {            // loop through final 3 RC_CHANNELS
      if(i == 0)
        cstr = strtok (buffer, ",");
      else cstr = strtok (NULL, ",");

      switch(i) {
        case 0:
          m_pHalBoard->setInertRolCor(atof(cstr) );
          m_pHalBoard->setInertRolCor(m_pHalBoard->getInertRolCor()  > 10.f ? 10.f : m_pHalBoard->getInertRolCor() < -10.f ? -10.f : m_pHalBoard->getInertRolCor() );
        break;
        case 1:
          m_pHalBoard->setInertPitCor(atof(cstr) );
          m_pHalBoard->setInertPitCor(m_pHalBoard->getInertPitCor()  > 10.f ? 10.f : m_pHalBoard->getInertPitCor() < -10.f ? -10.f : m_pHalBoard->getInertPitCor() );
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
      m_pHalBoard->calibrate_inertial();
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
    uint_fast8_t type = atoi(cstr);

    m_pHalBoard->m_pBat->setup_source(type);
  }
  return true;
}

float *Receiver::parse_pid_substr(char* buffer) {
  static float pids[PID_SIZE];
  memset(pids, 0, 3*sizeof(float) );

  char ckP[32], ckI[32], cimax[32];
  uint_fast16_t i = 0, c = 0, p = 0;
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

    for(uint_fast8_t i = 0; i < PID_ARGS; i++) {
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

bool Receiver::read_uartA(uint_fast16_t bytesAvail) {
  static uint_fast16_t offset = 0;

  bool bRet = false;
  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)m_pHalBoard->m_pHAL->console->read();// read next byte
    if(c == '\n') {                                     // new line reached - process cmd
      m_cBuffer[offset] = '\0';                         // null terminator
      bRet = parse(m_cBuffer);
      if(bRet) {
        m_iSParseTimer_A = m_iSParseTimer;
      }
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
  int_fast16_t thr = 1000 + ((uint_fast8_t)buffer[0] * 100) + (uint_fast8_t)buffer[1]; // 1000 - 1900
  int_fast16_t pit = (uint_fast8_t)buffer[2] - 127;                               // -45° - 45°
  int_fast16_t rol = (uint_fast8_t)buffer[3] - 127;                               // -45° - 45°
  int_fast16_t yaw = (uint_fast8_t)buffer[5] * ((uint_fast8_t)buffer[4] - 127);        // -180° - 180°
  int_fast16_t chk = (uint_fast8_t)buffer[6];                                     // checksum

  // Calculate checksum
  uint_fast8_t checksum = 0;
  for(uint_fast8_t i = 0; i < 6; i++) {
    checksum = (checksum + buffer[i]) << 1;
  }

  // Validity check: 
  // First checksum
  if(checksum != chk)
    return false;
  // Later other values
  if(rol > 45 || rol < -45)
    return false;
  if(pit > 45 || pit < -45)
    return false;
  if(yaw > 180 || yaw < -180)
    return false;
  if(thr > RC_THR_80P || thr < RC_THR_MIN)
    return false;
    
  // Set values
  m_pChannelsRC[0] = rol;
  m_pChannelsRC[1] = pit;
  m_pChannelsRC[2] = thr;
  m_pChannelsRC[3] = yaw;
  
  m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
  return true;
}

bool Receiver::read_uartC(uint_fast16_t bytesAvail) {
  static uint_fast16_t offset = 0;

  bool bRet = false;
  for(; bytesAvail > 0; bytesAvail--) {
    // command must be exactly 8 bytes long
    if(offset > RADIO_MAX_OFFS) {                             // if message is longer than it should be
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;   // reset everything
      return false;                                           // and break loop
    }
    char c = (char)m_pHalBoard->m_pHAL->uartC->read();        // read next byte
    if(c == (char)254) {                                      // this control char is not used for any other symbol
      m_cBuffer[offset] = '\0';                               // null terminator at 8th position
      if(offset != RADIO_MAX_OFFS) {                          // theoretically a broken message can still be shorter than it should be
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0; //so break here if something was wrong
        return false;
      }
      else {                                                  // message has perfect length and stop byte
        bRet = parse_radio(m_cBuffer);
        if(bRet) {
          m_iSParseTimer_C = m_iSParseTimer;
        }
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
      }
    }
    else if(offset < sizeof(m_cBuffer)-1) {
      m_cBuffer[offset++] = c;                                // store in buffer and continue until newline
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

uint_fast32_t Receiver::timeLastSuccessfulParse() {
  return m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer;
}

uint_fast32_t Receiver::timeLastSuccessfulParse_uartA() {
  return m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_A;
}

uint_fast32_t Receiver::timeLastSuccessfulParse_uartC() {
  return m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_C;
}
