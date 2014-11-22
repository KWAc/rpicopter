#include <AP_InertialSensor.h> // for user interactant
#include <AP_AHRS.h>
#include <RC_Channel.h>     // RC Channel Library

#include <float.h>

#include "receiver.h"
#include "scheduler.h"
#include "device.h"
#include "BattMonitor.h"
#include "arithmetics.h"



///////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////
inline void send_reply_msg(const AP_HAL::HAL *pHAL, char *cstr) {
  pHAL->console->printf("{\"t\":\"acpt\",\"c\":%s}\n", cstr);
}

inline uint_fast8_t calc_chksum(char *str) {
  uint_fast8_t nc = 0;
  for(size_t i = 0; i < strlen(str); i++) {
    nc = (nc + str[i]) << 1;
  }
  return nc;
}

//checksum verifier
inline bool verf_chksum(char *str, char *chk) {
  uint_fast8_t  nc  = calc_chksum(str);
  long chkl = strtol(chk, NULL, 16);  // supplied chksum to long
  if(chkl == static_cast<long>(nc) ) {              // compare
    return true;
  }
  return false;
}

inline void run_calibration(Device *pHalBoard, Scheduler *m_pOutSched) {
  while(pHalBoard->m_pHAL->console->available() ) {
    pHalBoard->m_pHAL->console->read();
  }

#if !defined( __AVR_ATmega1280__ )
  // Stop the sensor output
  m_pOutSched->stop();
  
  // Do the calibration
  float roll_trim, pitch_trim;
  AP_InertialSensor_UserInteractStream interact(pHalBoard->m_pHAL->console);
  pHalBoard->m_pInert->calibrate_accel(&interact, roll_trim, pitch_trim);
  
  // Adjust AHRS
  pHalBoard->m_pAHRS->set_trim(Vector3f(roll_trim, pitch_trim, 0) );
  
  // After all, resume the sensor output
  m_pOutSched->resume();
#else
	pHalBoard->m_pHAL->console->println_P(PSTR("calibrate_accel not available on 1280") );
#endif
}

inline float *parse_pid_substr(char* buffer) {
  static float rgfPIDS[PID_BUFFER_S] = { 0 };
  char rgcPIDS[PID_BUFFER_S][32];
  memset(rgfPIDS, 0, sizeof(rgfPIDS) );
  memset(rgcPIDS, 0, sizeof(rgcPIDS) );

  size_t i = 0, iPIDcstr = 0, iPID = 0;
  for(; i < strlen(buffer); i++) {
    // String ended here
    if(buffer[i] == '\0') {
      break;
    }
    // Avoid buffer overflow
    else if(iPID >= PID_BUFFER_S) {
      break;
    }
    // Reached new variable; Go over to next char
    else if(buffer[i] == ',') {
      iPID++;
      iPIDcstr = 0;
      continue;
    }
    // Read the current variable
    else {
      rgcPIDS[iPID][iPIDcstr]   = buffer[i];
      rgcPIDS[iPID][++iPIDcstr] = '\0';
    }
  }
  for (size_t i = 0; i <= iPID; i++) {
    rgfPIDS[i] = atof(rgcPIDS[i]);
  }
  return rgfPIDS;
}

inline bool check_input(int_fast16_t iRol, int_fast16_t iPit, int_fast16_t iThr, int_fast16_t iYaw) {
  if(!in_range(RC_PIT_MIN, RC_PIT_MAX, iPit) ) {
    return false;
  }
  if(!in_range(RC_ROL_MIN, RC_ROL_MAX, iRol) ) {
    return false;
  }
  if(!in_range(RC_THR_OFF, RC_THR_MAX, iThr) ) {
    return false;
  }
  if(!in_range(RC_YAW_MIN, RC_YAW_MAX, iYaw) ) {
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////
// Receiver
///////////////////////////////////////////////////////////////////////////////////////
Receiver::Receiver(Device *pHalBoard, Scheduler *pTMUart) {
  m_pHalBoard = pHalBoard;
  m_pTMUartOut = pTMUart;

  memset(m_cBuffer, 0, sizeof(m_cBuffer) );
  memset(m_rgChannelsRC, 0, sizeof(m_rgChannelsRC) );
  
  m_iPPMTimer = m_iSParseTimer_A = m_iSParseTimer_C = m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();
  m_iPPMTime  = m_iSParseTime_A  = m_iSParseTime_C  = m_iSParseTime  = 0;
  m_eErrors   = NOTHING_F;
  
  m_pRCRol = new RC_Channel(RC_ROL);
  m_pRCPit = new RC_Channel(RC_PIT);
  m_pRCThr = new RC_Channel(RC_THR);
  m_pRCYaw = new RC_Channel(RC_YAW);
  
  // setup radio
  if (m_pRCThr->radio_min == 0) {
    // cope with AP_Param not being loaded
    m_pRCThr->radio_min = RC_THR_OFF;
  }
  if (m_pRCThr->radio_max == 0) {
    // cope with AP_Param not being loaded
    m_pRCThr->radio_max = RC_THR_MAX;
  }
  
  // set rc channel ranges
  m_pRCRol->set_angle(RC_ROL_MAX*100);
  m_pRCPit->set_angle(RC_PIT_MAX*100);
  m_pRCThr->set_range(RC_THR_ACRO-RC_THR_OFF, RC_THR_MAX-RC_THR_OFF);
  m_pRCYaw->set_angle(RC_YAW_MAX*100);
}

void Receiver::set_channel(uint_fast8_t index, int_fast32_t value) {
  if(index >= APM_IOCHAN_CNT) {
    m_rgChannelsRC[APM_IOCHAN_CNT-1] = value;
  }
  m_rgChannelsRC[index] = value;
}

int_fast32_t Receiver::get_channel(uint_fast8_t index) const {
  if(index >= APM_IOCHAN_CNT) {
    return m_rgChannelsRC[APM_IOCHAN_CNT-1];
  }
  return m_rgChannelsRC[index];
}

int_fast32_t *Receiver::get_channels() {
  return &m_rgChannelsRC[0];
}

GPSPosition *Receiver::get_waypoint() {
  return &m_Waypoint;
}

uint_fast32_t Receiver::last_parse_t32() {
  m_iSParseTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer;

  if(m_iSParseTime > COM_PKT_TIMEOUT) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(add_flag(m_eErrors, UART_TIMEOUT_F) );
  }

  return m_iSParseTime;
}

uint_fast32_t Receiver::last_parse_uartA_t32() {
  m_iSParseTime_A = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_A;
  return m_iSParseTime_A;
}

uint_fast32_t Receiver::last_parse_uartC_t32() {
  m_iSParseTime_C = m_pHalBoard->m_pHAL->scheduler->millis() - m_iSParseTimer_C;
  return m_iSParseTime_C;
}

uint_fast32_t Receiver::last_rcin_t32() {
  m_iPPMTime = m_pHalBoard->m_pHAL->scheduler->millis() - m_iPPMTimer;
  return m_iPPMTime;
}

// remote control stuff
bool Receiver::parse_ctrl_com(char* buffer) {
  char *str = strtok(buffer, "*");                      // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                        // chk = chksum

  if(verf_chksum(str, chk) ) {                          // if chksum OK
    char *ch = strtok(str, ",");                        // first channel
    m_rgChannelsRC[0] = strtol(ch, NULL, 10);           // parse
    for(uint_fast8_t i = 1; i < APM_IOCHAN_CNT; i++) {  // loop through final 3 RC_CHANNELS
      char *ch = strtok(NULL, ",");
      m_rgChannelsRC[i] = strtol(ch, NULL, 10);
    }
    m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis(); // update last valid packet
  }
  return true;
}

// drift compensation
// maximum value is between -10 and 10 degrees
bool Receiver::parse_gyr_cor(char* buffer) {
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum
  bool bRet = false;

  float fRol = FLT_MAX;
  float fPit = FLT_MAX;
  if(verf_chksum(str, chk) ) {                      // if chksum OK
    char *cstr;

    for(uint_fast8_t i = 0; i < GYRO_ARGS; i++) {   // loop through final 3 RC_CHANNELS
      if(i == 0)
        cstr = strtok (buffer, ",");
      else cstr = strtok (NULL, ",");

      switch(i) {
        case 0:
          fRol = atof(cstr);
          break;
        case 1:
          fPit = atof(cstr);
          bRet = true;
          break;
      }
    }
  }

  // First simple check
  if(!in_range(-10.f, 10.f, fRol) || !in_range(-10.f, 10.f, fPit) ) {
    bRet = false;
  }
  // Second check
  if(bRet) {
    m_pHalBoard->set_trims(fRol, fPit);
  }

  return bRet;
}

bool Receiver::parse_waypoint(char *buffer) {
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum
  bool bRet = false;

  int_fast32_t lat           = 0;
  int_fast32_t lon           = 0;
  int_fast32_t alt_cm        = 0;
  GPSPosition::UAV_TYPE flag = GPSPosition::NOTHING_F;

  if(verf_chksum(str, chk) ) {                      // if chksum OK
    char *cstr;

    for(uint_fast8_t i = 0; i < GPSP_ARGS; i++) {   // loop through final 3 RC_CHANNELS
      if(i == 0) {
        cstr = strtok (buffer, ",");
      } else {
        cstr = strtok (NULL, ",");
      }

      switch(i) {
        case 0:
          lat    = atol(cstr);
          break;
        case 1:
          lon    = atol(cstr);
          break;
        case 2:
          alt_cm = atol(cstr);
          break;
        case 3:
          // Parse the type flag
          flag = static_cast<GPSPosition::UAV_TYPE>(atoi(cstr) );
          // Override the height if the flag is HLD_ALTITUDE_F
          if(flag == GPSPosition::HLD_ALTITUDE_F) {
            bool bOK = false;
            // Measure the current height
            alt_cm = Device::get_altitude_cm(m_pHalBoard, bOK);
            // If height measurement failed, then break it
            if(!bOK) {
              flag = GPSPosition::NOTHING_F;
            }
          }
          // Indicate everything went well
          bRet = true;
          break;
      }
    }
  }
  // Set new waypoint only if the everything worked out like expected
  if(bRet == true) {
    m_Waypoint = GPSPosition(lat, lon, alt_cm, flag);
  }
  return bRet;
}

bool Receiver::parse_gyr_cal(char* buffer) {
  if(m_rgChannelsRC[2] > RC_THR_ACRO) {
    return false;
  }

  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum
  
  if(verf_chksum(str, chk) ) {
    // process cmd
    bool bcalib = (bool)atoi(str);
    // only if quadro is _not_ armed
    if(bcalib) {
      // This functions checks whether model is ready for a calibration
      run_calibration(m_pHalBoard, m_pTMUartOut);
    }
  }

  return true;
}

bool Receiver::parse_comp_offs(char* buffer, int8_t &type) {
  if(!m_pHalBoard->m_pComp->healthy() ) {
    return false;
  }
  
  bool bRet = false;

  int8_t  iT = 0; // type of compensation: 0 for compass offsets and 1 for motor compensations
  uint8_t iN = 0; // compass index
  float   fX = FLT_MAX;
  float   fY = FLT_MAX;
  float   fZ = FLT_MAX;
  
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum
  
  if(verf_chksum(str, chk) ) {
    char *cstr;
    
    for(uint_fast8_t i = 0; i < COMP_ARGS; i++) {
      if(i == 0)
        cstr = strtok (buffer, ",");
      else cstr = strtok (NULL, ",");

      switch(i) {
        case 0:
          iT = atoi(cstr);
          break;
        case 1:
          iN = atoi(cstr);
          break;
        case 2:
          fX = atof(cstr);
          break;
        case 3:
          fY = atof(cstr);
          break;
        case 4:
          fZ = atof(cstr);
          bRet = true;
          break;
      }
    }
  }

  // First simple check
  if(iT < 0 || iT > 1) {
    bRet = false;
  }
  if(iN < 0 || iN > 5) {
    bRet = false;
  }
  if(!in_range(-45.f, 45.f, fX) || !in_range(-45.f, 45.f, fY) || !in_range(-45.f, 45.f, fZ) ) {
    bRet = false;
  }
  type = iT;
  if(iT == 0 && bRet) {
    // Correction for e.g. iron near compass
    m_pHalBoard->m_pComp->set_and_save_offsets(iN, Vector3f(fX, fY, fZ) );
  }
  if(iT == 1 && bRet) {
    // Motor compensation: Amp per throttle unit
    m_pHalBoard->m_pComp->set_motor_compensation(iN, Vector3f(fX, fY, fZ) );
    m_pHalBoard->m_pComp->save_motor_compensation();
  }
  
  return true;
}

/*
 * Changes the sensor type used for the battery monitor
 */
bool Receiver::parse_bat_type(char* buffer) {
  char *str = strtok(buffer, "*");                // str
  char *chk = strtok(NULL, "*");                  // chk = chksum

  if(verf_chksum(str, chk) ) {                    // if chksum OK
    int type = atoi(str);
    m_pHalBoard->m_pBat->setup_type(type);
    m_pHalBoard->m_pBat->init();
  }
  return true;
}

bool Receiver::parse_pid_conf(char* buffer) {
  if(m_rgChannelsRC[2] > RC_THR_ACRO) {             // If motors run: Do nothing!
    return false;
  }

  // process cmd
  bool bRet = false;
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum

  if(verf_chksum(str, chk) ) {                      // if chksum OK
    char *cstr;

    for(uint_fast8_t i = 0; i < PID_ARGS; i++) {
      if(i == 0)
        cstr = strtok (buffer, ";");
      else cstr = strtok (NULL, ";");

      float *pids = parse_pid_substr(cstr);
      switch(i) {
      case 0:
        m_pHalBoard->get_pid(PID_PIT_RATE).kP(pids[0]);
        m_pHalBoard->get_pid(PID_PIT_RATE).kI(pids[1]);
        m_pHalBoard->get_pid(PID_PIT_RATE).kD(pids[2]);
        m_pHalBoard->get_pid(PID_PIT_RATE).imax(pids[3]);
        break;
      case 1:
        m_pHalBoard->get_pid(PID_ROL_RATE).kP(pids[0]);
        m_pHalBoard->get_pid(PID_ROL_RATE).kI(pids[1]);
        m_pHalBoard->get_pid(PID_ROL_RATE).kD(pids[2]);
        m_pHalBoard->get_pid(PID_ROL_RATE).imax(pids[3]);
        break;
      case 2:
        m_pHalBoard->get_pid(PID_YAW_RATE).kP(pids[0]);
        m_pHalBoard->get_pid(PID_YAW_RATE).kI(pids[1]);
        m_pHalBoard->get_pid(PID_YAW_RATE).kD(pids[2]);
        m_pHalBoard->get_pid(PID_YAW_RATE).imax(pids[3]);
        break;
      case 3:
        m_pHalBoard->get_pid(PID_THR_RATE).kP(pids[0]);
        m_pHalBoard->get_pid(PID_THR_RATE).kI(pids[1]);
        m_pHalBoard->get_pid(PID_THR_RATE).kD(pids[2]);
        m_pHalBoard->get_pid(PID_THR_RATE).imax(pids[3]);
        break;
      case 4:
        m_pHalBoard->get_pid(PID_ACC_RATE).kP(pids[0]);
        m_pHalBoard->get_pid(PID_ACC_RATE).kI(pids[1]);
        m_pHalBoard->get_pid(PID_ACC_RATE).kD(pids[2]);
        m_pHalBoard->get_pid(PID_ACC_RATE).imax(pids[3]);
        break;
      case 5:
        m_pHalBoard->get_pid(PID_PIT_STAB).kP(pids[0]);
        m_pHalBoard->get_pid(PID_ROL_STAB).kP(pids[1]);
        m_pHalBoard->get_pid(PID_YAW_STAB).kP(pids[2]);
        m_pHalBoard->get_pid(PID_THR_STAB).kP(pids[3]);
        m_pHalBoard->get_pid(PID_ACC_STAB).kP(pids[4]);
        bRet = true;
        // Save the PIDs to the EEPROM
        m_pHalBoard->save_pids();
        break;
      }
    }
  }
  return bRet;
}

/*
 * Compact remote control packet system for the radio on Uart2,
 * Everything fits into 7 bytes
 */
bool Receiver::parse_radio(char *buffer) {
  int_fast16_t thr = 1000 + (static_cast<uint_fast16_t>(buffer[0]) * 100) + (uint_fast16_t)buffer[1];    // 1000 - 1900
  int_fast16_t pit = static_cast<int_fast16_t>(buffer[2]);                                               // -45° - 45°
  int_fast16_t rol = static_cast<int_fast16_t>(buffer[3]);                                               // -45° - 45°
  int_fast16_t yaw = static_cast<uint_fast8_t>(buffer[5]) * static_cast<int_fast16_t>(buffer[4]);        // -180° - 180°
  uint_fast8_t chk = static_cast<uint_fast8_t>(buffer[6]);                                               // checksum

  // Calculate checksum
  uint_fast8_t checksum = 0;
  for(uint_fast8_t i = 0; i < 6; i++) {
    checksum = (checksum + buffer[i]) << 1;
  }

  // Validity check: first checksum
  if(checksum != chk) {
    return false;
  }
  // Validity check: second ratings
  else if(!check_input(rol, pit, thr, yaw) ) {
    return false;
  }
  // Set values if everything seems fine
  else {
    m_rgChannelsRC[RC_ROL] = rol;
    m_rgChannelsRC[RC_PIT] = pit;
    m_rgChannelsRC[RC_THR] = thr;
    m_rgChannelsRC[RC_YAW] = yaw;

    m_iSParseTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
    return true;
  }
}

// Parse incoming text
// str = "%d,%d,%d,%d * checksum" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
// str = "%d,%d,%d; %d,%d,%d; %d,%d,%d * checksum"
bool Receiver::parse_cstr(char *buffer) {
  char *ctype = strtok(buffer, "#");                // type of string
  char *command = strtok(NULL, "#");                // command string

  // Remote control command
  if(strcmp(ctype, "RC") == 0) {
    return parse_ctrl_com(command);
  }
  // attitude offset for small imbalances
  else if(strcmp(ctype, "CMP") == 0) {
    bool bOk = parse_gyr_cor(command);
    if(bOk) {
      send_reply_msg(m_pHalBoard->m_pHAL, "cmp");
    }
    return bOk;
  }
  // Waypoints for UAV mode
  else if(strcmp(ctype, "UAV") == 0) {
    bool bOk = parse_waypoint(command);
    if(bOk) {
      send_reply_msg(m_pHalBoard->m_pHAL, "uav");
    }
    return bOk;
  }
  // Gyrometer calibration
  else if(strcmp(ctype, "GYR") == 0) {
    bool bOk = parse_gyr_cal(command);
    if(bOk) {
      send_reply_msg(m_pHalBoard->m_pHAL, "gyr");
    }
  }
  // PID regulator constants
  else if(strcmp(ctype, "PID") == 0) {
    bool bOk = parse_pid_conf(command);
    if(bOk) {
      send_reply_msg(m_pHalBoard->m_pHAL, "pid");
    }
    return bOk;
  }
  // Set battery type
  else if(strcmp(ctype, "BAT") == 0) {
    bool bOk = parse_bat_type(command);
    if(bOk) {
      send_reply_msg(m_pHalBoard->m_pHAL, "bat");
    }
    return bOk;
  }
  // Set battery type
  else if(strcmp(ctype, "COMP") == 0) {
    int8_t type = -1;
    bool bOk = parse_comp_offs(command, type);
    if(bOk) {
      if(type == 0) {
        send_reply_msg(m_pHalBoard->m_pHAL, "comp_o");
      }
      else if(type == 1) {
        send_reply_msg(m_pHalBoard->m_pHAL, "comp_m");
      }
    }
    return bOk;
  }
  // Nothing to do or maybe string broken
  else {
    return false;
  }
}

bool Receiver::read_radio(AP_HAL::UARTDriver *pIn, uint_fast16_t &offset) {
  bool bRet = false;
  uint_fast16_t bytesAvail = pIn->available();
  for(; bytesAvail > 0; bytesAvail--) {
    char c = static_cast<char>(pIn->read() );                 // read next byte
    
    // New radio message found
    if(c == static_cast<char>(254) ) {                        // this control char is not used for any other symbol
      m_cBuffer[offset] = '\0';                               // null terminator at 8th position
      if(offset != RADIO_MSG_LENGTH) {                        // theoretically a broken message can still be shorter than it should be
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0; //so break here if something was wrong
        return false;
      } else {                                                // message has perfect length and stop byte
        bRet = parse_radio(m_cBuffer);
        if(bRet) {
          m_iSParseTimer_C = m_iSParseTimer;
        }
        memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0; //so break here if something was wrong
      }
    }
    // For the radio, the command string has to be 7 bytes long
    else if(offset < RADIO_MSG_LENGTH) {
      m_cBuffer[offset++] = c;                                // store in buffer and continue until newline
    } 
    // If it is longer, but there is no stop byte, 
    // it is likely it is a different type of command string
    else {
      return false;
    }
  }

  return bRet;
}

bool Receiver::read_cstring(AP_HAL::UARTDriver *pIn, uint_fast16_t &offset) {
  bool bRet = false;
  
  uint_fast16_t bytesAvail = pIn->available();
  for(; bytesAvail > 0; bytesAvail--) {
    char c = static_cast<char>(pIn->read() );           // read next byte
 
    // error check if for a broken radio string, ..
    if(c == static_cast<char>(254) ) {
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
      return false;
    }
 
    // .. otherwise it should be regular string
    if(c == '\n') {                                     // new line or special termination signature (z): Process cmd
      m_cBuffer[offset] = '\0';                         // null terminator
      bRet = parse_cstr(m_cBuffer);
      if(bRet) {
        m_iSParseTimer_A = m_iSParseTimer;
      }
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
    }
    // Delimiter before end of the message, not used yet
    else if(c == '\r') {
      continue;
    }
    // End of regular message reached
    else if(offset < (sizeof(m_cBuffer)-1) ) {
      m_cBuffer[offset++] = c;                          // store in buffer and continue until newline
    }
    // Message seems to be broken, so reset buffer and return
    else {
      memset(m_cBuffer, 0, sizeof(m_cBuffer) ); offset = 0;
      return false;
    }
  }

  return bRet;
}

bool Receiver::read_uartX(AP_HAL::UARTDriver *pIn, bool bToggleRadio) {
  bool bRet = false;
  static uint_fast16_t offset = 0;
  
  // Try to read very brief command string from radio first, ..
  if(bToggleRadio) {
    bRet = read_radio(pIn, offset);
  }
  
  // .. otherwise treat it as JSON
  if(!bRet) {
    bRet = read_cstring(pIn, offset);
  }
  
  return bRet;
}

// In addition to the attitude control loop,
// reading from the radio or other input sources is the main performance sink.
// This function has the aim to be optimized as much as possible
bool Receiver::try_any() {
  bool bOK = false;

  // Try rcin (PPM radio)
  #if USE_RCIN
  bOK = read_rcin();
  #endif

  // Try WiFi over uartA
  #if USE_UART_A
  if(last_rcin_t32() > RCIN_TIMEOUT && !bOK) {
    bOK = read_uartX(m_pHalBoard->m_pHAL->uartA, false);
    // Disable sensor data output for uartC and enable it for uartA
    if(bOK) {
      m_pTMUartOut->set_arguments(UART_A);
    }
  }
  #endif

  // Try radio (433 or 900 MHz) over uartC
  #if USE_UART_C
  if(last_parse_uartA_t32() > UART_A_TIMEOUT && !bOK) {
    // If currently in other modes, radio could be still helpful
    if(!chk_fset(m_Waypoint.mode, GPSPosition::GPS_NAVIGATN_F) ) {
      // Disable sensor data output for uartA and enable it for uartC
      #if !DEBUG_OUT // only if the debug output is disabled
      m_pTMUartOut->set_arguments(UART_C);
      #endif
    }
    bOK = read_uartX(m_pHalBoard->m_pHAL->uartC, true);
  }
  #endif

  // Update the time for the last successful parse of a control string
  last_parse_t32();
  return bOK;
}

bool Receiver::read_rcin() {
  if(!m_pHalBoard->m_pHAL->rcin->new_input() ) {
    return false;
  }

  m_pRCPit->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_PIT) );
  m_pRCRol->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_ROL) );
  m_pRCThr->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_THR) );
  m_pRCYaw->set_pwm(m_pHalBoard->m_pHAL->rcin->read(RC_YAW) );
  
  int_fast16_t pit = m_pRCPit->control_in / 100;
  int_fast16_t rol = m_pRCRol->control_in / 100;
  int_fast16_t thr = m_pRCThr->control_in;
  int_fast16_t yaw = m_pRCYaw->control_in / 100;
  
  // Small validity check
  if(!check_input(rol, pit, thr, yaw) ) {
    return false;
  }

  // If check was successful we feed the input into or rc array
  m_rgChannelsRC[RC_THR] = thr;
  // dezi degree to degree
  m_rgChannelsRC[RC_PIT] = pit;
  m_rgChannelsRC[RC_ROL] = rol;
  m_rgChannelsRC[RC_YAW] = yaw;
  
  // Update timers
  m_iSParseTimer = m_iPPMTimer = m_pHalBoard->m_pHAL->scheduler->millis();           // update last valid packet
  return true;
}