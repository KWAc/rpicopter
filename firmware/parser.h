#ifndef PARSER_h
#define PARSER_h


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
    RC_PACKET_T = hal.scheduler->millis();          // update last valid packet
    memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  } 
  else {
    memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  }
}

float *parse_PID(char* buffer) {
  static float pids[3];
  memset(pids, 0, 3*sizeof(float) );

  if(buffer == NULL)
    return pids;

  char ckP[32], ckI[32], cimax[32];
  unsigned int i = 0, c = 0, p = 0;
  for(; i < strlen(buffer); i++) {
    if(buffer[i] == '\0') {
      break;
    } else if(buffer[i] != ',') {
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
    } else {
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
    
    for(int i = 0; i < PID_NR_OF_ARGS; i++) {
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
  } else {
    memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  }
  return true;
}

// Parse incoming text
// str = "%d,%d,%d,%d * checksum" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
// str = "%d,%d,%d; %d,%d,%d; %d,%d,%d * checksum"
void parse_input(int &bytesAvail) {
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

      offset = 0;
    } else if(c != '\r' && offset < sizeof(buffer)-1) {
      buffer[offset++] = c;                             // store in buffer and continue until newline
    }
  }
}

#endif
