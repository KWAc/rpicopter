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
  hal.console->printf(str);
  hal.console->printf("\n");

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
    hal.console->printf("Invalid checksum\n");
    memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  }
}

// kP, kI, imax
float *parse_PID(char* buffer) {
  static float res_pid[PID_SIZE];
  memset(buffer, 0, PID_SIZE * sizeof(float) );

  if(buffer != NULL) {
    char *sub_cstr = strtok (buffer, ",");
    for(int i = 0; i < PID_SIZE && sub_cstr != NULL; i++) {
      sub_cstr = strtok (NULL, ",");
      res_pid[i] = atof(sub_cstr);
    }
  }

  return res_pid;
}

// PIT_RATE; ROL_RATE; YAW_RATE; STAB
bool config_pids(char* buffer) { 
  // process cmd
  char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
  char *chk = strtok(NULL, "*");                    // chk = chksum
  
  if(verify_chksum(str, chk) ) {                    // if chksum OK
    float *pids = NULL;
    char *sub_cstr;
    
    int i = 0; do {
      if(i == 0)
        sub_cstr = strtok (str, ";");
      else sub_cstr = strtok (NULL, ";");

      pids = parse_PID(sub_cstr);
      if(pids == NULL)
        return false;

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
      i++;
    } while(sub_cstr != NULL);
  }
  else {
    hal.console->printf("Invalid checksum\n");
    memset(buffer, 0, sizeof(buffer));              // flush buffer after everything
  }
    
  return true;
}

// Parse incoming text
// str = "%d,%d,%d,%d * checksum" % (p['roll'], p['pitch'], p['thr'], p['yaw'])
// str = "%d,%d,%d; %d,%d,%d; %d,%d,%d * checksum"
void parse_input(int &bytesAvail) {
  static uint32_t offset = 0;
  char buffer[128];
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
      else if(strcmp(ctype, "PID") == 0) {
        config_pids(command);
      }

      offset = 0;
    }
    else if(c != '\r' && offset < sizeof(buffer)-1) {
      buffer[offset++] = c;                             // store in buffer and continue until newline
    }
  }
}

#endif
