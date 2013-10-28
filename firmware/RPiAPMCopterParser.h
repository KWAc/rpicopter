#ifndef PARSER_h
#define PARSER_h


//checksum verifier
uint8_t verify_chksum(char *str, char *chk) {
  hal.console->printf(str);
  hal.console->printf("\n");

  uint8_t nc = 0;
  for(int i=0; i<strlen(str); i++) 
    nc = (nc + str[i]) << 1;

  long chkl = strtol(chk, NULL, 16);                    // supplied chksum to long
  if(chkl == (long)nc)                                  // compare
    return true;

  return false;
}

// Parse incoming text
void parse_input(int &bytesAvail) {
  static uint32_t offset = 0;
  char buffer[32];
  memset(buffer, 0, sizeof(buffer) );

  for(; bytesAvail > 0; bytesAvail--) {
    char c = (char)hal.console->read();                 // read next byte
    if(c == '\n') {                                     // new line reached - process cmd
      buffer[offset] = '\0';                            // null terminator
      // process cmd
      char *str = strtok(buffer, "*");                  // str = roll, pit, thr, yaw
      char *chk = strtok(NULL, "*");                    // chk = chksum

      if(verify_chksum(str, chk)) {                     // if chksum OK
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
      offset = 0;
    }
    else if(c != '\r' && offset < sizeof(buffer)-1) {
      buffer[offset++] = c;                             // store in buffer and continue until newline
    }
  }
}

#endif
