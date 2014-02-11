#ifndef RECVR_h
#define RECVR_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "config.h"

class Device;


class Receiver {
private:
  char    m_cBuffer[512];
  
  Device  *m_pHalBoard;
  uint32_t m_iSerialTimer;

  uint8_t calc_chksum     (char *);
  bool    verf_chksum     (char *str, char *chk);
  float*  parse_pid_substr(char*);
  
protected:
  bool    parse_ctrl_com  (char*);
  bool    parse_gyr_cor   (char*);
  bool    parse_gyr_cal   (char*);
  bool    parse_bat_type  (char*);
  bool    parse_pid_conf  (char*);

  bool    parse           (char*);
  
public:
  Receiver(Device *pHalBoard);

  int16_t m_pChannelsRC[APM_IOCHANNEL_COUNT];
  // Read from serial bus 
  bool read_uartA(uint32_t bytesAvail); // console in APM 2
  bool read_uartC(uint32_t bytesAvail); // radio in APM 2
  // time since last command string was parsed successfully
  uint32_t  time_elapsed();
};

#endif

