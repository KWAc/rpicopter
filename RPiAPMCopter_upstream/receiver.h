#ifndef RECVR_h
#define RECVR_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "config.h"
#include "absdevice.h"

class Device;


class Receiver : public AbsErrorDevice {  
private:
  char    m_cBuffer[512];

  Device  *m_pHalBoard;
  
  uint_fast32_t m_iSParseTimer;   // Last successful read timer of command string from radio or wifi
  uint_fast32_t m_iSParseTimer_A; // Last successful read timer of command string from wifi
  uint_fast32_t m_iSParseTimer_C; // Last successful read timer of command string from radio

  uint_fast32_t m_iSParseTime;    // Last successful read time of command string from radio or wifi
  uint_fast32_t m_iSParseTime_A;  // Last successful read time of command string from wifi
  uint_fast32_t m_iSParseTime_C;  // Last successful read time of command string from radio
  
  uint_fast8_t calc_chksum(char *);
  bool    verf_chksum     (char *str, char *chk);
  float*  parse_pid_substr(char *);
  
protected:
  bool    parse_ctrl_com  (char *);
  bool    parse_radio     (char *); // Very compact to fit into 8 bytes, stop byte and checksum byte inclusive
  bool    parse_gyr_cor   (char *);
  bool    parse_gyr_cal   (char *);
  bool    parse_bat_type  (char *);
  bool    parse_pid_conf  (char *);

  bool    parse           (char *);

public:
  Receiver(Device *pHalBoard);

  // Eight channel remote control plus one for altitude hold (height in cm)
  int_fast32_t m_rgChannelsRC[APM_IOCHAN_CNT];
  
  // Read from serial bus
  bool read_uartA(uint_fast16_t bytesAvail); // console in APM 2
  bool read_uartC(uint_fast16_t bytesAvail); // radio in APM 2
  bool try_uartAC();
  
  // time since last command string was parsed successfully from:
  uint_fast32_t last_parse_t32();       // general
  uint_fast32_t last_parse_uartA_t32(); // UART A
  uint_fast32_t last_parse_uartC_t32(); // UART C
};

#endif

