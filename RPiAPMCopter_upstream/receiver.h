#ifndef RECVR_h
#define RECVR_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "config.h"
#include "absdevice.h"
#include "containers.h"

#define CH_PIT 0
#define CH_ROL 1
#define CH_THR 2
#define CH_YAW 3


class Device;
class RC_Channel;


class Receiver : public AbsErrorDevice {
private:
  char    m_cBuffer[256];

  Device  *m_pHalBoard;

  uint_fast32_t m_iSParseTimer;   // Last successful read timer of command string from radio or wifi
  uint_fast32_t m_iSParseTimer_A; // Last successful read timer of command string from wifi
  uint_fast32_t m_iSParseTimer_C; // Last successful read timer of command string from radio
  uint_fast32_t m_iPPMTimer;

  uint_fast32_t m_iSParseTime;    // Last successful read time of command string from radio or wifi
  uint_fast32_t m_iSParseTime_A;  // Last successful read time of command string from wifi
  uint_fast32_t m_iSParseTime_C;  // Last successful read time of command string from radio
  uint_fast32_t m_iPPMTime;

  uint_fast8_t calc_chksum(char *);
  bool    verf_chksum     (char *str, char *chk);
  float*  parse_pid_substr(char *);

  // Inertial calibration
  void run_calibration();
  void init_radio();
  
protected:
  bool    parse_ctrl_com  (char *);
  bool    parse_radio     (char *); // Very compact to fit into 8 bytes, stop byte and checksum byte inclusive
  bool    parse_gyr_cor   (char *);
  bool    parse_gyr_cal   (char *);
  bool    parse_bat_type  (char *);
  bool    parse_pid_conf  (char *);
  bool    parse_waypoint  (char *);
  
  // Switch for all the different kind of commands to parse
  bool    parse           (char *);

public:
  Receiver(Device *pHalBoard);
  
  // Channels for the primary radio
  RC_Channel *m_pRCPit;
  RC_Channel *m_pRCRol;
  RC_Channel *m_pRCThr;
  RC_Channel *m_pRCYaw;

  // Eight channel remote control plus one for altitude hold (height in cm)
  int_fast32_t m_rgChannelsRC[APM_IOCHAN_CNT];
  // Current position for autonomous flight
  GPSPosition m_Waypoint;

  // Read from serial bus
  bool read_uartA(uint_fast16_t bytesAvail); // console in APM 2
  bool read_uartC(uint_fast16_t bytesAvail); // radio in APM 2
  bool read_rcin();                          // PPM radio source
  
  bool try_any();
  
  // time since last command string was parsed successfully from:
  uint_fast32_t last_parse_t32();       // general
  uint_fast32_t last_parse_uartA_t32(); // UART A
  uint_fast32_t last_parse_uartC_t32(); // UART C
  // Try to read ppm radio
  uint_fast32_t last_rcin_t32();        // ppm input site (radio)
};

#endif

