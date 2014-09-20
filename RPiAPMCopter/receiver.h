#ifndef RECVR_h
#define RECVR_h

#include <stdint.h>
#include <stddef.h>

#include <AP_Math.h>
#include <PID.h>

#include "config.h"
#include "absdevice.h"
#include "containers.h"


class Device;
class Scheduler;
class RC_Channel;

#define _

class Receiver : public AbsErrorDevice {
private /*variables*/:
  char          m_cBuffer[IN_BUFFER_S];         // Input buffer 
  int_fast32_t  m_rgChannelsRC[APM_IOCHAN_CNT]; // Eight channel remote control plus one for altitude hold (height in cm)
  GPSPosition   m_Waypoint;                     // Current position for autonomous flight
  
  Device       *m_pHalBoard;                    // Device module pointer
  Scheduler    *m_pTMUartOut;                   // Scheduler for standard output
  
  // Channels for the ppm radio
  RC_Channel   *m_pRCPit;
  RC_Channel   *m_pRCRol;
  RC_Channel   *m_pRCThr;
  RC_Channel   *m_pRCYaw;
  
  uint_fast32_t m_iSParseTimer;                 // Last successful read timer of command string from radio or wifi
  uint_fast32_t m_iSParseTimer_A;               // Last successful read timer of command string from wifi
  uint_fast32_t m_iSParseTimer_C;               // Last successful read timer of command string from radio
  uint_fast32_t m_iPPMTimer;

  uint_fast32_t m_iSParseTime;                  // Last successful read time of command string from radio or wifi
  uint_fast32_t m_iSParseTime_A;                // Last successful read time of command string from wifi
  uint_fast32_t m_iSParseTime_C;                // Last successful read time of command string from radio
  uint_fast32_t m_iPPMTime;
  
protected /*functions*/:
  bool    parse_ctrl_com  (char *);
  bool    parse_gyr_cor   (char *);
  bool    parse_gyr_cal   (char *);
  bool    parse_bat_type  (char *);
  bool    parse_pid_conf  (char *);
  bool    parse_waypoint  (char *);
  
  bool    parse_cstr      (char *);             // Switch for all the different kind of commands to parse
  bool    parse_radio     (char *);             // Very compact to fit into 8 bytes, stop byte and checksum byte inclusive
  
public /*functions*/:
  Receiver(Device *, Scheduler *);

  void          set_channel(uint_fast8_t, int_fast32_t);
  int_fast32_t  get_channel(uint_fast8_t) const;
  int_fast32_t *get_channels();
  GPSPosition  *get_waypoint();
  
  // time since last command string was parsed successfully from:
  uint_fast32_t last_parse_t32();                     // general
  uint_fast32_t last_parse_uartA_t32();               // UART A
  uint_fast32_t last_parse_uartC_t32();               // UART C
  uint_fast32_t last_rcin_t32();                      // PPM input (radio)

  // Read from serial bus
  bool          read_uartX(AP_HAL::UARTDriver *pIn, bool bToggleRadio);
  bool          read_radio(AP_HAL::UARTDriver *pIn, uint_fast16_t &iOffs);
  bool          read_cstring(AP_HAL::UARTDriver *pIn, uint_fast16_t &offset);
  
  bool          read_rcin();                          // PPM radio source
  bool          try_any();                            // This functions tries to read from any best input source. The order is: PPM radio, UartA, UartC
};

#endif

