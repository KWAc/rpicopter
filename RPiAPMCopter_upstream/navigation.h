#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;


class UAVNav {
private:
  uint_fast32_t m_t32YawTimer;

  float  m_dX;
  float  m_dY;

  float         m_fTargetYaw_deg;
  int_fast16_t  m_iTargetPit_deg;

  Device*       m_pHalBoard;
  Receiver*     m_pReceiver;
  Exception*    m_pExeption;

  /*
   * Calculate the change in degrees
   * necessary to head with the front of the frame to the target way point
   */
  float calc_error_deg();

  float width_of_merid_m(const float fLat_deg) const;
  float dist_2_equat_m  (const float fLat_deg) const;
  float dist_2_greenw_m (const float fLat_deg, const float fLon_deg) const;

public:
  UAVNav(Device *, Receiver *, Exception *);

  // This function is overriding the remote control
  // and implementing the way the copter has to move to the defined target way-point
  virtual int_fast16_t calc_yaw();
};