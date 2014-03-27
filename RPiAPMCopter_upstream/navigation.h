#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;


class UAVNav {
private:
  float      m_fHeading;

  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;

public:
  UAVNav(Device *, Receiver *, Exception *);

  /*
   * Calculate the change in degrees 
   * necessary to head with the front of the frame to the target way point
   */
  void calc_dheading();
};