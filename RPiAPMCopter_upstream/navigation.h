#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;


inline float sigm_uav_f(float x, float mod);

class UAVNav {
private:  
  uint_fast32_t m_t32YawTimer;

  float  m_fDestin_deg;
  float  m_fDelta_deg;
  float  m_fCurrentYaw_deg;
  
  float  m_fTargetYaw_deg;

  Device*       m_pHalBoard;
  Receiver*     m_pReceiver;
  Exception*    m_pExeption;

  /*
   * Calculate the change in degrees 
   * necessary to head with the front of the frame to the target way point
   */
  void target_angle();
  int_fast32_t delta_angle();   // target angle minus current angle in degree * 1000
  
public:
  UAVNav(Device *, Receiver *, Exception *);

  // This function is overriding the remote control
  // and implementing the way the copter has to move to the defined target way-point
  virtual void run();
};