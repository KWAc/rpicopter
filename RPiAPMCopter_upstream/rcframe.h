#include <stdint.h>
#include <stddef.h>

#include "config.h"

class Device;
class Receiver;
class Exception;
class UAVNav;
class AP_MotorsQuad;
class RC_Channel;


/*
 * Abstract class:
 * Overload run() - This function is called in the main loop.
 */
class Frame {
protected:
  Device*    m_pHalBoard;
  Receiver*  m_pReceiver;
  Exception* m_pExeption;
  UAVNav*    m_pNavigation;

public:
  Frame(Device *, Receiver *, Exception *, UAVNav *);

  // Execute the magic here!
  virtual void run() = 0;
};

/*
 * Implementation of an quad-copter with X-configuration
 */
class M4XFrame : public Frame {
private:
  int_fast16_t _iPitOut;
  int_fast16_t _iRolOut;
  int_fast16_t _iThrOut;
  int_fast16_t _iYawOut;

  AP_MotorsQuad *m_pMotors;
  
protected:
  // Write to the motors
  void out();
  // Set the values to the defined minimum (see config.h)
  void clear();

  void calc_attitude_hold();
  void calc_altitude_hold();
  void calc_gpsnavig_hold();

public:
  M4XFrame(Device *, Receiver *, Exception *, UAVNav *);

  // Execute the magic here!
  void run();
  
  // Init the motor class
  void init_motors();
};
