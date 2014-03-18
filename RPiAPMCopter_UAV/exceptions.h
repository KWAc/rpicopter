#ifndef NAVIG_h
#define NAVIG_h

#include "containers.h"
#include "config.h"

class Device;
class Receiver;

class Exception {
private:
  Device*       m_pHalBoard;
  Receiver*     m_pReceiver;
  bool          m_bRcvrOvride;
  int_fast16_t  m_RcvrCom[APM_IOCHAN_CNT];                     // override remote control if certain exceptions happen
  
  uint_fast32_t m_iInertTimer;                                 // Timer for calculating the reduction of the throttle
  uint_fast32_t m_iAltitudeTimer;                              // Timer for reading the current altitude
  
  float m_fLastAltitude_m;
  float estim_altit(bool &);
  
  /*
   * Saves the current remote control command one time.
   * rls_recvr() must be called before, this function is usable again.
   */
  void lck_recvr();
  /*
   * Removes the lock for saving the remote control command.
   */
  void rls_recvr();
  
  /*
   * !Critical: Section!
   * This function only stops if the motors of the model stopped spinning!
   * There is no possibility to get control back before. 
   * Only use this function, if there are severe hardware problems.
   */
  void dev_take_down();
  /*
   * !Intermed. critical: Section!
   * This function stops if the receiver fetches a signal again
   */
  void rcvr_take_down();
  
protected:
  /*
   * This functions measures the height and reduces the throttle.
   */
  void reduce_thr(float fTime);
  
public:
  Exception(Device *, Receiver *);
  
  bool handle();                                              // Check all defines exceptions and call error handlers
};

#endif /*UAV_h*/
