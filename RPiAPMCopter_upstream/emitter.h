#ifndef EMITTER_h
#define EMITTER_h

#include <stdint.h>
#include <stddef.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "config.h"


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
class Emitter {
public:
  Emitter(void (*pf_foo)(), uint16_t delay = 0, uint8_t mult = 1);

  bool emit();
  void reset();
  uint16_t getDelay();

  uint32_t getTimer();
  void setTimer(const uint32_t iTimer);
  
private:
  bool      m_bSend;
  uint32_t  m_iTimer;                       // Timer variable
  uint16_t  m_iDelay;                       // Certain delay which is added to the tick rate
  uint8_t   m_iDelayMultplr;                // multiplier for m_iDelay (helpful if many emitters share the same tick rate). If m_iDelayMultplr zero: m_iDelay is zero too
  void (*pfEmitter)();                      // function pointer
};
///////////////////////////////////////////////////////////
// Container for emitter objects
///////////////////////////////////////////////////////////
class Emitters {
private:
  const AP_HAL::HAL *m_pHAL;

  uint8_t   m_iItems;                       // Current number of items in the arrays below
  Emitter  *m_functionList[NO_PRC_SCHED];   // function list
  uint16_t  m_tickrateList[NO_PRC_SCHED];   // tick rates are intervals e.g.: Call rate is 100 ms + delay[ms]*multiplier 

  bool isEmitted(const uint8_t iInd);
  void resetAll();

public:
  Emitters(const AP_HAL::HAL *);

  void addEmitter(Emitter *pEmitter, uint16_t iTickRate);
  void run();
};

#endif
