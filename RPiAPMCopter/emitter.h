#ifndef EMITTER_h
#define EMITTER_h

#include <stdint.h>
#include <stddef.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
class Emitter {
public:
  Emitter(void (*pf_foo)(), int delay = 0);
  
  bool emit();
  void reset();
  uint32_t getDelay(uint16_t iNum);
  
private:
  bool bSend;
  int iDelay;
  void (*pfEmitter)();
};
///////////////////////////////////////////////////////////
// Container for emitter objects
///////////////////////////////////////////////////////////
class Emitters {
private:
  const AP_HAL::HAL *m_pHAL;

  uint32_t m_iFastTimer;
  uint32_t m_iMediTimer;
  uint32_t m_iSlowTimer;
  uint32_t m_iUslwTimer;
  int m_ifC, m_imC, m_isC, m_iuC;
  
protected:
  Emitter *m_fastList[8];
  Emitter *m_mediList[8];
  Emitter *m_slowList[8];
  Emitter *m_uslwList[8];
  
  inline void scheduler(Emitter **pEmitters, uint16_t iSize_N, uint32_t &iTimer, const uint16_t &iTickRate);

public:
  Emitters(const AP_HAL::HAL *);
  
  void addFastEmitter(Emitter *);
  void addMediEmitter(Emitter *);
  void addSlowEmitter(Emitter *);
  void addUslwEmitter(Emitter *);  
  
  void run();
};

#endif
