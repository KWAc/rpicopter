#ifndef SCHEDULER_h
#define SCHEDULER_h

#include <stdint.h>
#include <stddef.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "config.h"


///////////////////////////////////////////////////////////
// Container for tasks
///////////////////////////////////////////////////////////
class Task {
public:
  Task(void (*pf_foo)(), uint16_t delay = 0, uint8_t mult = 1);

  bool start();
  void reset();
  uint16_t getDelay();

  uint32_t getTimer();
  void setTimer(const uint32_t iTimer);
  
private:
  bool      m_bSend;
  uint32_t  m_iTimer;                       // Timer variable
  uint16_t  m_iDelay;                       // Certain delay which is added to the tick rate
  uint8_t   m_iDelayMultplr;                // multiplier for m_iDelay (helpful if many emitters share the same tick rate). If m_iDelayMultplr zero: m_iDelay is zero too
  void (*pfTask)();                         // function pointer
};
///////////////////////////////////////////////////////////
// Simple task managemant
///////////////////////////////////////////////////////////
class Scheduler {
private:
  const AP_HAL::HAL *m_pHAL;

  uint8_t   m_iItems;                       // Current number of items in the arrays below
  Task*     m_functionList[NO_PRC_SCHED];   // function list
  uint16_t  m_tickrateList[NO_PRC_SCHED];   // tick rates are intervals e.g.: Call rate is 100 ms + delay[ms]*multiplier 

  bool isStarted(const uint8_t iInd);
  void resetAll();

public:
  Scheduler(const AP_HAL::HAL *);

  void addTask(Task *pTask, uint16_t iTickRate);
  void run();
};

#endif
