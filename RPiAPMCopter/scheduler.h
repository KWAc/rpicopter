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
  Task(void (*pf_foo)(), uint_fast16_t delay = 0, uint_fast8_t mult = 1);

  bool start();
  void reset();
  uint_fast16_t getDelay();

  uint_fast32_t getTimer();
  void setTimer(const uint_fast32_t iTimer);
  
private:
  bool m_bSend;
  uint_fast32_t m_iTimer;                   // Timer variable
  uint_fast16_t m_iDelay;                   // Certain delay which is added to the tick rate
  uint_fast8_t  m_iDelayMultplr;            // multiplier for m_iDelay (helpful if many emitters share the same tick rate). If m_iDelayMultplr zero: m_iDelay is zero too
  void (*pfTask)();                         // function pointer
};
///////////////////////////////////////////////////////////
// Simple task managemant
///////////////////////////////////////////////////////////
class Scheduler {
private:
  const AP_HAL::HAL *m_pHAL;

  uint_fast8_t   m_iItems;                        // Current number of items in the arrays below
  Task*          m_functionList[NO_PRC_SCHED];    // function list
  uint_fast16_t  m_tickrateList[NO_PRC_SCHED];    // tick rates are intervals e.g.: Call rate is 100 ms + delay[ms]*multiplier 

  bool isStarted(const uint_fast8_t iInd);

public:
  Scheduler(const AP_HAL::HAL *);

  void addTask(Task *pTask, uint_fast16_t iTickRate);
  void run();
  void resetAll();
};

#endif
