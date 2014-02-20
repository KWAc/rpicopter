#include "scheduler.h"


Task::Task(void (*pf_foo)(), uint16_t delay, uint8_t mult) {
  m_bSend           = false;
  m_iDelay          = delay;
  pfTask            = pf_foo;
  m_iDelayMultplr   = mult;
  
  uint32_t m_iTimer = 0;
}

bool Task::start() {
  if(!m_bSend && pfTask != NULL) {
    pfTask();
    m_bSend = true;
    return true;
  }
  return false;
}

void Task::reset() {
  m_bSend = false;
}

uint32_t Task::getTimer() {
  return m_iTimer;
}

void Task::setTimer(const uint32_t iTimer) {
  m_iTimer = iTimer;
}

uint16_t Task::getDelay() {
  return m_iDelay * m_iDelayMultplr;
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
Scheduler::Scheduler(const AP_HAL::HAL *p) {
  m_pHAL = p;

  memset(m_functionList, NULL, sizeof(m_functionList));
  memset(m_tickrateList, 0, sizeof(m_tickrateList));
  
  m_iItems = 0;
}

void Scheduler::addTask(Task *p, uint16_t iTickRate) {
  if(m_iItems < NO_PRC_SCHED && p != NULL) {
    m_functionList[m_iItems] = p;
    m_tickrateList[m_iItems] = iTickRate;
    m_iItems++;
  }
}

bool Scheduler::isStarted(const uint8_t i) {
  Task *pCurTask = m_functionList[i];
  uint32_t time = m_pHAL->scheduler->millis() - pCurTask->getTimer();
  
  // Time yet to start the current emitter?
  if(time <= m_tickrateList[i] + pCurTask->getDelay() ) {
    return false;
  } else {
    // Release the block for the transmitter
    pCurTask->reset();
  }
  
  if(pCurTask->start() ) {
    // Set timer to the current time
    pCurTask->setTimer(m_pHAL->scheduler->millis() );
  } else {
    return false;
  }
  
  return true;
}

void Scheduler::resetAll() {
  // Reset everything if last emitter successfully emitted
  for(uint16_t i = 0; i < m_iItems; i++) {
    m_functionList[i]->reset();
  }
}

void Scheduler::run() {
  if(m_pHAL == NULL)
    return;

  for(uint8_t i = 0; i < m_iItems; i++) {
    // Run all tasks
    if(!isStarted(i) ) {
      continue;
    }
  }
}
