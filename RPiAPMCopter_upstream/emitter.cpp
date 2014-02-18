#include "emitter.h"


Emitter::Emitter(void (*pf_foo)(), uint16_t delay, uint8_t mult) {
  m_bSend           = false;
  m_iDelay          = delay;
  pfEmitter         = pf_foo;
  m_iDelayMultplr   = mult;
  
  uint32_t m_iTimer = 0;
}

bool Emitter::emit() {
  if(!m_bSend && pfEmitter != NULL) {
    pfEmitter();
    m_bSend = true;
    return true;
  }
  return false;
}

void Emitter::reset() {
  m_bSend = false;
}

uint32_t Emitter::getTimer() {
  return m_iTimer;
}

void Emitter::setTimer(const uint32_t iTimer) {
  m_iTimer = iTimer;
}

uint16_t Emitter::getDelay() {
  return m_iDelay * m_iDelayMultplr;
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
Emitters::Emitters(const AP_HAL::HAL *p) {
  m_pHAL = p;

  memset(m_functionList, NULL, sizeof(m_functionList));
  memset(m_tickrateList, 0, sizeof(m_tickrateList));
  
  m_iItems = 0;
}

void Emitters::addEmitter(Emitter *p, uint16_t iTickRate) {
  if(m_iItems < NO_PRC_SCHED && p != NULL) {
    m_functionList[m_iItems] = p;
    m_tickrateList[m_iItems] = iTickRate;
    m_iItems++;
  }
}

bool Emitters::isEmitted(const uint8_t i) {
  Emitter *pCurEmit = m_functionList[i];
  uint32_t time = m_pHAL->scheduler->millis() - pCurEmit->getTimer();
  
  // Time yet to start the current emitter?
  if(time <= m_tickrateList[i] + pCurEmit->getDelay() ) {
    return false;
  } else {
    // Release the block for the transmitter
    pCurEmit->reset();
  }
  
  if(pCurEmit->emit() ) {
    // Set timer to the current time
    pCurEmit->setTimer(m_pHAL->scheduler->millis() );
  } else {
    return false;
  }
  
  return true;
}

void Emitters::resetAll() {
  // Reset everything if last emitter successfully emitted
  for(uint16_t i = 0; i < m_iItems; i++) {
    m_functionList[i]->reset();
  }
}

void Emitters::run() {
  if(m_pHAL == NULL)
    return;

  for(uint8_t i = 0; i < m_iItems; i++) {
    // Run all emitters
    if(!isEmitted(i) ) {
      continue;
    }
  }
}
