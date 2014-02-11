#include "emitter.h"


Emitter::Emitter(void (*pf_foo)(), uint16_t delay) {
  bSend = false;
  iDelay = delay;
  pfEmitter = pf_foo;
}

bool Emitter::emit() {
  if(!bSend && pfEmitter != NULL) {
    pfEmitter();
    bSend = true;
    return true;
  }
  return false;
}

void Emitter::reset() {
  bSend = false;
}

uint16_t Emitter::getDelay(uint16_t iNum) {
  return iDelay * (iNum+1);
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
Emitters::Emitters(const AP_HAL::HAL *p) {
  m_pHAL = p;

  memset(m_fastList, NULL, sizeof(m_fastList));
  memset(m_mediList, NULL, sizeof(m_mediList));
  memset(m_slowList, NULL, sizeof(m_slowList));
  memset(m_uslwList, NULL, sizeof(m_uslwList));

  m_ifC = 0;
  m_imC = 0;
  m_isC = 0;
  m_iuC = 0;

  m_iFastTimer = 0;
  m_iMediTimer = 0;
  m_iSlowTimer = 0;
  m_iUslwTimer = 0;
}

void Emitters::addFastEmitter(Emitter *p) {
  if(m_ifC < sizeof(m_fastList)-1 && p != NULL) {
    m_fastList[m_ifC] = p;
    m_ifC++;
  }
}

void Emitters::addMediEmitter(Emitter *p) {
  if(m_imC < sizeof(m_fastList)-1  && p != NULL) {
    m_mediList[m_imC] = p;
    m_imC++;
  }
}

void Emitters::addSlowEmitter(Emitter *p) {
  if(m_isC < sizeof(m_fastList)-1  && p != NULL) {
    m_slowList[m_isC] = p;
    m_isC++;
  }
}

void Emitters::addUslwEmitter(Emitter *p) {
  if(m_iuC < sizeof(m_fastList)-1  && p != NULL) {
    m_uslwList[m_iuC] = p;
    m_iuC++;
  }
}
///////////////////////////////////////////////////////////
// pEmitters: Array of iSize_N elements
// iTickrate: the time in ms until the first emitter in the array will emit again
// pEmitters: Array of iSize_N elements
// iTickrate: the time in ms until the first emitter in the array will emit again
///////////////////////////////////////////////////////////
void Emitters::scheduler(Emitter **pEmitters, uint8_t iSize_N, uint32_t &iTimer, const int16_t iTickRate) {
  if(m_pHAL == NULL)
    return;

  uint32_t time = m_pHAL->scheduler->millis() - iTimer;
  for(uint8_t i = 0; i < iSize_N; i++) {
    if(time > iTickRate + pEmitters[i]->getDelay(i) ) {
      if(pEmitters[i]->emit() ) {
        if(i == (iSize_N - 1) ) { // Reset everything if last emitter successfully emitted
          for(uint16_t i = 0; i < iSize_N; i++) {
            pEmitters[i]->reset();
          }
          iTimer = m_pHAL->scheduler->millis();
        }
      }
    }
  }
}

void Emitters::run() {
  scheduler(m_fastList, m_ifC, m_iFastTimer, 75);
  scheduler(m_mediList, m_imC, m_iMediTimer, 1000);
  scheduler(m_slowList, m_isC, m_iSlowTimer, 2000);
  scheduler(m_uslwList, m_iuC, m_iUslwTimer, 5000);
}
