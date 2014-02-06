#include "container.h"


Emitter::Emitter(void (*pf_foo)(), int delay) {
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

uint32_t Emitter::getDelay(uint16_t iNum) {
  return iDelay * (iNum+1);
}
