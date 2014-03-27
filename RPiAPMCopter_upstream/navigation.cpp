#include "navigation.h"
#include "device.h"
#include "receiver.h"
#include "exceptions.h"


UAVNav::UAVNav(Device *pDev, Receiver *pRecv, Exception *pExcp) {
  m_pHalBoard = pDev;
  m_pReceiver = pRecv;
  m_pExeption = pExcp;
}

void UAVNav::calc_dheading() {

}