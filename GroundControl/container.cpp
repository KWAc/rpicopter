#include "container.h"


void RANGE::setF1 () {
    ROL_MIN = -10;
    ROL_MAX = 10;

    PIT_MIN = -10;
    PIT_MAX = 10;

    YAW_MIN = -45;
    YAW_MAX = 45;
}
void RANGE::setF2 () {
    ROL_MIN = -20;
    ROL_MAX = 20;

    PIT_MIN = -20;
    PIT_MAX = 20;

    YAW_MIN = -90;
    YAW_MAX = 90;
}
void RANGE::setF3 () {
    ROL_MIN = -30;
    ROL_MAX = 30;

    PIT_MIN = -30;
    PIT_MAX = 30;

    YAW_MIN = -135;
    YAW_MAX = 135;
}
void RANGE::setF4 () {
    ROL_MIN = -45;
    ROL_MAX = 45;

    PIT_MIN = -45;
    PIT_MAX = 45;

    YAW_MIN = -180;
    YAW_MAX = 180;
}

RANGE::RANGE() {
    ROL_MIN = -10;
    ROL_MAX = 10;

    PIT_MIN = -10;
    PIT_MAX = 10;

    YAW_MIN = -45;
    YAW_MAX = 45;

    THR_MIN = 1100;
    THR_MAX = 1900;

    THR_80P = 1650;
}


RC_COM::RC_COM() {
    ROL = 0; PIT = 0; YAW = 0; THR = 1100;
}

QString RC_COM::str_makeWiFiCommand() {
    QString com = "";
    com.append("{\"type\":\"rc\",\"r\":");  com.append(QString::number((int)ROL, 10) ); com.append(",");
    com.append("\"p\":");                   com.append(QString::number((int)PIT, 10) ); com.append(",");
    com.append("\"t\":");                   com.append(QString::number((int)THR, 10) ); com.append(",");
    com.append("\"y\":");                   com.append(QString::number((int)YAW, 10) ); com.append("}");

    return com;
}

int RC_COM::calc_chksum(char *str) {
  int nc = 0;
  for(unsigned int i = 0; i < strlen(str); i++)
    nc = (nc + str[i]) << 1;

  return nc;
}

QPair<int, char*> RC_COM::cstr_makeRadioCommand() {
    uint8_t thr_high  = ((int)THR/100)%10;
    uint8_t thr_low   = THR - (1000 + (((int)THR/100)%10) * 100);
    uint8_t ypm = YAW < 0 ? -1 : 1;
    uint8_t yaw = YAW < 0 ? (-1 * YAW) : YAW;
    
    m_cRadioCommand[0] = thr_high;
    m_cRadioCommand[1] = thr_low;
    m_cRadioCommand[2] = uint8_t(PIT+127);
    m_cRadioCommand[3] = uint8_t(ROL+127);
    m_cRadioCommand[4] = uint8_t(ypm+127);
    m_cRadioCommand[5] = uint8_t(yaw);
    
    uint8_t checksum = 0;
    for(unsigned int i = 0; i < 6; i++) {
      checksum = (checksum + m_cRadioCommand[i] ) << 1;
    }
    m_cRadioCommand[6] = checksum;
    m_cRadioCommand[7] = uint8_t(254);

    return QPair<int, char*> (8, m_cRadioCommand);
}

DRIFT_CAL::DRIFT_CAL() {
    ROL = 0; PIT = 0;
}

QString DRIFT_CAL::str_makeWiFiCommand() {
    QString com = "";
    com.append("{\"type\":\"cmp\",\"r\":"); com.append(QString::number(ROL, 'f', 2) ); com.append(",");
    com.append("\"p\":");                   com.append(QString::number(PIT, 'f', 2) ); com.append("}");

    return com;
}

QPair<int, char*> DRIFT_CAL::cstr_makeWiFiCommand() {
    QString com = str_makeWiFiCommand();

    memset(m_cWiFiCommand, 0, sizeof(m_cWiFiCommand) );
    for(unsigned int i = 0; i < static_cast<unsigned int>(com.size() ) && i < sizeof(m_cWiFiCommand); i++) {
        m_cWiFiCommand[i] = com.at(i).toLatin1();
    }
    return QPair<int, char*> (com.size(), m_cWiFiCommand);
}