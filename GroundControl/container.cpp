#include "container.h"


void RANGE::setF1 () {
    ROL_MIN = -15;
    ROL_MAX = 15;

    PIT_MIN = -15;
    PIT_MAX = 15;

    YAW_MIN = -60;
    YAW_MAX = 60;
}
void RANGE::setF2 () {
    ROL_MIN = -25;
    ROL_MAX = 25;

    PIT_MIN = -25;
    PIT_MAX = 25;

    YAW_MIN = -90;
    YAW_MAX = 90;
}
void RANGE::setF3 () {
    ROL_MIN = -35;
    ROL_MAX = 35;

    PIT_MIN = -35;
    PIT_MAX = 35;

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
    setF1();

    THR_MIN = 1100;
    THR_MAX = 1900;
    THR_80P = 1650;
}


RC_COM::RC_COM() {
    ROL = 0; PIT = 0; YAW = 0; THR = 1100;
    memset(m_cRadioCommand, 0, sizeof(m_cRadioCommand) );
    memset(m_cWiFiCommand, 0, sizeof(m_cWiFiCommand) );
}

QString RC_COM::str_makeWiFiCommand() {
    QString com = "";
    com.append("{\"t\":\"rc\",\"r\":");  com.append(QString::number((int)ROL, 10) ); com.append(",");
    com.append("\"p\":");                com.append(QString::number((int)PIT, 10) ); com.append(",");
    com.append("\"f\":");                com.append(QString::number((int)THR, 10) ); com.append(",");
    com.append("\"y\":");                com.append(QString::number((int)YAW, 10) ); com.append("}");

    return com;
}

int RC_COM::calc_chksum(char *str) {
  int nc = 0;
  for(unsigned int i = 0; i < strlen(str); i++)
    nc = (nc + str[i]) << 1;

  return nc;
}

QPair<int, char*> RC_COM::cstr_makeRadioCommand() {
    int_fast16_t iROL = (int)ROL;
    int_fast16_t iPIT = (int)PIT;
    int_fast16_t iYAW = (int)YAW;
    int_fast16_t iTHR = (int)THR;

    int_fast16_t thr_high  = (iTHR/100)%10;
    int_fast16_t thr_low   = iTHR - (1000 + ((iTHR/100)%10) * 100);

    int_fast16_t ypm = iYAW < 0 ? -1 : 1;
    int_fast16_t yaw = iYAW < 0 ? (-1 * YAW) : YAW;
    
    m_cRadioCommand[0] = thr_high;
    m_cRadioCommand[1] = thr_low;
    m_cRadioCommand[2] = iPIT;
    m_cRadioCommand[3] = iROL;
    m_cRadioCommand[4] = ypm;
    m_cRadioCommand[5] = yaw;
    
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
    memset(m_cWiFiCommand, 0, sizeof(m_cWiFiCommand) );
}

QString DRIFT_CAL::str_makeWiFiCommand() {
    QString com = "";
    com.append("{\"t\":\"cmp\",\"r\":"); com.append(QString::number(ROL, 'f', 2) ); com.append(",");
    com.append("\"p\":");                com.append(QString::number(PIT, 'f', 2) ); com.append("}");

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
