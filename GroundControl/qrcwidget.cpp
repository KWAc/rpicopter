#include "qrcwidget.h"
#include <cassert>


void QRCWidget::sl_setRadioEnabled(bool state) {
    m_bRadioEnabled = state;
}

QRCWidget::QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort, QWidget *parent) : QAbsFrame(parent) {
    assert(pSock != NULL);
    m_pUdpSock = pSock;

    m_pSerialPort = pSerialPort;

    this->setMinimumSize(480, 480);
    setFocusPolicy(Qt::StrongFocus);

    m_bRadioEnabled = true;
    m_bAltitudeHold = false;

    m_iUpdateTime = 20;
    m_fTimeConstRed = (float)m_iUpdateTime/150.f;
    m_fTimeConstEnh = (float)m_iUpdateTime/75.f;


    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyPressHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyReleaseHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_sendRC2UDP() ) );
}

void QRCWidget::start() {
    m_keyEventTimer.start(m_iUpdateTime);
}

void QRCWidget::stop() {
    m_keyEventTimer.stop();
}

int CUSTOM_KEY::mapCustomKeyIndex(const int key) {
    if(key >= 0x01000000) {
        return 256 + (key ^ 0x01000000);
    } else {
        return key;
    }
}

void QRCWidget::initGyro2UDP() {
    if(m_COM.THR > m_RANGE.THR_MIN) {
        qDebug() << "Gyrometer calibration failed, because throttle is too high";
        return;
    }

    QString com = "";
    com.append("{\"type\":\"gyr\",\"cal\":"); 
    com.append(QString::number(true) ); 
    com.append("}");

    this->stop();
    qDebug() << "Try to start gyrometer calibration";
    sendJSON2UDP(com, false);
    
    this->start();
}

void QRCWidget::activAltihold2UDP() {
    QString com = "";
    com.append("{\"type\":\"uav\",\"lat_d\":");
    com.append(QString::number(0) );
    com.append(",\"lon_d\":");
    com.append(QString::number(0) );
    com.append(",\"alt_m\":");
    com.append(QString::number(0) );
    com.append(",\"flag_t\":");
    com.append(QString::number(1 << 1) ); // HLD_ALTITUDE_F
    com.append("}");

    qDebug() << "Try to init altitude hold";
    sendJSON2UDP(com, false);
}

void QRCWidget::deactAltihold2UDP() {
    QString com = "";
    com.append("{\"type\":\"uav\",\"lat_d\":");
    com.append(QString::number(0) );
    com.append(",\"lon_d\":");
    com.append(QString::number(0) );
    com.append(",\"alt_m\":");
    com.append(QString::number(0) );
    com.append(",\"flag_t\":");
    com.append(QString::number(1 << 0) ); // NOTHING_F
    com.append("}");

    qDebug() << "Try to init normal mode";
    sendJSON2UDP(com, false);
}

void QRCWidget::sl_customKeyPressHandler() {
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Backspace)] == true) {
        m_COM.PIT = 0;
        m_COM.ROL = 0;
        m_COM.YAW = 0;
        m_COM.THR = m_RANGE.THR_MIN;

        m_DRIFT.ROL = 0; 
        m_DRIFT.PIT = 0;

        sendJSON2UDP(m_DRIFT.str_makeWiFiCommand(), false);
    }

    // Inertial calibration
    if(m_customKeyStatus[Qt::Key_C] == true) {
        //qDebug() << "C";
        initGyro2UDP();
        m_customKeyStatus[Qt::Key_C] = false;
    }

    if(m_customKeyStatus[Qt::Key_H] == true) {
        //qDebug() << "H";
        if(!m_bAltitudeHold) {
            activAltihold2UDP();
            m_bAltitudeHold = true;
        }
    }

    if(m_customKeyStatus[Qt::Key_W] == true) {
        qDebug() << "W";
        if(m_COM.PIT > 0) {
            m_COM.PIT = 0;
        }

        if(m_COM.PIT + m_RANGE.PIT_MIN/10 >= m_RANGE.PIT_MIN)
            m_COM.PIT += m_RANGE.PIT_MIN/10 * m_fTimeConstEnh;
        else m_COM.PIT = m_RANGE.PIT_MIN;
    }
    if(m_customKeyStatus[Qt::Key_S] == true) {
        qDebug() << "S";
        if(m_COM.PIT < 0) {
            m_COM.PIT = 0;
        }

        if(m_COM.PIT + m_RANGE.PIT_MAX/10 <= m_RANGE.PIT_MAX)
            m_COM.PIT += m_RANGE.PIT_MAX/10 * m_fTimeConstEnh;
        else m_COM.PIT = m_RANGE.PIT_MAX;
    }

    if(m_customKeyStatus[Qt::Key_A] == true) {
        qDebug() << "A";
        if(m_COM.ROL > 0) {
            m_COM.ROL = 0;
        }

        if(m_COM.ROL + m_RANGE.ROL_MIN/10 >= m_RANGE.ROL_MIN)
            m_COM.ROL += m_RANGE.ROL_MIN/10 * m_fTimeConstEnh;
        else m_COM.ROL = m_RANGE.ROL_MIN;
    }
    if(m_customKeyStatus[Qt::Key_D] == true) {
        //qDebug() << "D";
        if(m_COM.ROL < 0) {
            m_COM.ROL = 0;
        }

        if(m_COM.ROL + m_RANGE.ROL_MAX/10 <= m_RANGE.ROL_MAX)
            m_COM.ROL += m_RANGE.ROL_MAX/10 * m_fTimeConstEnh;
        else m_COM.ROL = m_RANGE.ROL_MAX;
    }

    if(m_customKeyStatus[Qt::Key_Q] == true) {
        //qDebug() << "Q";
        if(m_COM.YAW < 0) {
            m_COM.YAW = 0;
        }

        if(m_COM.YAW + m_RANGE.YAW_MAX/10 <= m_RANGE.YAW_MAX)
            m_COM.YAW += m_RANGE.YAW_MAX/10 * m_fTimeConstEnh;
        else m_COM.YAW = m_RANGE.YAW_MAX;
    }
    if(m_customKeyStatus[Qt::Key_E] == true) {
        //qDebug() << "E";
        if(m_COM.YAW > 0) {
            m_COM.YAW = 0;
        }

        if(m_COM.YAW + m_RANGE.YAW_MIN/10 >= m_RANGE.YAW_MIN)
            m_COM.YAW += m_RANGE.YAW_MIN/10 * m_fTimeConstEnh;
        else m_COM.YAW = m_RANGE.YAW_MIN;
    }

    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_F1)] == true) {
        m_RANGE.setF1();
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_F2)] == true) {
        m_RANGE.setF2();
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_F3)] == true) {
        m_RANGE.setF3();
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_F4)] == true) {
        m_RANGE.setF4();
    }

    // Quadro moves to front
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_8)] == true) {
        m_DRIFT.PIT -= 0.05 * m_fTimeConstEnh;
        sendJSON2UDP(m_DRIFT.str_makeWiFiCommand(), false);
        qDebug() << "Drift correction: Pitch=" << m_DRIFT.str_makeWiFiCommand();
    }
    // Quadro moves backwards
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_2)] == true) {
        m_DRIFT.PIT += 0.05 * m_fTimeConstEnh;
        sendJSON2UDP(m_DRIFT.str_makeWiFiCommand(), false);
        qDebug() << "Drift correction: Pitch=" << m_DRIFT.str_makeWiFiCommand();
    }
    // Quadro moves to the left
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_4)] == true) {
        m_DRIFT.ROL -= 0.05 * m_fTimeConstEnh;
        sendJSON2UDP(m_DRIFT.str_makeWiFiCommand(), false);
        qDebug() << "Drift correction: Roll=" << m_DRIFT.str_makeWiFiCommand();
    }
    // // Quadro moves to the right
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_6)] == true) {
        m_DRIFT.ROL += 0.05 * m_fTimeConstEnh;
        sendJSON2UDP(m_DRIFT.str_makeWiFiCommand(), false);
        qDebug() << "Drift correction: Roll=" << m_DRIFT.str_makeWiFiCommand();
    }

    float fStep = 2.5;
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Up)] == true) {
        //qDebug() << "Up";
        if(m_COM.THR + fStep <= m_RANGE.THR_80P)
            m_COM.THR += fStep * m_fTimeConstEnh;
        else m_COM.THR = m_RANGE.THR_80P;
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Down)] == true) {
        //qDebug() << "Down";
        if(m_COM.THR - fStep >= m_RANGE.THR_MIN)
            m_COM.THR -= fStep * m_fTimeConstEnh;
        else m_COM.THR = m_RANGE.THR_MIN;
    }

    update();
}

void QRCWidget::sl_customKeyReleaseHandler() {
    if(!m_customKeyStatus[Qt::Key_W] && !m_customKeyStatus[Qt::Key_S]) {
        m_COM.PIT > 0 ? m_COM.PIT -= 1 * m_fTimeConstRed : m_COM.PIT += 1 * m_fTimeConstRed;
    }

    if(!m_customKeyStatus[Qt::Key_A] && !m_customKeyStatus[Qt::Key_D]) {
        m_COM.ROL > 0 ? m_COM.ROL -= 1 * m_fTimeConstRed : m_COM.ROL += 1 * m_fTimeConstRed;
    }

    if(!m_customKeyStatus[Qt::Key_Q] && !m_customKeyStatus[Qt::Key_E]) {
        m_COM.YAW > 0 ? m_COM.YAW -= 1 * m_fTimeConstRed : m_COM.YAW += 1 * m_fTimeConstRed;
    }

    if(m_bAltitudeHold) {
        if(!m_customKeyStatus[Qt::Key_H]) {
            deactAltihold2UDP();
            m_bAltitudeHold = false;
        }
    }

    update();
}

void QRCWidget::sendJSON2UDP(QString sCom, bool isCommand) {
    if(!m_pUdpSock)
        return;

    if (sCom.size() > 0) {
        //qDebug() << "WiFi: " << sCom;
        m_pUdpSock->write(sCom.toLatin1(), sCom.size() );
        if(isCommand) {
            emit si_send2Model(sCom, "command");
        }
        else {
            emit si_send2Model(sCom, "option");
        }
    }
}

void QRCWidget::sendJSON2COM(QPair<int, char*> pair, bool isCommand) {
    if(!m_pSerialPort || !m_pSerialPort->isOpen() )
        return;

    if (pair.first > 0) {
        qDebug() << "COM: " << pair.first << pair.second;
        m_pSerialPort->write(pair.second, pair.first);
    }
}

void QRCWidget::sl_sendRC2UDP() {
    sendJSON2UDP(m_COM.str_makeWiFiCommand() );
    if(m_bRadioEnabled) {
        sendJSON2COM(m_COM.cstr_makeRadioCommand() );
    }
}

void QRCWidget::sl_startTimer() {
    m_keyEventTimer.start(m_iUpdateTime);
}
