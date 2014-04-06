#include "qrcwidget.h"
#include <cassert>


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

    THR_80P = (int)((float)(THR_MAX - THR_MIN) * 0.8f) + THR_MIN;
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

QPair<int, char*> RC_COM::cstr_makeWiFiCommand() {
    QString com = str_makeWiFiCommand();

    memset(m_cWiFiCommand, 0, sizeof(m_cWiFiCommand) );
    for(unsigned int i = 0; i < static_cast<unsigned int>(com.size() ) && i < sizeof(m_cWiFiCommand); i++) {
        m_cWiFiCommand[i] = com.at(i).toLatin1();
    }

    return QPair<int, char*> (com.size(), m_cWiFiCommand);
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

CUSTOM_KEY::CUSTOM_KEY() {
    memset(KEY, 0, sizeof(KEY));
}

void QRCWidget::sl_setRadioEnabled(bool state) {
    m_bRadioEnabled = state;
}

QRCWidget::QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort, QWidget *parent) : QFrame(parent) {
    assert(pSock != NULL);
    m_pUdpSock = pSock;

    m_pSerialPort = pSerialPort;
    m_fYaw = 0;

    this->setMinimumSize(480, 480);
    setFocusPolicy(Qt::StrongFocus);

    m_bRadioEnabled = true;
    m_bAltitudeHold = false;

    m_iUpdateTime = 8;
    m_fTimeConstRed = (float)m_iUpdateTime/150.f;
    m_fTimeConstEnh = (float)m_iUpdateTime/75.f;

    m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? this->width() : this->height();

    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyPressHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyReleaseHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_sendRC2UDP() ) );
}

void QRCWidget::setYaw(float fVal) {
    m_fYaw = fVal;
    update();
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
    for(int i = 0; i < 16; i++) {
        sendJSON2UDP(com);
    }
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
    for(int i = 0; i < 16; i++) {
        sendJSON2UDP(com);
    }
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
    for(int i = 0; i < 16; i++) {
        sendJSON2UDP(com);
    }
}

void QRCWidget::sl_customKeyPressHandler() {
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Backspace)] == true) {
        m_COM.PIT = 0;
        m_COM.ROL = 0;
        m_COM.YAW = 0;
        m_COM.THR = m_RANGE.THR_MIN;

        m_DRIFT.ROL = 0; 
        m_DRIFT.PIT = 0;

        sendJSON2UDP(m_DRIFT.cstr_makeWiFiCommand() );
        emit si_throttleChanged(m_COM.THR);
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
        QPair<int, char*> cor = m_DRIFT.cstr_makeWiFiCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Pitch=" << cor.second;
    }
    // Quadro moves backwards
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_2)] == true) {
        m_DRIFT.PIT += 0.05 * m_fTimeConstEnh;
        QPair<int, char*> cor = m_DRIFT.cstr_makeWiFiCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Pitch=" << cor.second;
    }
    // Quadro moves to the left
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_4)] == true) {
        m_DRIFT.ROL -= 0.05 * m_fTimeConstEnh;
        QPair<int, char*> cor = m_DRIFT.cstr_makeWiFiCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Roll=" << cor.second;
    }
    // // Quadro moves to the right
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_6)] == true) {
        m_DRIFT.ROL += 0.05 * m_fTimeConstEnh;
        QPair<int, char*> cor = m_DRIFT.cstr_makeWiFiCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Roll=" << cor.second;
    }

    float fStep = 2.5;
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Up)] == true) {
        //qDebug() << "Up";
        if(m_COM.THR + fStep <= m_RANGE.THR_80P)
            m_COM.THR += fStep * m_fTimeConstEnh;
        else m_COM.THR = m_RANGE.THR_80P;

        emit si_throttleChanged(m_COM.THR);
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Down)] == true) {
        //qDebug() << "Down";
        if(m_COM.THR - fStep >= m_RANGE.THR_MIN)
            m_COM.THR -= fStep * m_fTimeConstEnh;
        else m_COM.THR = m_RANGE.THR_MIN;

        emit si_throttleChanged(m_COM.THR);
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

void QRCWidget::sendJSON2UDP(QPair<int, char*> pair) {
    if(!m_pUdpSock)
        return;

    if (pair.first > 0) {
        qDebug() << "WiFi: " << pair.first << pair.second;
        m_pUdpSock->write(pair.second, pair.first);
    }
}

void QRCWidget::sendJSON2UDP(QString sCom) {
    if(!m_pUdpSock)
        return;

    if (sCom.size() > 0) {
        qDebug() << "WiFi: " << sCom;
        m_pUdpSock->write(sCom.toLatin1(), sCom.size() );
    }
}

void QRCWidget::sendJSON2COM(QPair<int, char*> pair) {
    if(!m_pSerialPort || !m_pSerialPort->isOpen() )
        return;

    if (pair.first > 0) {
        //qDebug() << "COM: " << pair.first << pair.second;
        m_pSerialPort->write(pair.second, pair.first);
    }
}

void QRCWidget::sl_sendRC2UDP() {
    sendJSON2UDP(m_COM.cstr_makeWiFiCommand() );
    if(m_bRadioEnabled) {
        sendJSON2COM(m_COM.cstr_makeRadioCommand() );
    }
}

void QRCWidget::sl_startTimer() {
    m_keyEventTimer.start(m_iUpdateTime);
}


void QRCWidget::paintEvent(QPaintEvent *pEvent) {
    Q_UNUSED(pEvent);
    QPainter painter(this);

    QPen redPM(Qt::red);

    QPen grayPM(Qt::darkGray);
    grayPM.setWidthF(0);

    QPen blackPM(Qt::black);
    QPen blackPF(Qt::black);
    blackPM.setWidth(2);
    QBrush blackB(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if(!this->isEnabled() ) {
        float centerW   = m_fWidth / 2;
        float centerH   = m_fWidth / 2;

        QFont serifFont("Times", 16, QFont::Bold);
        QRectF rect(-centerW/2.0, -centerH, centerW, centerH);

        painter.translate(m_fWidth/2, m_fHeight/2);
        painter.setFont(serifFont);
        painter.setPen(blackPM);
        painter.drawText(rect, Qt::AlignCenter, tr("No connection.") );
        painter.resetTransform();

        redPM = QPen(Qt::gray);
        grayPM = QPen(Qt::gray);
        blackPF = QPen(Qt::gray);
        blackPM = QPen(Qt::gray);
        blackB = QBrush(Qt::gray);

    }
    // Center plate black
    painter.setBrush(blackB);
    painter.setPen(blackPM);
    float diameter  = m_fWidth / 4;
    float centerW   = m_fWidth / 8;
    float centerH   = m_fWidth / 5.5;

    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.drawEllipse(QRectF(-centerW/2.0, -centerH/2.0, centerW, centerH));

    // Rotor bottom right
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(m_fWidth/4, m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor top left
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(-m_fWidth/4, -m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor bottom left
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(-m_fWidth/4, m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor top right
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(m_fWidth/4, -m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));


    // Now draw the other frame parts
    QLineF bl_fr(-m_fWidth/4, m_fHeight/4, m_fWidth/4, -m_fHeight/4);
    QLineF br_fl(m_fWidth/4, m_fHeight/4, -m_fWidth/4, -m_fHeight/4);

    blackPF.setWidth(m_fWidth < m_fHeight ? m_fWidth/48 : m_fHeight/48);
    painter.setPen(blackPF);

    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);

    painter.drawLine(bl_fr);
    painter.drawLine(br_fl);

    if(this->isEnabled() ) {
        // Highlighting active motors ..
        QPen redHighlightP(QColor(255,0,0, 45));
        redHighlightP.setWidth(16);
        QBrush redHighlightB(QColor(255,0,0, 25));
        painter.setPen(redHighlightP);
        painter.setBrush(redHighlightB);
        // bottom in red
        if(m_customKeyStatus[Qt::Key_W] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
        // top in red
        if(m_customKeyStatus[Qt::Key_S] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // right in red
        if(m_customKeyStatus[Qt::Key_A] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // left in red
        if(m_customKeyStatus[Qt::Key_D] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }


        // bottom left and top right in red
        if(m_customKeyStatus[Qt::Key_Q] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // bottom right and top left in red
        if(m_customKeyStatus[Qt::Key_E] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
    }

    // Draw Coordinate system
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(grayPM);
    painter.setBrush(Qt::NoBrush);

    float fWAx = 0.1 * m_fWidth;
    float fHAx = 0.1 * m_fHeight;

    float fWC = 0.025 * m_fWidth;
    float fHC = 0.025 * m_fHeight;

    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);

    painter.drawEllipse(QRectF(-(m_fWidth/2-fWC/2), -(m_fHeight/2-fHC/2), m_fWidth-fWC, m_fHeight-fHC));
    QLineF lineC90 ( m_fWidth/2 -fWC/2 - 5, 0, m_fWidth/2 -fWC/2 + 5, 0 );
    QLineF lineC270( -(m_fWidth/2 -fWC/2 - 5), 0, -(m_fWidth/2 -fWC/2 + 5), 0 );
    QLineF lineC180( 0, m_fHeight/2 -fHC/2 - 5, 0, m_fHeight/2 -fHC/2 + 5 );
    QLineF lineC360( 0, -(m_fHeight/2 -fHC/2 - 5), 0, -(m_fHeight/2 -fHC/2 + 5) );

    painter.drawLine(lineC90);
    painter.drawLine(lineC180);
    painter.drawLine(lineC270);
    painter.drawLine(lineC360);

    // X and Y axis
    QLineF lineAxisY( 0, -m_fHeight/2 + fHAx, 0, 0 );
    QLineF lineAxisX( 0, 0, m_fWidth/2 - fWAx, 0 );

    painter.rotate(m_fYaw);
    painter.drawLine(lineAxisY);
    painter.setPen(redPM);
    painter.drawLine(lineAxisX);
}
void QRCWidget::keyPressEvent ( QKeyEvent * event ) {
    if(event->isAutoRepeat() )
        return;

    int key = event->key();
    int i = CUSTOM_KEY::mapCustomKeyIndex(key);
    switch (key) {
        case Qt::Key_Backspace:
        m_customKeyStatus[i] = true;
        //qDebug() << "Backspace";
        break;

        case Qt::Key_A:
        m_customKeyStatus[i] = true;
        //qDebug() << "A";
        break;
        case Qt::Key_D:
        m_customKeyStatus[i] = true;
        //qDebug() << "D";
        break;

        case Qt::Key_W:
        m_customKeyStatus[i] = true;
        //qDebug() << "W";
        break;
        case Qt::Key_S:
        m_customKeyStatus[i] = true;
        //qDebug() << "S";
        break;

        case Qt::Key_Q:
        m_customKeyStatus[i] = true;
        //qDebug() << "Q";
        break;
        case Qt::Key_E:
        m_customKeyStatus[i] = true;
        //qDebug() << "E";
        break;

        case Qt::Key_C:
        m_customKeyStatus[i] = true;
        //qDebug() << "C";
        break;

        case Qt::Key_H:
        m_customKeyStatus[i] = true;
        //qDebug() << "H";
        break;

        case Qt::Key_Up:
        m_customKeyStatus[i] = true;
        //qDebug() << "Up";
        break;
        case Qt::Key_Down:
        m_customKeyStatus[i] = true;
        //qDebug() << "Down";
        break;

        case Qt::Key_F1:
        m_customKeyStatus[i] = true;
        //qDebug() << "F1";
        break;
        case Qt::Key_F2:
        m_customKeyStatus[i] = true;
        //qDebug() << "F2";
        break;
        case Qt::Key_F3:
        m_customKeyStatus[i] = true;
        //qDebug() << "F3";
        break;
        case Qt::Key_F4:
        m_customKeyStatus[i] = true;
        //qDebug() << "F4";
        break;

        case Qt::Key_8:
        m_customKeyStatus[i] = true;
        //qDebug() << "F8";
        break;
        case Qt::Key_2:
        m_customKeyStatus[i] = true;
        //qDebug() << "F2";
        break;
        case Qt::Key_4:
        m_customKeyStatus[i] = true;
        //qDebug() << "F4";
        break;
        case Qt::Key_6:
        m_customKeyStatus[i] = true;
        //qDebug() << "F6";
        break;
    }
}
void QRCWidget::keyReleaseEvent ( QKeyEvent * event ) {
    if(event->isAutoRepeat() )
        return;

    int key = event->key();
    int i = CUSTOM_KEY::mapCustomKeyIndex(key);
    switch (key) {
        case Qt::Key_Backspace:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_A:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_D:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_W:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_S:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_Q:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_E:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_C:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_H:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_Up:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_Down:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_F1:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F2:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F3:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F4:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_8:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_2:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_4:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_6:
        m_customKeyStatus[i] = false;
        break;
    }
}
void QRCWidget::resizeEvent( QResizeEvent * event ) {
    m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? event->size().width() : event->size().height();
}
