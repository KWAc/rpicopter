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
    THR_MAX = 1700;

    THR_80P = (int)((float)(1700 - 1100) * 0.8f) + THR_MIN;
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

QString RC_COM::str_makeRadioCommand() {
    QString com = "";
    
    short thr_high  = (THR/100)%10;
    short thr_low   = THR - (1000 + ((THR/100)%10) * 100)
    short ypm = YAW < 0 ? -1 : 1;
    short yaw = YAW < 0 ? (-1 * YAW) : YAW;
    
    com.append(QChar(thr_high) );           // throttle
    com.append(QChar(thr_low) );            // throttle
    com.append(QChar((short)(PIT+127) ) );  // pitch
    com.append(QChar((short)(ROL+127) ) );  // roll
    com.append(QChar((short)(ypm+127) ) );  // yaw
    com.append(QChar((short)yaw) );         // yaw
    
    int checksum = 0;
    for(unsigned int i = 0; i < 6; i++) {
      checksum = (checksum + com.data()[i].toLatin1() ) << 1;
    }
    
    com.append(QChar(checksum) );            // checksum byte
    com.append(QChar((short)254) );         // end byte
    return com;
}

DRIFT_CAL::DRIFT_CAL() {
    ROL = 0; PIT = 0;
}

QString DRIFT_CAL::str_makeCommand() {
    QString com = "";
    com.append("{\"type\":\"cmp\",\"r\":"); com.append(QString::number(ROL, 'f', 2) ); com.append(",");
    com.append("\"p\":");                   com.append(QString::number(PIT, 'f', 2) ); com.append("}");
    return com;
}

CUSTOM_KEY::CUSTOM_KEY() {
    memset(KEY, 0, sizeof(KEY));
}


QRCWidget::QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort, QWidget *parent) : QFrame(parent) {
    assert(pSock != NULL);
    m_pUdpSock = pSock;

    m_pSerialPort = pSerialPort;
    m_fYaw = 0;

    this->setMinimumSize(480, 480);
    setFocusPolicy(Qt::StrongFocus);

    m_iUpdateTime = 13;
    m_fTimeConstRed = (float)m_iUpdateTime/150.f;
    m_fTimeConstEnh = (float)m_iUpdateTime/75.f;

    m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? this->width() : this->height();

    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyPressHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyReleaseHandler() ) );
    connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_sendRC2UDP() ) );
    connect(&m_comPortTimer,  SIGNAL(timeout() ), this, SLOT(sl_sendRC2COM() ) );
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

void QRCWidget::sl_customKeyPressHandler() {
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Backspace)] == true) {
        m_COM.PIT = 0;
        m_COM.ROL = 0;
        m_COM.YAW = 0;
        m_COM.THR = m_RANGE.THR_MIN;

        m_DRIFT.ROL = 0; 
        m_DRIFT.PIT = 0;
        sendJSON2UDP(m_DRIFT.str_makeCommand());
    }

    if(m_customKeyStatus[Qt::Key_C] == true) {
        //qDebug() << "C";
        initGyro2UDP();
        m_customKeyStatus[Qt::Key_C] = false;
    }

    if(m_customKeyStatus[Qt::Key_W] == true) {
        //qDebug() << "W";
        if(m_COM.PIT + m_RANGE.PIT_MIN/10 >= m_RANGE.PIT_MIN)
            m_COM.PIT += m_RANGE.PIT_MIN/10 * m_fTimeConstEnh;
        else m_COM.PIT = m_RANGE.PIT_MIN;
    }
    if(m_customKeyStatus[Qt::Key_S] == true) {
        //qDebug() << "S";
        if(m_COM.PIT + m_RANGE.PIT_MAX/10 <= m_RANGE.PIT_MAX)
            m_COM.PIT += m_RANGE.PIT_MAX/10 * m_fTimeConstEnh;
        else m_COM.PIT = m_RANGE.PIT_MAX;
    }

    if(m_customKeyStatus[Qt::Key_A] == true) {
        //qDebug() << "A";
        if(m_COM.ROL + m_RANGE.ROL_MIN/10 >= m_RANGE.ROL_MIN)
            m_COM.ROL += m_RANGE.ROL_MIN/10 * m_fTimeConstEnh;
        else m_COM.ROL = m_RANGE.ROL_MIN;
    }
    if(m_customKeyStatus[Qt::Key_D] == true) {
        //qDebug() << "D";
        if(m_COM.ROL + m_RANGE.ROL_MAX/10 <= m_RANGE.ROL_MAX)
            m_COM.ROL += m_RANGE.ROL_MAX/10 * m_fTimeConstEnh;
        else m_COM.ROL = m_RANGE.ROL_MAX;
    }

    if(m_customKeyStatus[Qt::Key_Q] == true) {
        //qDebug() << "Q";
        if(m_COM.YAW + m_RANGE.YAW_MAX/10 <= m_RANGE.YAW_MAX)
            m_COM.YAW += m_RANGE.YAW_MAX/10 * m_fTimeConstEnh;
        else m_COM.YAW = m_RANGE.YAW_MAX;
    }
    if(m_customKeyStatus[Qt::Key_E] == true) {
        //qDebug() << "E";
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
        QString cor = m_DRIFT.str_makeCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Pitch=" << cor;
    }
    // Quadro moves backwards
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_2)] == true) {
        m_DRIFT.PIT += 0.05 * m_fTimeConstEnh;
        QString cor = m_DRIFT.str_makeCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Pitch=" << cor;
    }
    // Quadro moves to the left
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_4)] == true) {
        m_DRIFT.ROL -= 0.05 * m_fTimeConstEnh;
        QString cor = m_DRIFT.str_makeCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Roll=" << cor;
    }
    // // Quadro moves to the right
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_6)] == true) {
        m_DRIFT.ROL += 0.05 * m_fTimeConstEnh;
        QString cor = m_DRIFT.str_makeCommand();
        sendJSON2UDP(cor);
        qDebug() << "Drift correction: Roll=" << cor;
    }

    float fStep = 2.5;
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Up)] == true) {
        qDebug() << "Up";
        if(m_COM.THR + fStep <= m_RANGE.THR_80P)
            m_COM.THR += fStep * m_fTimeConstEnh;
        else m_COM.THR = m_RANGE.THR_80P;
    }
    if(m_customKeyStatus[CUSTOM_KEY::mapCustomKeyIndex(Qt::Key_Down)] == true) {
        qDebug() << "Down";
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

    update();
}

void QRCWidget::sendJSON2UDP(const QString &sJSON) {
    if(!m_pUdpSock)
        return;

    if (sJSON.length() > 0) {
        m_pUdpSock->write(sJSON.toLocal8Bit(), sJSON.length() );
        qDebug() << "WiFi: " << sJSON;
    }
}

void QRCWidget::sendJSON2COM(const QString &sCommand) {
    if(!m_pSerialPort)
        return;
        
    if (sCommand.length() > 0) {
        m_pSerialPort->write(sCommand.toLocal8Bit(), sCommand.length() );
        qDebug() << "Radio: " << sCommand;
    }
}

void QRCWidget::sl_sendRC2UDP() {
    sendJSON2UDP(m_COM.str_makeWiFiCommand() );
    sendJSON2COM(m_COM.str_makeRadioCommand() );
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
        qDebug() << "Backspace";
        break;

        case Qt::Key_A:
        m_customKeyStatus[i] = true;
        qDebug() << "A";
        break;
        case Qt::Key_D:
        m_customKeyStatus[i] = true;
        qDebug() << "D";
        break;

        case Qt::Key_W:
        m_customKeyStatus[i] = true;
        qDebug() << "W";
        break;
        case Qt::Key_S:
        m_customKeyStatus[i] = true;
        qDebug() << "S";
        break;

        case Qt::Key_Q:
        m_customKeyStatus[i] = true;
        qDebug() << "Q";
        break;
        case Qt::Key_E:
        m_customKeyStatus[i] = true;
        qDebug() << "E";
        break;

        case Qt::Key_C:
        m_customKeyStatus[i] = true;
        qDebug() << "C";
        break;

        case Qt::Key_Up:
        m_customKeyStatus[i] = true;
        qDebug() << "Up";
        break;
        case Qt::Key_Down:
        m_customKeyStatus[i] = true;
        qDebug() << "Down";
        break;

        case Qt::Key_F1:
        m_customKeyStatus[i] = true;
        qDebug() << "F1";
        break;
        case Qt::Key_F2:
        m_customKeyStatus[i] = true;
        qDebug() << "F2";
        break;
        case Qt::Key_F3:
        m_customKeyStatus[i] = true;
        qDebug() << "F3";
        break;
        case Qt::Key_F4:
        m_customKeyStatus[i] = true;
        qDebug() << "F4";
        break;

        case Qt::Key_8:
        m_customKeyStatus[i] = true;
        qDebug() << "F8";
        break;
        case Qt::Key_2:
        m_customKeyStatus[i] = true;
        qDebug() << "F2";
        break;
        case Qt::Key_4:
        m_customKeyStatus[i] = true;
        qDebug() << "F4";
        break;
        case Qt::Key_6:
        m_customKeyStatus[i] = true;
        qDebug() << "F6";
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
