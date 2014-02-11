#ifndef RCWIDGET
#define RCWIDGET

#include <math.h>
#include <QtGui>
#include <QUdpSocket>

#define PI 3.14159265


struct RANGE {
    void setF1 () {
        ROL_MIN = -10;
        ROL_MAX = 10;

        PIT_MIN = -10;
        PIT_MAX = 10;

        YAW_MIN = -45;
        YAW_MAX = 45;
    }
    void setF2 () {
        ROL_MIN = -20;
        ROL_MAX = 20;

        PIT_MIN = -20;
        PIT_MAX = 20;

        YAW_MIN = -90;
        YAW_MAX = 90;
    }
    void setF3 () {
        ROL_MIN = -30;
        ROL_MAX = 30;

        PIT_MIN = -30;
        PIT_MAX = 30;

        YAW_MIN = -135;
        YAW_MAX = 135;
    }
    void setF4 () {
        ROL_MIN = -45;
        ROL_MAX = 45;

        PIT_MIN = -45;
        PIT_MAX = 45;

        YAW_MIN = -180;
        YAW_MAX = 180;
    }

    RANGE() {
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

    int ROL_MIN;
    int ROL_MAX;

    int PIT_MIN;
    int PIT_MAX;

    int YAW_MIN;
    int YAW_MAX;

    int THR_MIN;
    int THR_MAX;

    int THR_80P;
};
struct RC_COM{
    RC_COM() {
        ROL = 0; PIT = 0; YAW = 0; THR = 1100; COM_STR = "";
    }

    float ROL;
    float PIT;
    float YAW;
    float THR;

    QString COM_STR;
};
struct DRIFT_CAL{
    DRIFT_CAL() {
        ROL = 0; PIT = 0; COM_STR = "";
    }

    float ROL;
    float PIT;

    QString COM_STR;
};
struct CUSTOM_KEY {
    CUSTOM_KEY() {
        memset(KEY, 0, sizeof(KEY));
    }

    bool KEY[512];

    bool& operator[](int const& index) {
        return KEY[index];
    }

    const bool& operator[](int const& index) const {
        return KEY[index];
    }
};


class QRCWidget : public QFrame {
Q_OBJECT
public:
    QRCWidget(QUdpSocket *pSock, QWidget *parent = NULL) : QFrame(parent) {
        if(pSock)
            m_pUdpSock = pSock;

        m_fYaw = 0;

        this->setMinimumSize(480, 480);
        setFocusPolicy(Qt::StrongFocus);

        m_iUpdateTime = 13;
        m_fTimeConstRed = (float)m_iUpdateTime/150.f;
        m_fTimeConstEnh = (float)m_iUpdateTime/75.f;

        m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? this->width() : this->height();

        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyPressHandler() ) );
        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyReleaseHandler() ) );
        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_sendCommand() ) );
    }

    void setYaw(float fVal) {
        m_fYaw = fVal;
        update();
    }

    void start() {
        m_keyEventTimer.start(m_iUpdateTime);
    }

    void stop() {
        m_keyEventTimer.stop();
    }

private:
    QTimer m_keyEventTimer;

    float m_fTimeConstEnh;
    float m_fTimeConstRed;
    int m_iUpdateTime;

    QUdpSocket *m_pUdpSock;
    CUSTOM_KEY m_customKeyStatus;
    RC_COM m_COM;
    RANGE m_RANGE;
    DRIFT_CAL m_DRIFT;

    float m_fYaw;

    float m_fWidth;
    float m_fHeight;

    static int mapIndex(int key) {
        if(key >= 0x01000000) {
            return 256 + (key ^ 0x01000000);
        } else {
            return key;
        }
    }
    static QString makeCommand(int iROL, int iPIT, int iYAW, int iTHR) {
        QString com = "";
        com.append("{\"type\":\"rc\",\"r\":"); com.append(QString::number(iROL) ); com.append(",");
        com.append("\"p\":");                  com.append(QString::number(iPIT) ); com.append(",");
        com.append("\"t\":");                  com.append(QString::number(iTHR) ); com.append(",");
        com.append("\"y\":");                  com.append(QString::number(iYAW) ); com.append("}");
        return com;
    }

    static QString makeCommand(int iROL, int iPIT) {
        QString com = "";
        com.append("{\"type\":\"cmp\",\"r\":"); com.append(QString::number(iROL) ); com.append(",");
        com.append("\"p\":");                  com.append(QString::number(iPIT) ); com.append("}");
        return com;
    }

    void initGyroCalibration() {
        if(m_COM.THR > m_RANGE.THR_MIN) {
            qDebug() << "Gyrometer calibration failed, because throttle is too high";
            return;
        }

        QString com = "";
        com.append("{\"type\":\"gyr\",\"cal\":"); com.append(QString::number(true) ); com.append("}");

        this->stop();
        qDebug() << "Try to start gyrometer calibration";
        for(int i = 0; i < 16; i++) {
            m_pUdpSock->write(m_COM.COM_STR.toAscii(), m_COM.COM_STR.length() );
        }
        this->start();
    }

private slots:
    void sl_customKeyPressHandler() {
        if(m_customKeyStatus[mapIndex(Qt::Key_Backspace)] == true) {
            m_COM.PIT = 0;
            m_COM.ROL = 0;
            m_COM.YAW = 0;
            m_COM.THR = m_RANGE.THR_MIN;
        }

        if(m_customKeyStatus[Qt::Key_C] == true) {
            //qDebug() << "C";
            initGyroCalibration();
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

        if(m_customKeyStatus[mapIndex(Qt::Key_F1)] == true) {
            m_RANGE.setF1();
        }
        if(m_customKeyStatus[mapIndex(Qt::Key_F2)] == true) {
            m_RANGE.setF2();
        }
        if(m_customKeyStatus[mapIndex(Qt::Key_F3)] == true) {
            m_RANGE.setF3();
        }
        if(m_customKeyStatus[mapIndex(Qt::Key_F4)] == true) {
            m_RANGE.setF4();
        }

        QString cmp;
        // Quadro moves to front
        if(m_customKeyStatus[mapIndex(Qt::Key_8)] == true) {
            m_DRIFT.PIT -= 0.05 * m_fTimeConstEnh;
            qDebug() << "Drift correction: Pitch=" << m_DRIFT.PIT;
            cmp = makeCommand(m_DRIFT.ROL, m_DRIFT.PIT);
            sl_sendJSON(cmp);
        }
        // Quadro moves backwards
        if(m_customKeyStatus[mapIndex(Qt::Key_2)] == true) {
            m_DRIFT.PIT += 0.05 * m_fTimeConstEnh;
            qDebug() << "Drift correction: Pitch=" << m_DRIFT.PIT;
            cmp = makeCommand(m_DRIFT.ROL, m_DRIFT.PIT);
            sl_sendJSON(cmp);
        }
        // Quadro moves to the left
        if(m_customKeyStatus[mapIndex(Qt::Key_4)] == true) {
            m_DRIFT.ROL -= 0.05 * m_fTimeConstEnh;
            qDebug() << "Drift correction: Roll=" << m_DRIFT.ROL;
            cmp = makeCommand(m_DRIFT.ROL, m_DRIFT.PIT);
            sl_sendJSON(cmp);
        }
        // // Quadro moves to the right
        if(m_customKeyStatus[mapIndex(Qt::Key_6)] == true) {
            m_DRIFT.ROL += 0.05 * m_fTimeConstEnh;
            qDebug() << "Drift correction: Roll=" << m_DRIFT.ROL;
            cmp = makeCommand(m_DRIFT.ROL, m_DRIFT.PIT);
            sl_sendJSON(cmp);
        }

        float fStep = 2.5;
        if(m_customKeyStatus[mapIndex(Qt::Key_Up)] == true) {
            qDebug() << "Up";
            if(m_COM.THR + fStep <= m_RANGE.THR_80P)
                m_COM.THR += fStep * m_fTimeConstEnh;
            else m_COM.THR = m_RANGE.THR_80P;
        }
        if(m_customKeyStatus[mapIndex(Qt::Key_Down)] == true) {
            qDebug() << "Down";
            if(m_COM.THR - fStep >= m_RANGE.THR_MIN)
                m_COM.THR -= fStep * m_fTimeConstEnh;
            else m_COM.THR = m_RANGE.THR_MIN;
        }

        update();
        m_COM.COM_STR = makeCommand(m_COM.ROL, m_COM.PIT, m_COM.YAW, m_COM.THR);
    }
    void sl_customKeyReleaseHandler() {
        if(!m_customKeyStatus[Qt::Key_W] && !m_customKeyStatus[Qt::Key_S]) {
            m_COM.PIT > 0 ? m_COM.PIT -= 1 * m_fTimeConstRed : m_COM.PIT += 1 * m_fTimeConstRed;
        }

        if(!m_customKeyStatus[Qt::Key_A] && m_customKeyStatus[Qt::Key_D] == false) {
            m_COM.ROL > 0 ? m_COM.ROL -= 1 * m_fTimeConstRed : m_COM.ROL += 1 * m_fTimeConstRed;
        }

        if(!m_customKeyStatus[Qt::Key_Q] && !m_customKeyStatus[Qt::Key_E]) {
            m_COM.YAW > 0 ? m_COM.YAW -= 1 * m_fTimeConstRed : m_COM.YAW += 1 * m_fTimeConstRed;
        }

        update();
    }

    void sl_sendJSON(QString sJSON) {
        if(!m_pUdpSock)
            return;

        if (sJSON.length() > 0) {
            m_pUdpSock->write(sJSON.toAscii(), sJSON.length() );
            qDebug() << sJSON;
        }
    }
    void sl_sendCommand() {
        if(!m_pUdpSock)
            return;

        if (m_COM.COM_STR.length() > 0) {
            m_pUdpSock->write(m_COM.COM_STR.toAscii(), m_COM.COM_STR.length() );
            qDebug() << m_COM.COM_STR;
        }
    }

public slots:
    void sl_startTimer() {
        m_keyEventTimer.start(m_iUpdateTime);
    }

protected:
    QRectF rotateQRectF(QRectF rect, float fAngle) {
        float x = rect.x() * cos(fAngle * PI / 180.0) - rect.y() * sin(fAngle * PI / 180.0);
        float y = rect.x() * sin(fAngle * PI / 180.0) + rect.y() * cos(fAngle * PI / 180.0);

        return QRectF(x, y, rect.width(), rect.height() );
    }

    void paintEvent(QPaintEvent *pEvent) {
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
    void keyPressEvent ( QKeyEvent * event ) {
        if(event->isAutoRepeat() )
            return;

        int key = event->key();
        int i = mapIndex(key);
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
    void keyReleaseEvent ( QKeyEvent * event ) {
        if(event->isAutoRepeat() )
            return;

        int key = event->key();
        int i = mapIndex(key);
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
    void resizeEvent( QResizeEvent * event ) {
        m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? event->size().width() : event->size().height();
    }
};

#endif
