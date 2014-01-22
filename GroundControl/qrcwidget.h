#ifndef RCWIDGET
#define RCWIDGET

#include <QtGui>
#include <QUdpSocket>


struct RANGE {
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

        this->setMinimumSize(480, 480);
        setFocusPolicy(Qt::StrongFocus);

        m_iUpdateTime = 13;
        m_fTimeConstRed = (float)m_iUpdateTime/150.f;
        m_fTimeConstEnh = (float)m_iUpdateTime/75.f;

        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyPressHandler() ) );
        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_customKeyReleaseHandler() ) );
        connect(&m_keyEventTimer, SIGNAL(timeout() ), this, SLOT(sl_sendCommand() ) );
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

private slots:
    void sl_customKeyPressHandler() {
        update();
        if(m_customKeyStatus[Qt::Key_R] == true) {
            m_COM.PIT = 0;
            m_COM.ROL = 0;
            m_COM.YAW = 0;
            m_COM.THR = m_RANGE.THR_MIN;
        }

        if(m_customKeyStatus[Qt::Key_W] == true) {
            if(m_COM.PIT + m_RANGE.PIT_MIN/10 >= m_RANGE.PIT_MIN)
                m_COM.PIT += m_RANGE.PIT_MIN/10 * m_fTimeConstEnh;
            else m_COM.PIT = m_RANGE.PIT_MIN;
        }
        if(m_customKeyStatus[Qt::Key_S] == true) {
            if(m_COM.PIT + m_RANGE.PIT_MAX/10 <= m_RANGE.PIT_MAX)
                m_COM.PIT += m_RANGE.PIT_MAX/10 * m_fTimeConstEnh;
            else m_COM.PIT = m_RANGE.PIT_MAX;
        }

        if(m_customKeyStatus[Qt::Key_A] == true) {
            if(m_COM.ROL + m_RANGE.ROL_MIN/10 >= m_RANGE.ROL_MIN)
                m_COM.ROL += m_RANGE.ROL_MIN/10 * m_fTimeConstEnh;
            else m_COM.ROL = m_RANGE.ROL_MIN;
        }
        if(m_customKeyStatus[Qt::Key_D] == true) {
            if(m_COM.ROL + m_RANGE.ROL_MAX/10 <= m_RANGE.ROL_MAX)
                m_COM.ROL += m_RANGE.ROL_MAX/10 * m_fTimeConstEnh;
            else m_COM.ROL = m_RANGE.ROL_MAX;
        }

        if(m_customKeyStatus[Qt::Key_Q] == true) {
            if(m_COM.YAW + m_RANGE.YAW_MAX/10 <= m_RANGE.YAW_MAX)
                m_COM.YAW += m_RANGE.YAW_MAX/10 * m_fTimeConstEnh;
            else m_COM.YAW = m_RANGE.YAW_MAX;
        }
        if(m_customKeyStatus[Qt::Key_E] == true) {
            if(m_COM.YAW + m_RANGE.YAW_MIN/10 >= m_RANGE.YAW_MIN)
                m_COM.YAW += m_RANGE.YAW_MIN/10 * m_fTimeConstEnh;
            else m_COM.YAW = m_RANGE.YAW_MIN;
        }

        float fStep = 2.5;
        if(m_customKeyStatus[mapIndex(Qt::Key_Up)] == true) {
            if(m_COM.THR + fStep <= m_RANGE.THR_80P)
                m_COM.THR += fStep * m_fTimeConstEnh;
            else m_COM.THR = m_RANGE.THR_80P;
        }
        if(m_customKeyStatus[mapIndex(Qt::Key_Down)] == true) {
            if(m_COM.THR - fStep >= m_RANGE.THR_MIN)
                m_COM.THR -= fStep * m_fTimeConstEnh;
            else m_COM.THR = m_RANGE.THR_MIN;
        }

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
    }
    void sl_sendCommand() {
        if(!m_pUdpSock)
            return;

        if (m_COM.COM_STR.length() > 0) {
            m_pUdpSock->write(m_COM.COM_STR.toAscii(), m_COM.COM_STR.length() );
            //qDebug() << m_COM.COM_STR;
        }
    }

public slots:
    void sl_startTimer() {
        m_keyEventTimer.start(m_iUpdateTime);
    }

protected:
    void paintEvent(QPaintEvent *pEvent) {
        Q_UNUSED(pEvent);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.translate(width()/2, height()/2);

        // Center black
        float diameter = width() / 5;
        float center = width() / 7;
        painter.drawEllipse(QRectF(-center/2.0, -diameter/2.0, center, diameter));

        // Rotor bottom right
        painter.resetTransform();
        painter.translate(width()/2, height()/2);
        painter.translate(width()/4, height()/4);
        painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        // Rotor top left
        painter.resetTransform();
        painter.translate(width()/2, height()/2);
        painter.translate(-width()/4, -height()/4);
        painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        // Rotor bottom left
        painter.resetTransform();
        painter.translate(width()/2, height()/2);
        painter.translate(-width()/4, height()/4);
        painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        // Rotor top right
        painter.resetTransform();
        painter.translate(width()/2, height()/2);
        painter.translate(width()/4, -height()/4);
        painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

        QPen redHighlightP(QColor(255,0,0, 45));
        QBrush redHighlightB(QColor(255,0,0, 25));
        redHighlightP.setWidth(10);
        painter.setPen(redHighlightP);
        painter.setBrush(redHighlightB);
        // bottom in red
        if(m_customKeyStatus[Qt::Key_W] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
        // top in red
        if(m_customKeyStatus[Qt::Key_S] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // right in red
        if(m_customKeyStatus[Qt::Key_A] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // left in red
        if(m_customKeyStatus[Qt::Key_D] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }


        // bottom left and top right in red
        if(m_customKeyStatus[Qt::Key_Q] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // bottom right and top left in red
        if(m_customKeyStatus[Qt::Key_E] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(width()/4, height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top left
            painter.resetTransform();
            painter.translate(width()/2, height()/2);
            painter.translate(-width()/4, -height()/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
    }

    void keyPressEvent ( QKeyEvent * event ) {
        if(event->isAutoRepeat() )
            return;

        int key = event->key();
        int i = mapIndex(key);
        switch (key) {
            case Qt::Key_R:
            m_customKeyStatus[i] = true;
            break;

            case Qt::Key_A:
            m_customKeyStatus[i] = true;
            break;
            case Qt::Key_D:
            m_customKeyStatus[i] = true;
            break;

            case Qt::Key_W:
            m_customKeyStatus[i] = true;
            break;
            case Qt::Key_S:
            m_customKeyStatus[i] = true;
            break;

            case Qt::Key_Q:
            m_customKeyStatus[i] = true;
            break;
            case Qt::Key_E:
            m_customKeyStatus[i] = true;
            break;

            case Qt::Key_Up:
            m_customKeyStatus[i] = true;
            break;
            case Qt::Key_Down:
            m_customKeyStatus[i] = true;
            break;
        }
    }
    void keyReleaseEvent ( QKeyEvent * event ) {
        if(event->isAutoRepeat() )
            return;

        int key = event->key();
        int i = mapIndex(key);
        switch (key) {
            case Qt::Key_R:
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

            case Qt::Key_Up:
            m_customKeyStatus[i] = false;
            break;
            case Qt::Key_Down:
            m_customKeyStatus[i] = false;
            break;
        }
    }
};

#endif
