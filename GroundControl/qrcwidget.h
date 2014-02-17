#ifndef RCWIDGET
#define RCWIDGET

#include <math.h>
#include <QtWidgets>
#include <QUdpSocket>
#include <QSerialPort>


#define PI 3.14159265


struct RANGE {
    void setF1 ();
    void setF2 ();
    void setF3 ();
    void setF4 ();

    RANGE();

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

struct RC_COM {
    RC_COM();

    float ROL;
    float PIT;
    float YAW;
    float THR;

    int calc_chksum(char *str);

    QString str_makeWiFiCommand ();
    QString str_makeRadioCommand();
};

struct DRIFT_CAL {
    DRIFT_CAL();
    
    float ROL;
    float PIT;

    QString str_makeCommand();
};

struct CUSTOM_KEY {
    CUSTOM_KEY();

    bool KEY[512];

    bool& operator[](int const& index) {
        return KEY[index];
    }
    const bool& operator[](int const& index) const {
        return KEY[index];
    }
    
    static int mapCustomKeyIndex(const int key);
};


class QRCWidget : public QFrame {
Q_OBJECT

private:
    QTimer m_keyEventTimer;
    QTimer m_comPortTimer;
    int m_iRadioCounter;
    QString m_sRadioCom;

    float m_fTimeConstEnh;
    float m_fTimeConstRed;
    int   m_iUpdateTime;

    QUdpSocket  *m_pUdpSock;
    QSerialPort *m_pSerialPort;
    
    CUSTOM_KEY m_customKeyStatus;
    RANGE m_RANGE;
    RC_COM m_COM;
    DRIFT_CAL m_DRIFT;

    float m_fYaw;
    float m_fWidth;
    float m_fHeight;

    void sendJSON2UDP(const QString &sJSON);
    void sendJSON2COM(const QString &sCommand);
    void initGyro2UDP();
    
private slots:
    void sl_customKeyPressHandler();
    void sl_customKeyReleaseHandler();
    void sl_sendRC2UDP(); // Emitted by timer; Calls sendJSON2UDP
    void sl_sendRC2COM(); // Emitted by timer; Calls sendJSON2UDP

protected:
    void paintEvent(QPaintEvent *pEvent);
    void keyPressEvent ( QKeyEvent * event );
    void keyReleaseEvent ( QKeyEvent * event );
    void resizeEvent( QResizeEvent * event );
    
public slots:
    void sl_startTimer();

public:
    QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort = NULL, QWidget *parent = NULL);

    void setYaw(float fVal);
    void start();
    void stop();
};

#endif
