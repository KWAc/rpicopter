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

class RC_COM {
private:
    char    m_cRadioCommand[8];
    char    m_cWiFiCommand[512];
public:
    RC_COM();

    float ROL;
    float PIT;
    float YAW;
    float THR;

    int calc_chksum(char *str);

    QPair<int, char*> cstr_makeWiFiCommand ();
    QPair<int, char*> cstr_makeRadioCommand ();
};

class DRIFT_CAL {
private:
    char    m_cWiFiCommand[512];

public:
    DRIFT_CAL();
    
    float ROL;
    float PIT;

    QPair<int, char*> cstr_makeWiFiCommand ();
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
    char   m_cWiFiCommand[512];
    QTimer m_keyEventTimer;

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

    void sendJSON2UDP(QPair<int, char*> );
    void sendJSON2COM(QPair<int, char*> );
    void initGyro2UDP();
    
private slots:
    void sl_customKeyPressHandler();
    void sl_customKeyReleaseHandler();
    void sl_sendRC2UDP(); // Emitted by timer; Calls sendJSON2UDP

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
