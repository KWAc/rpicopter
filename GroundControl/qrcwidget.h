#ifndef RCWIDGET
#define RCWIDGET

#include <math.h>
#include <QtWidgets>
#include <QUdpSocket>
#include <QSerialPort>

#include "qabsframe.h"

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

    QString str_makeWiFiCommand();
    QPair<int, char*> cstr_makeRadioCommand ();
};

class DRIFT_CAL {
private:
    char    m_cWiFiCommand[512];

public:
    DRIFT_CAL();
    
    float ROL;
    float PIT;

    QString str_makeWiFiCommand();
    QPair<int, char*> cstr_makeWiFiCommand ();
};


class QRCWidget : public QAbsFrame {
Q_OBJECT
private:
    char   m_cWiFiCommand[512];
    QTimer m_keyEventTimer;

    float m_fTimeConstEnh;
    float m_fTimeConstRed;
    int   m_iUpdateTime;

    QUdpSocket  *m_pUdpSock;
    QSerialPort *m_pSerialPort;
    
    DRIFT_CAL m_DRIFT;

    void sendJSON2UDP(QString);
    void sendJSON2COM(QPair<int, char*> );
    void initGyro2UDP();
    void activAltihold2UDP();
    void deactAltihold2UDP();

    bool m_bRadioEnabled;
    bool m_bAltitudeHold;
    
private slots:
    void sl_customKeyPressHandler();
    void sl_customKeyReleaseHandler();
    void sl_sendRC2UDP(); // Emitted by timer; Calls sendJSON2UDP
    
public slots:
    void sl_startTimer();
    void sl_setRadioEnabled(bool state);

signals:
    void si_throttleChanged(int);
    void si_send2Model(QString);

public:
    QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort = NULL, QWidget *parent = NULL);

    RANGE  m_RANGE;
    RC_COM m_COM;

    void start();
    void stop();
};

#endif
