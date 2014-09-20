#ifndef RCWIDGET
#define RCWIDGET

#include <math.h>
#include <QtWidgets>
#include <QUdpSocket>
#include <QSerialPort>

#include "absframe.h"
#include "container.h"


class QRCWidget : public QAbsFrame {
Q_OBJECT
private:
    char   m_cWiFiCommand[512];
    QTimer m_keyEventTimer;
    QTimer m_caliEventTimer;

    QTimer m_trimTimer;
    QTime  m_caliTime;

    bool   m_bThrottleHold;
    QTime  m_tThrottleEnh;
    QTime  m_tThrottleRed;

    float m_fTimeConstEnh;
    float m_fTimeConstRed;
    int   m_iUpdateTime;

    QUdpSocket  *m_pUdpSock;
    QSerialPort *m_pSerialPort;

    void sendJSON2UDP(QString, bool isCommand = true);
    void sendJSON2COM(QString, bool isCommand = true);
    void sendJSON2COM(QPair<int, char*>);
    void initGyro2UDP();
    void activAltihold2UDP();
    void deactAltihold2UDP();

    bool m_bRadioEnabled;
    bool m_bAltitudeHold;
    
private slots:
    void sl_customKeyPressHandler();
    void sl_customKeyReleaseHandler();

    void sl_sendTrim2UDP();
    
    void sl_sendRC2UDP(); // Emitted by timer; Calls sendJSON2UDP
    
public slots:
    void sl_setRadioEnabled(bool state);
    void sl_setAttitudeCorr(float fRoll, float fPitch);
    
signals:
    void si_send2Model(QString, QString);
    void si_attitudeCorrChanged(float roll, float pitch);

public:
    QRCWidget(QUdpSocket *pSock, QSerialPort *pSerialPort = NULL, QWidget *parent = NULL);

    RANGE     m_RANGE;
    RC_COM    m_COM;
    DRIFT_CAL m_DRIFT;

    void start();
    void stop();
};

#endif
