#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QUdpSocket>
#include <QIODevice>
#include <QSerialPort>
#include <QSerialPortInfo>

#include "qplotdockwidget.h"
#include "qattitudedockwidget.h"
#include "qrcwidget.h"
#include "qlogdockwidget.h"
#include "qpiddockwidget.h"
#include "qpiddialog.h"


#define PING_T_MS 250


class MainWindow : public QMainWindow {
Q_OBJECT
private: 
    QMenu *m_pFileM;
    QMenu *m_pOptionM;
    QAction *m_pFileMSave;
    QAction *m_pOptionMPIDConf;
    QAction *m_pOptionRadioEnabled;

    QTime m_tSensorTime;
    QTimer m_plotTimer;

    QByteArray m_udpCurLine;

    QString m_sHostName;
    QUdpSocket *m_pUdpSocket;
    QSerialPort m_pSerialPort;
    QSerialPortInfo m_serialPortInfo;

    bool m_bUdpSockCon;
    char m_cUdpRecvBuf[256];
    
    QTimer m_pingTimer;
    int m_iCurrentPingID;
    int m_iCurrentPingSent;
    int m_iCurrentPingRecv;

    QRCWidget *m_pRCWidget;
    
    QString m_sStatBarSensor;
    QString m_sStatBarRC;
    QString m_sStatBarOptions;

    QTextStream *m_pStatBarStream;
    QStatusBar *m_pStatusBar;
    
    QGroupBox *m_pMainWidget;
    QProgressBar *m_pThrottleBar;
    QHBoxLayout *m_pMainLayout;

    QPIDDockWidget *m_pPIDConfig;
    QLogDockWidget *m_pLogger;
    QAttitudeDockWidget *m_pAttitude;
    QPlotDockWidget *m_pBattery;
    QPlotDockWidget *m_pBarometer;
    QPlotDockWidget *m_pNetworkDelay;
    
    QVector<double> m_vBarometer_s;
    QVector<double> m_vAirPressure;
    QVector<double> m_vAltitude_m;
    QVector<double> m_vTemperature_C;
    QVector<double> m_vClimbrate_ms;
    QVector<double> m_vTemperature_samples;

    QVector<double> m_vBattery_s;
    QVector<double> m_vBattery_V;
    QVector<double> m_vBattery_A;

    QVector<double> m_vNetwork_s;
    QVector<double> m_vLatency_ms;

    QPIDConfig *m_pPIDConfigDial;

    bool searchSerialRadio();
    void connectWidgets();
    void prepareWidgets();
    void prepareMenus();
    void prepareGraphs();

private slots:
    void sl_saveLog();
    void sl_configPIDs();

    void sl_recvCommand();
    void sl_replotGraphs();
    void sl_UpdateSensorData(QPair<double, QVariantMap> map);

    void sl_updateStatusBar();
    void sl_updateStatusBar(QString, QString);
    
    void sl_sendPing();

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QAbstractSocket *getSocket();
    void connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode = QIODevice::ReadWrite | QIODevice::Text);
};

#endif // MAINWINDOW_H
