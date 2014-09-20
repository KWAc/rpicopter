#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QUdpSocket>
#include <QIODevice>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QSettings>

#include "plotdockwidget.h"
#include "attitudedockwidget.h"
#include "rcwidget.h"
#include "logdockwidget.h"
#include "piddockwidget.h"
#include "piddialog.h"
#include "gmaps.h"
#include "trimprofile.h"


#define PING_T_MS 250


class MainWindow : public QMainWindow {
Q_OBJECT
private:
    QSettings *m_pConf;

    QMenu *m_pFileM;
    QMenu *m_pOptionM;
    QAction *m_pFileMSave;
    QAction *m_pFileMLoad;
    QAction *m_pOptionMPIDConf;
    QAction *m_pOptionRadioEnabled;
    QAction *m_pOptionTrackingEnabled;
    QAction *m_pOptionPing;
    QAction *m_pOptionHost;
    QAction *m_pOptionProfiles;

    double m_fSLTime_s;
    QTime m_tSensorTime;
    QTimer m_plotTimer;

    QUdpSocket *m_pUdpSocket;
    QSerialPort m_pSerialPort;
    QSerialPortInfo m_serialPortInfo;

    bool m_bUdpSockCon;
    
    QTimer m_pingTimer;
    unsigned long m_iCurrentPingID;

    int m_iCurrentPingSent;
    int m_iCurrentPingRecv;
    
    QString m_sStatBarPing;
    QString m_sStatBarSensor;
    QString m_sStatBarRC;
    QString m_sStatBarOptions;

    QTextStream *m_pStatBarStream;
    QStatusBar *m_pStatusBar;
    
    Maps *m_pGMaps;
    QTabWidget *m_pMainTab;
    QRCWidget *m_pRCWidget;
    TrimProfile *m_pTrimProfiles;
    QProgressBar *m_pRCThrottle;

    QByteArray m_udpCurLine;

    QLogDockWidget *m_pLogger;
    QPIDDockWidget *m_pPIDConfig;
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
    void sl_loadLog();

    void sl_recvCommand_UDP();
    void sl_recvCommand_Radio();
    void sl_replotGraphs();
    void sl_UpdateSensorData(QPair<double, QVariantMap> map);

    void sl_updateStatusBar();
    void sl_updateStatusBar(QString, QString);
    
    void sl_sendPing();
    void sl_radioToggleChanged(bool);
    void sl_trackingToggleChanged(bool);

    void sl_changeTab(int);

    void sl_configPIDs();
    void sl_configPing();
    void sl_configHost();

    void sl_configProfiles();

public:
    MainWindow(QSettings *pConf, QWidget *parent = 0);
    ~MainWindow();

    QAbstractSocket *getSocket();
    void connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode = QIODevice::ReadWrite | QIODevice::Text);
};

#endif // MAINWINDOW_H
