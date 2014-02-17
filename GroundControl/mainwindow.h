#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QUdpSocket>
#include <QIODevice>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <qjson/parser.h>

#include "qplotdockwidget.h"
#include "qattitudedockwidget.h"
#include "qrcwidget.h"
#include "qlogdockwidget.h"
#include "qpiddockwidget.h"
#include "qpiddialog.h"



class MainWindow : public QMainWindow {
Q_OBJECT
private: 
    QMenu *m_pFileM;
    QMenu *m_pOptionM;
    QAction *m_pFileMSave;
    QAction *m_pOptionMPIDConf;

    QTime m_tSensorTime;
    QTimer m_udpRecvTimer;
    QTimer m_plotTimer;

    QString m_sHostName;
    QUdpSocket *m_pUdpSocket;
    QSerialPort *m_pSerialPort;
    QSerialPortInfo m_serialPortInfo;

    bool m_bUdpSockCon;
    
    char m_cUdpRecvBuf[256];
    QJson::Parser m_JSONParser;

    QRCWidget *m_pRCWidget;
    
    QString m_sStatBarText;
    QTextStream *m_pStatBarStream;
    QStatusBar *m_pStatusBar;
    
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

private slots:
    void sl_saveLog();
    void sl_configPIDs();

    void sl_recvCommand();
    void sl_PrepareGraphs();
    void sl_replotGraphs();
    void sl_UpdateSensorData(QPair<unsigned long, QVariantMap> map);
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QAbstractSocket *getSocket() {
        return m_pUdpSocket;
    }

    void connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode = QIODevice::ReadWrite | QIODevice::Text);
};

#endif // MAINWINDOW_H
