#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QUdpSocket>
#include <QIODevice>
#include <qjson/parser.h>

#include "qplotdockwidget.h"
#include "qattitudedockwidget.h"
#include "qrcwidget.h"
#include "qlogdockwidget.h"



class MainWindow : public QMainWindow
{
    Q_OBJECT
private: 
    QTimer m_udpRecvTimer;

    QString m_sHostName;
    QUdpSocket *m_pUdpSocket;
    bool m_bUdpSockCon;
    
    char m_cUdpRecvBuf[256];
    QJson::Parser m_JSONParser;

    QTime m_tSensorTime;
    QList<QPair<unsigned long, QVariantMap> > m_lSensorReads;

    QRCWidget *m_pRCWidget;
    
    QString m_sStatBarText;
    QTextStream *m_pStatBarStream;
    QStatusBar *m_pStatusBar;
    
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

private slots:
    void sl_recvCommand();

    void sl_PrepareGraphs();
    void sl_UpdateSensorData(QPair<unsigned long, QVariantMap>);
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QAbstractSocket *getSocket() {
        return m_pUdpSocket;
    }

    void connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode = QIODevice::ReadWrite | QIODevice::Text);
};

#endif // MAINWINDOW_H
