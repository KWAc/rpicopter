#include "mainwindow.h"
#include <QStringRef>



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    m_pUdpSocket = new QUdpSocket(this);
    m_pUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    m_pLogger = new QLogDockWidget("Logger");
    m_pAttitude = new QAttitudeDockWidget("Attitude");
    m_pBattery = new QPlotDockWidget("Battery Monitor", 2);
    m_pBarometer = new QPlotDockWidget("Barometer Monitor", 3);
    m_pNetworkDelay = new QPlotDockWidget("Network (WiFi) Monitor", 1);

    sl_PrepareGraphs();

    m_pStatBarStream = new QTextStream(&m_sStatBarText);
    *m_pStatBarStream << "No data from sensors received!";

    m_pStatusBar = new QStatusBar(this);
    this->setStatusBar(m_pStatusBar);
    m_pStatusBar->showMessage(m_sStatBarText);
    
    this->addDockWidget(Qt::BottomDockWidgetArea, m_pLogger);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pNetworkDelay);
    this->addDockWidget(Qt::LeftDockWidgetArea, m_pAttitude);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pBattery);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pBarometer);

    m_udpRecvTimer.start(2);
    m_tSensorTime.start();
    memset(m_cUdpRecvBuf, 0, sizeof(m_cUdpRecvBuf) );
    m_bUdpSockCon = false;
    m_sHostName = "";

    m_pRCWidget = new QRCWidget(m_pUdpSocket, this);
    this->setCentralWidget(m_pRCWidget);
    m_pRCWidget->setFocus();

    connect(m_pUdpSocket, SIGNAL(connected() ), this, SLOT(sl_recvCommand() ) );
    connect(&m_udpRecvTimer, SIGNAL(timeout() ), this, SLOT(sl_recvCommand() ) );
}

MainWindow::~MainWindow()
{
}

void MainWindow::sl_PrepareGraphs() {
    m_pBarometer->GetGraph()->graph(0)->setPen(QPen(Qt::black));
    m_pBarometer->GetGraph()->graph(1)->setPen(QPen(Qt::black));
    m_pBarometer->GetGraph()->graph(2)->setPen(QPen(Qt::blue));
    m_pBarometer->GetGraph()->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 45))); // first graph will be filled with translucent blue
    m_pBarometer->GetGraph()->graph(1)->setBrush(QBrush(QColor(0, 0, 0, 25))); // first graph will be filled with translucent blue
    m_pBarometer->GetGraph()->graph(2)->setBrush(QBrush(QColor(0, 0, 255, 25))); // first graph will be filled with translucent blue
    //m_pBarometer->GetGraph()->xAxis2->setVisible(true);
    //m_pBarometer->GetGraph()->xAxis2->setTickLabels(false);
    m_pBarometer->GetGraph()->yAxis2->setVisible(true);
    m_pBarometer->GetGraph()->yAxis2->setTickLabels(false);
    m_pBarometer->GetGraph()->xAxis->setLabel("time in s");
    m_pBarometer->GetGraph()->yAxis->setLabel("climb rate in m/s");
    //m_pBarometer->GetGraph()->xAxis2->setLabel("Barometer monitor");
    m_pBarometer->GetGraph()->yAxis2->setLabel("Altitude in m");
    m_pBarometer->GetGraph()->graph(0)->rescaleAxes(true);
    m_pBarometer->GetGraph()->graph(1)->rescaleAxes(true);
    m_pBarometer->GetGraph()->graph(2)->rescaleAxes(true);
    m_pBarometer->GetGraph()->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    m_pBattery->GetGraph()->graph(0)->setPen(QPen(Qt::black));
    m_pBattery->GetGraph()->graph(1)->setPen(QPen(Qt::blue));
    m_pBattery->GetGraph()->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 25)));
    m_pBattery->GetGraph()->graph(1)->setBrush(QBrush(QColor(0, 0, 255, 25)));
    //m_pBattery->GetGraph()->xAxis2->setVisible(true);
    //m_pBattery->GetGraph()->xAxis2->setTickLabels(false);
    m_pBattery->GetGraph()->yAxis2->setVisible(true);
    m_pBattery->GetGraph()->yAxis2->setTickLabels(false);
    m_pBattery->GetGraph()->xAxis->setLabel("time in s");
    m_pBattery->GetGraph()->yAxis->setLabel("voltage in V");
    //m_pBattery->GetGraph()->xAxis2->setLabel("Current/Voltage monitor");
    m_pBattery->GetGraph()->yAxis2->setLabel("current in A");
    m_pBattery->GetGraph()->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    m_pNetworkDelay->GetGraph()->graph(0)->setPen(QPen(Qt::black));
    m_pNetworkDelay->GetGraph()->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 25)));
    //m_pNetworkDelay->GetGraph()->xAxis2->setVisible(true);
    //m_pNetworkDelay->GetGraph()->xAxis2->setTickLabels(false);
    m_pNetworkDelay->GetGraph()->xAxis->setLabel("time in s");
    m_pNetworkDelay->GetGraph()->yAxis->setLabel("latency in ms");
    m_pNetworkDelay->GetGraph()->xAxis2->setLabel("Network monitor");
    m_pNetworkDelay->GetGraph()->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void MainWindow::sl_UpdateSensorData(QPair<unsigned long, QVariantMap> sensorRead) {
    double time_s = sensorRead.first;
    QVariantMap map = sensorRead.second;

    #ifndef _WIN32
    QProcess ping;
    ping.start("ping", QStringList() << "-c 1" << m_sHostName);
    if(ping.waitForFinished(250) ) {
        while(ping.canReadLine()) {
            QString line = ping.readLine();
            if(line.contains("time=")) {
                int iStart = line.indexOf("time=") + 5;
                int iStop = line.indexOf(" ms");
                QStringRef latency(&line, iStart, iStop-iStart);

                m_vNetwork_s.append(time_s);
                m_vLatency_ms.append(QString(latency.toAscii()).toDouble());

                m_pNetworkDelay->GetGraph()->graph(0)->setData(m_vNetwork_s, m_vLatency_ms);
                m_pNetworkDelay->GetGraph()->graph(0)->rescaleAxes(true);
                m_pNetworkDelay->GetGraph()->replot();

                break;
            }
        }
    }
    #endif

    // Sensors read-outs
    // Compass
    if(map["type"].toString() == "s_cmp") {

    }
    // Barometer
    if(map["type"].toString() == "s_bar") {
        m_vBarometer_s.append(time_s);
        m_vAirPressure.append(map["p"].toDouble() / 100000 );
        m_vAltitude_m.append(map["a"].toDouble() );
        m_vTemperature_C.append(map["t"].toDouble() );
        m_vClimbrate_ms.append(map["c"].toDouble() );
        m_vTemperature_samples.append(map["s"].toDouble() );
        
        m_sStatBarText.clear();
        *m_pStatBarStream << "Temperature: " << map["t"].toDouble() << " Celsius;\t Air pressure: " << map["p"].toDouble() << " Pascal";
        m_pStatusBar->showMessage(m_sStatBarText, 5000);

        m_pBarometer->GetGraph()->graph(0)->setData(m_vBarometer_s, m_vAirPressure);
        m_pBarometer->GetGraph()->graph(1)->setData(m_vBarometer_s, m_vAltitude_m);
        m_pBarometer->GetGraph()->graph(2)->setData(m_vBarometer_s, m_vClimbrate_ms);

        m_pBarometer->GetGraph()->graph(0)->rescaleAxes(true);
        m_pBarometer->GetGraph()->graph(1)->rescaleAxes(true);
        m_pBarometer->GetGraph()->graph(2)->rescaleAxes(true);

        m_pBarometer->GetGraph()->replot();
    }
    // GPS
    if(map["type"].toString() == "s_gps") {

    }
    // Battery
    if(map["type"].toString() == "s_bat") {
        m_vBattery_s.append(time_s);
        m_vBattery_V.append(map["V"].toDouble() );
        m_vBattery_A.append(map["A"].toDouble() );

        m_pBattery->GetGraph()->graph(0)->setData(m_vBattery_s, m_vBattery_V);
        m_pBattery->GetGraph()->graph(1)->setData(m_vBattery_s, m_vBattery_A);

        m_pBattery->GetGraph()->graph(0)->rescaleAxes(true);
        m_pBattery->GetGraph()->graph(1)->rescaleAxes(true);

        m_pBattery->GetGraph()->replot();
    }
    // Attitude
    if(map["type"].toString() == "s_att") {
        qreal fRol = map["r"].toDouble();
        qreal fPit = map["p"].toDouble();
        //double fYaw = map["y"].toDouble();

        m_pAttitude->GetIndicator()->setRoll(fRol);
        m_pAttitude->GetIndicator()->setPitch(fPit);
    }

    // Current configuration
    // PID configuration
    if(map["type"].toString() == "pid_cnf") {

    }
}

void MainWindow::sl_recvCommand() {
    if(!m_bUdpSockCon)
        return;

    // if data coming, then clean last message
    if(m_pUdpSocket->bytesAvailable() ) {
        QByteArray line(m_cUdpRecvBuf);
        bool bOk = false;
        QVariantMap result = m_JSONParser.parse(line, &bOk).toMap();
        if(!bOk) {
            qFatal("sl_recvCommand - An error occured during parsing");
        }
        else {
            double fTimeElapsed_s = ((double)m_tSensorTime.elapsed() / 1000.f);
            QPair<unsigned long, QVariantMap> pair(fTimeElapsed_s, result);
            m_lSensorReads.append(pair);
            
            sl_UpdateSensorData(pair);
            
            if(line != "") {
                QString sLog = "";
                QTextStream sSLog(&sLog); sSLog << fTimeElapsed_s << " s: " << line << '\n';
                m_pLogger->getTextEdit()->insertPlainText(sLog);

                QTextCursor c =  m_pLogger->getTextEdit()->textCursor();
                c.movePosition(QTextCursor::End);
                m_pLogger->getTextEdit()->setTextCursor(c);
            }
        }
        
        memset(m_cUdpRecvBuf, 0, sizeof(m_cUdpRecvBuf) );
    }

    qint64 lineLength = m_pUdpSocket->read(m_cUdpRecvBuf, sizeof(m_cUdpRecvBuf) );
    if (lineLength != -1) {
         qDebug() << m_cUdpRecvBuf;
    }
}

void MainWindow::connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode) {
    m_pUdpSocket->connectToHost(hostName, port, openMode);
    m_bUdpSockCon = m_pUdpSocket->waitForConnected(1000);
    if(m_pUdpSocket->state() == QUdpSocket::ConnectedState) {
        qDebug("connectToHost: UDP socket connected");
        m_sHostName = hostName;
        m_pRCWidget->sl_startTimer();
    }
    else {
        qDebug() << "connectToHost - Not connected: " << m_pUdpSocket->error();
    }
}
