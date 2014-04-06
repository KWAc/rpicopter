#include "mainwindow.h"
#include <QJsonDocument>
#include <QStringRef>



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    searchSerialRadio();

    m_pStatusBar     = new QStatusBar(this);
    m_pUdpSocket     = new QUdpSocket(this);
    m_pPIDConfigDial = new QPIDConfig(m_pUdpSocket);
    m_pRCWidget      = new QRCWidget(m_pUdpSocket, &m_pSerialPort, this);
    m_pStatBarStream = new QTextStream(&m_sStatBarText);

    m_pThrottleBar   = new QProgressBar(this);

    m_pPIDConfig     = new QPIDDockWidget("PID Configuration");
    m_pLogger        = new QLogDockWidget("Logger");
    m_pAttitude      = new QAttitudeDockWidget("Attitude");
    m_pBattery       = new QPlotDockWidget("Battery Monitor", 2);
    m_pBarometer     = new QPlotDockWidget("Barometer Monitor", 3);
    m_pNetworkDelay  = new QPlotDockWidget("Network (WiFi) Monitor", 1);

    this->addDockWidget(Qt::BottomDockWidgetArea, m_pAttitude);
    this->addDockWidget(Qt::BottomDockWidgetArea, m_pPIDConfig);
    this->addDockWidget(Qt::BottomDockWidgetArea, m_pLogger);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pNetworkDelay);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pBattery);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pBarometer);

    this->setStatusBar(m_pStatusBar);

    prepareMenus();
    prepareGraphs();
    prepareWidgets();
    connectWidgets();
}

void MainWindow::prepareMenus() {
    // Prepare menu bars
    m_pFileM = new QMenu(tr("File"), this);
    m_pOptionM = new QMenu(tr("Options"), this);
    this->menuBar()->addMenu(m_pFileM);
    this->menuBar()->addMenu(m_pOptionM);

    // Define actions
    m_pFileMSave        = new QAction(tr("Save Log"), m_pFileM);
    m_pOptionMPIDConf   = new QAction(tr("Configurate PIDs"), m_pOptionM);
    m_pOptionRadioEnabled = new QAction(tr("Enable radio"), m_pOptionM);
    m_pOptionRadioEnabled->setCheckable(true);
    m_pOptionRadioEnabled->setChecked(true);

    // Add action to menu
    m_pFileM->addAction(m_pFileMSave);

    m_pOptionM->addAction(m_pOptionMPIDConf);
    m_pOptionM->addSeparator();
    m_pOptionM->addAction(m_pOptionRadioEnabled);
}

void MainWindow::prepareWidgets() {
    m_tSensorTime.start();

    *m_pStatBarStream << "No data from sensors received!";
    m_pStatusBar->showMessage(m_sStatBarText);

    memset(m_cUdpRecvBuf, 0, sizeof(m_cUdpRecvBuf) );
    m_bUdpSockCon = false;
    m_sHostName = "";


    m_pThrottleBar->setOrientation(Qt::Vertical);
    m_pThrottleBar->setMinimum(m_pRCWidget->m_RANGE.THR_MIN);
    m_pThrottleBar->setMaximum(m_pRCWidget->m_RANGE.THR_80P);
    m_pThrottleBar->setTextVisible(true);
    //m_pThrottleBar->setTextDirection();

    m_pMainLayout = new QHBoxLayout(this);

    m_pMainLayout->addWidget(m_pRCWidget);
    m_pMainLayout->addWidget(m_pThrottleBar);

    m_pMainWidget = new QGroupBox(this);
    m_pMainWidget->setLayout(m_pMainLayout);

    this->setCentralWidget(m_pMainWidget);

    m_pRCWidget->setDisabled(true);
}

void MainWindow::sl_setThrottleBar(int value) {
    if(value >= m_pThrottleBar->maximum() )
        return;
    m_pThrottleBar->setValue(value);
}

void MainWindow::connectWidgets() {
    connect(m_pUdpSocket, SIGNAL(connected() ), this, SLOT(sl_recvCommand() ) );
    connect(&m_udpRecvTimer, SIGNAL(timeout() ), this, SLOT(sl_recvCommand() ) );
    connect(&m_plotTimer, SIGNAL(timeout() ), this, SLOT(sl_replotGraphs() ) );
    connect(m_pFileMSave, SIGNAL(triggered() ), this, SLOT(sl_saveLog() ) );

    connect(m_pOptionMPIDConf, SIGNAL(triggered() ), this, SLOT(sl_configPIDs() ) );
    connect(m_pOptionRadioEnabled, SIGNAL(toggled(bool) ) , m_pRCWidget, SLOT(sl_setRadioEnabled(bool) ) );
    connect(m_pRCWidget, SIGNAL(si_throttleChanged(int) ), this, SLOT(sl_setThrottleBar(int) ) );
    connect(m_pRCWidget, SIGNAL(si_send2Model(QString) ), this, SLOT(sl_updateStatusBar(QString) ) );
}

bool MainWindow::searchSerialRadio() {
    bool bRes = false;
    // Example use QSerialPortInfo
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        qDebug() << "Name        : " << info.portName();
        qDebug() << "Description : " << info.description();
        qDebug() << "Manufacturer: " << info.manufacturer();

        if(info.description().contains("FT232R")) {
            // Example use QSerialPort
            m_pSerialPort.setPort(info);
            if (m_pSerialPort.open(QIODevice::ReadWrite)) {
                m_pSerialPort.setBaudRate(QSerialPort::Baud9600);
                bRes = true;
            }
        }
    }
    return bRes;
}

MainWindow::~MainWindow() {
    m_pSerialPort.close();
}

void MainWindow::sl_saveLog() {
    QString sFileName = QFileDialog::getSaveFileName(this, tr("Save sensor log file"),
                                                     QString(), tr("txt (*.txt *.log *.json)"));
    QFile file( sFileName );
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
             return;

    QTextStream out(&file);
    out << m_pLogger->getTextEdit()->toPlainText();

    file.close();
}

void MainWindow::sl_configPIDs() {
    m_pPIDConfigDial->setWindowTitle(tr("PID Configuration"));
    m_pPIDConfigDial->show();
    //m_pRCWidget->stop();
}

void MainWindow::prepareGraphs() {
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
                m_vLatency_ms.append(QString(latency.toLocal8Bit()).toDouble());
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
    }
    // GPS
    if(map["type"].toString() == "s_gps") {

    }
    // Battery
    if(map["type"].toString() == "s_bat") {
        m_vBattery_s.append(time_s);
        m_vBattery_V.append(map["V"].toDouble() );
        m_vBattery_A.append(map["A"].toDouble() );
    }
    // Attitude
    if(map["type"].toString() == "s_att") {
        qreal fRol = map["r"].toDouble();
        qreal fPit = map["p"].toDouble();
        float fYaw = map["y"].toDouble();

        m_pAttitude->GetIndicator()->setRoll(-fRol);
        m_pAttitude->GetIndicator()->setPitch(-fPit);
        m_pRCWidget->setYaw(fYaw);
    }

    // Current configuration
    // PID configuration
    if(map["type"].toString() == "pid_cnf") {
        m_pPIDConfig->sl_setPIDs(map);

        m_pPIDConfigDial->sl_Activate();
        m_pPIDConfigDial->sl_setPIDs(map);
    }
}

void MainWindow::sl_updateStatusBar(QString str) {
    m_pStatusBar->showMessage(m_sStatBarText + "\t Current RC-JSON: " + str, 5000);
}

void MainWindow::sl_replotGraphs() {
    m_pNetworkDelay->GetGraph()->graph(0)->setData(m_vNetwork_s, m_vLatency_ms);
    m_pNetworkDelay->GetGraph()->graph(0)->rescaleAxes(true);
    m_pNetworkDelay->GetGraph()->replot();

    m_pBarometer->GetGraph()->graph(0)->setData(m_vBarometer_s, m_vAirPressure);
    m_pBarometer->GetGraph()->graph(1)->setData(m_vBarometer_s, m_vAltitude_m);
    m_pBarometer->GetGraph()->graph(2)->setData(m_vBarometer_s, m_vClimbrate_ms);
    m_pBarometer->GetGraph()->graph(0)->rescaleAxes(true);
    m_pBarometer->GetGraph()->graph(1)->rescaleAxes(true);
    m_pBarometer->GetGraph()->graph(2)->rescaleAxes(true);
    m_pBarometer->GetGraph()->replot();

    m_pBattery->GetGraph()->graph(0)->setData(m_vBattery_s, m_vBattery_V);
    m_pBattery->GetGraph()->graph(1)->setData(m_vBattery_s, m_vBattery_A);
    m_pBattery->GetGraph()->graph(0)->rescaleAxes(true);
    m_pBattery->GetGraph()->graph(1)->rescaleAxes(true);
    m_pBattery->GetGraph()->replot();
}

bool checkForJSON(const QByteArray &array) {
    if(array.size() > 8)
        if(array.at(0) == '{' && array.at(array.size()-1) == '}')
            return true;
    return false;
}

void MainWindow::sl_recvCommand() {
    if(!m_bUdpSockCon)
        return;

    // if data coming, then clean last message
    if(m_pUdpSocket->bytesAvailable() ) {
        QByteArray line(m_cUdpRecvBuf);

        if(checkForJSON(line) ) {
            qDebug() << "Parse JSON: " << line;
            QJsonDocument JSONDoc = QJsonDocument::fromJson(line);
            QVariantMap result = JSONDoc.toVariant().toMap();

            if(result.empty() )
                return;

            double fTimeElapsed_s = ((double)m_tSensorTime.elapsed() / 1000.f);
            QPair<unsigned long, QVariantMap> pair(fTimeElapsed_s, result);
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
    if (lineLength > 0) {
         //qDebug() << m_cUdpRecvBuf;
    }
}

void MainWindow::connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode) {
    m_pUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    m_pUdpSocket->connectToHost(hostName, port, openMode);
    m_bUdpSockCon = m_pUdpSocket->waitForConnected(1000);

    if(m_pUdpSocket->state() == QUdpSocket::ConnectedState) {
        m_pUdpSocket->write(0, 0);
        qDebug("connectToHost: UDP socket connected");
        m_sHostName = hostName;
    }
    else {
        qDebug() << "connectToHost - Not connected: " << m_pUdpSocket->error();
    }

    m_pRCWidget->sl_startTimer();
    m_pRCWidget->setDisabled(false);
    m_pRCWidget->setFocus();

    m_udpRecvTimer.start(25);
    m_plotTimer.start(1000);
}
