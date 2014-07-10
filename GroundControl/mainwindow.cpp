#include "mainwindow.h"
#include <QJsonDocument>
#include <QStringRef>
#include <cassert>


MainWindow::MainWindow(QSettings *pConf, QWidget *parent)
    : QMainWindow(parent)
{
    m_pConf = pConf;

    searchSerialRadio();
    
    m_iCurrentPingID   = 0;
    m_fAveragePing     = 0.f;
    m_sStatBarPing     = "not calculated";

    m_iCurrentPingSent = 0;
    m_iCurrentPingRecv = 0;
    m_fSLTime_s        = 0.f;

    m_pStatusBar     = new QStatusBar(this);
    m_pUdpSocket     = new QUdpSocket(this);
    m_pPIDConfigDial = new QPIDConfig(m_pUdpSocket);
    m_pRCWidget      = new QRCWidget(m_pUdpSocket, &m_pSerialPort, this);
    m_pStatBarStream = new QTextStream(&m_sStatBarSensor);

    m_pRCThrottle    = new QProgressBar(this);
    m_pGMaps         = new Maps(this);

    m_pPIDConfig     = new QPIDDockWidget("PID Configuration");
    m_pLogger        = new QLogDockWidget("Logger");
    m_pAttitude      = new QAttitudeDockWidget("Attitude");
    m_pBattery       = new QPlotDockWidget("Battery Monitor", 2);
    m_pBarometer     = new QPlotDockWidget("Barometer Monitor", 3);
    m_pNetworkDelay  = new QPlotDockWidget("Network (WiFi) Monitor", 1);

    m_pPIDConfig->setFeatures(QDockWidget::DockWidgetMovable);
    m_pLogger->setFeatures(QDockWidget::DockWidgetMovable);
    m_pAttitude->setFeatures(QDockWidget::DockWidgetMovable);
    m_pBattery->setFeatures(QDockWidget::DockWidgetMovable);
    m_pBarometer->setFeatures(QDockWidget::DockWidgetMovable);
    m_pNetworkDelay->setFeatures(QDockWidget::DockWidgetMovable);

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

    m_pTrimProfiles->loadConfig();
}

void MainWindow::prepareMenus() {
    assert(m_pConf != NULL);

    // Prepare menu bars
    m_pFileM = new QMenu(tr("File"), this);
    m_pOptionM = new QMenu(tr("Options"), this);
    this->menuBar()->addMenu(m_pFileM);
    this->menuBar()->addMenu(m_pOptionM);

    // Define actions
    m_pFileMSave        = new QAction(tr("Save Log"), m_pFileM);
    m_pFileMLoad        = new QAction(tr("Load Log"), m_pFileM);
    m_pOptionMPIDConf   = new QAction(tr("Configurate PIDs"), m_pOptionM);
    m_pOptionRadioEnabled = new QAction(tr("Enable radio"), m_pOptionM);
    m_pOptionRadioEnabled->setCheckable(true);
    m_pOptionTrackingEnabled = new QAction(tr("Enable tracking"), m_pOptionM);
    m_pOptionTrackingEnabled->setCheckable(true);
    m_pOptionPing       = new QAction(tr("Configurate ping timing"), m_pFileM);
    m_pOptionHost       = new QAction(tr("Configurate network"), m_pFileM);
    m_pOptionProfiles   = new QAction(tr("Trim profiles"), m_pOptionM);

    bool bEnableRadio = m_pConf->value("UseRadio", true).toBool();
    m_pOptionRadioEnabled->setChecked(bEnableRadio);
    m_pConf->setValue("UseRadio", bEnableRadio);

    bool bEnableTracking = m_pConf->value("GPSTracking", true).toBool();
    m_pOptionTrackingEnabled->setChecked(bEnableTracking);
    m_pConf->setValue("GPSTracking", bEnableTracking);

    // Add action to menu
    m_pFileM->addAction(m_pFileMSave);
    m_pFileM->addAction(m_pFileMLoad);

    m_pOptionM->addAction(m_pOptionProfiles);
    m_pOptionM->addAction(m_pOptionMPIDConf);
    m_pOptionM->addAction(m_pOptionPing);
    m_pOptionM->addAction(m_pOptionHost);
    m_pOptionM->addSeparator();
    m_pOptionM->addAction(m_pOptionRadioEnabled);
    m_pOptionM->addAction(m_pOptionTrackingEnabled);

    connect(m_pFileMSave, SIGNAL(triggered() ), this, SLOT(sl_saveLog() ) );
    connect(m_pFileMLoad, SIGNAL(triggered() ), this, SLOT(sl_loadLog() ) );

    connect(m_pOptionMPIDConf, SIGNAL(triggered() ), this, SLOT(sl_configPIDs() ) );
    connect(m_pOptionPing, SIGNAL(triggered() ), this, SLOT(sl_configPing() ) );
    connect(m_pOptionHost, SIGNAL(triggered() ), this, SLOT(sl_configHost() ) );

    connect(m_pOptionRadioEnabled, SIGNAL(toggled(bool) ) , this, SLOT(sl_radioToggleChanged(bool) ) );
    connect(m_pOptionTrackingEnabled, SIGNAL(toggled(bool) ), this, SLOT(sl_trackingToggleChanged(bool) ) );

    connect(m_pOptionProfiles, SIGNAL(triggered() ), this, SLOT(sl_configProfiles() ) );
}

void MainWindow::sl_configProfiles() {
    m_pTrimProfiles->show();
}

void MainWindow::sl_trackingToggleChanged(bool state) {
    m_pConf->setValue("GPSTracking", state);
    // Remove the quad in the center of the map if tracking is disabled
    if(!state) {
        m_pGMaps->sl_clearQuad();
    }
}

void MainWindow::sl_configHost() {
    bool ok;

    QString sHost =  m_pConf->value("HostIP", "192.168.42.1").toString() + ":" + m_pConf->value("HostPort", 7000).toString();
    QString text = QInputDialog::getText(this, tr("Configurate client"),
                                         tr("Host adress (IP and port):"), QLineEdit::Normal,
                                         sHost, &ok);
    if(ok && !text.isEmpty()) {
        QStringList host = text.split(":");
        if(host.size() == 2) {
            connectToHost(host.at(0), host.at(1).toInt() );
            m_pConf->setValue("HostIP", host.at(0));
            m_pConf->setValue("HostPort", host.at(1));
            qDebug() << "HostIP: " << host.at(0) << "HostPort: " << host.at(1);
        }
        else {
            qDebug() << "Host adress syntax not valid.";
        }
    }
}

void MainWindow::sl_configPing() {
    bool ok;
    int t_ms = QInputDialog::getInt(this, tr("Enter Ping in ms"),
                                    tr("Time:"), m_pConf->value("Ping_ms", PING_T_MS).toInt(), 1, 1000, 1, &ok);
    if (ok) {
        m_pingTimer.start(t_ms);
        m_pConf->setValue("Ping_ms", t_ms);
    }
}

void MainWindow::prepareWidgets() {
    m_tSensorTime.start();

    *m_pStatBarStream << "No data from sensors received!";
    m_pStatusBar->showMessage(m_sStatBarSensor);

    m_bUdpSockCon = false;

    m_pTrimProfiles = new TrimProfile(m_pConf);

    m_pRCThrottle->setOrientation(Qt::Vertical);
    m_pRCThrottle->setMinimum(m_pRCWidget->m_RANGE.THR_MIN);
    m_pRCThrottle->setMaximum(m_pRCWidget->m_RANGE.THR_80P);
    m_pRCThrottle->setTextVisible(true);
    //m_pRCThrottle->setTextDirection();
    
    QGridLayout *pRCLayout = new QGridLayout;
    //pRCLayout->addWidget(m_pTrimProfiles, 0, 0);
    pRCLayout->addWidget(m_pRCWidget, 0, 0);
    pRCLayout->addWidget(m_pRCThrottle, 0, 1);
    
    QGroupBox *pRCGroup = new QGroupBox(this);
    pRCGroup->setLayout(pRCLayout);

    m_pMainTab = new QTabWidget(this);
    setCentralWidget(m_pMainTab);
    m_pMainTab->addTab(pRCGroup, "Model control");
    m_pMainTab->addTab(m_pGMaps, "Route planner");
    
    m_pRCWidget->setDisabled(true);
}

void MainWindow::connectWidgets() {
    connect(&m_pingTimer, SIGNAL(timeout() ), this, SLOT(sl_sendPing() ) );
    connect(m_pUdpSocket, SIGNAL(readyRead() ), this, SLOT(sl_recvCommand() ) );
    connect(&m_plotTimer, SIGNAL(timeout() ), this, SLOT(sl_replotGraphs() ) );
    
    connect(m_pRCWidget, SIGNAL(si_send2Model(QString, QString) ), this, SLOT(sl_updateStatusBar(QString, QString) ) );
    connect(m_pRCWidget, SIGNAL(si_attitudeCorrChanged(float, float) ), m_pTrimProfiles, SLOT(sl_updateTrim(float, float) ) );
    
    connect(m_pMainTab, SIGNAL(currentChanged(int) ), this, SLOT(sl_changeTab(int) ) );
    connect(m_pTrimProfiles, SIGNAL(si_trimChanged(float, float) ), m_pRCWidget, SLOT(sl_setAttitudeCorr(float, float) ) );
}

void MainWindow::sl_changeTab(int index) {
    // Hide all remote control widget if the tab is changed
    // to another category
    if(index) {
        m_pAttitude->hide();
        m_pPIDConfig->hide();
        m_pNetworkDelay->hide();
        m_pBattery->hide();
        m_pBarometer->hide();
    }
    // RC-Tab
    else {
        m_pAttitude->show();
        m_pPIDConfig->show();
        m_pNetworkDelay->show();
        m_pBattery->show();
        m_pBarometer->show();
    }
}

void MainWindow::sl_radioToggleChanged(bool state) {
    assert(m_pConf != NULL);
    m_pConf->setValue("UseRadio", state);
    m_pRCWidget->sl_setRadioEnabled(state);
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

void MainWindow::sl_loadLog() {    
    QString sFileName = QFileDialog::getOpenFileName(this, tr("Open sensor log file"),
                                                     QString(), tr("txt (*.txt *.log *.json)"));
    QFile file( sFileName );
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;

    QTextStream in(&file);
    QString text = "";
    QString line = "";
    bool bLoadText = true;
    do {
        line = in.readLine();
        text += line + "\n";
        qDebug() << "Read from file: " << line;

        // Remove white spaces
        line.replace( " ", "" );

        QStringList sCurRead = line.split(':');

        if(sCurRead.size() < 2) {
            qDebug() << "Not valid read";
            bLoadText = false;
            continue;
        }        
        // Time
        QString sTime = sCurRead.at(0);
        sTime.replace( "s", "" );
        m_fSLTime_s = sTime.toDouble();

        // JSON
        sTime += "s:";
        line.replace( sTime, "" );
        QByteArray cJSON = line.toUtf8();
        qDebug() << "Pure JSON: " << cJSON;

        QJsonDocument JSONDoc = QJsonDocument::fromJson(cJSON);
        QVariantMap varSensor = JSONDoc.toVariant().toMap();

        // Update sensor widgets
        QPair<double, QVariantMap> pair(m_fSLTime_s, varSensor);
        sl_UpdateSensorData(pair);
    } while (!line.isNull() );

    // Load text into logger
    if(bLoadText) {
        m_pLogger->getTextEdit()->setPlainText(text);
    }
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

void MainWindow::sl_sendPing() {
    QString com = "";
    
    com.append("{\"type\":\"ping\",\"v\":");
    com.append(QString::number(m_iCurrentPingID) );
    com.append("}");
    
    qint64 bytes = m_pUdpSocket->write(com.toLatin1(), com.size() );
    if(bytes) {
      m_iCurrentPingSent = m_tSensorTime.elapsed();
      m_iCurrentPingRecv = m_iCurrentPingSent;
    }
}

QAbstractSocket *MainWindow::getSocket() {
    return m_pUdpSocket;
}

void MainWindow::sl_UpdateSensorData(QPair<double, QVariantMap> sensorRead) {
    static float fHeading = 0.f;

    static int iAvrPingCntr = 0;
    static float fAvrPing = 0.f;

    double time_s = sensorRead.first;
    QVariantMap map = sensorRead.second;

    // Latency of the connection
    if(map["type"].toString() == "pong") {
        int iCurrentPongID = map["v"].toInt();
        if(m_iCurrentPingID == iCurrentPongID) {
            m_iCurrentPingRecv = m_tSensorTime.elapsed();
            // Update the plot
            double ping_ms = (double)(m_iCurrentPingRecv - m_iCurrentPingSent);

            if(iAvrPingCntr < 25) {
                fAvrPing += ping_ms;
                iAvrPingCntr++;
            } else {
                m_fAveragePing = fAvrPing/25.f;
                m_sStatBarPing = QString::number(m_fAveragePing) + " ms";

                iAvrPingCntr = 0;
                fAvrPing = 0.f;
            }

            if(ping_ms < PING_T_MS && ping_ms >= 0) {
                m_vNetwork_s.append(time_s);
                m_vLatency_ms.append(ping_ms);
                // New identifier for the next ping message
                m_iCurrentPingID++;
            }
        }
    }
    // If not a ping/pong JSON than add to the logger
    else {
        if(m_udpCurLine != "") {
            QString sLog = "";
            QTextStream sSLog(&sLog); sSLog << time_s << " s: " << m_udpCurLine << '\n';
            m_pLogger->getTextEdit()->insertPlainText(sLog);

            QTextCursor c =  m_pLogger->getTextEdit()->textCursor();
            c.movePosition(QTextCursor::End);
            m_pLogger->getTextEdit()->setTextCursor(c);
        }
    }

    // Sensors read-outs
    // Compass
    if(map["type"].toString() == "s_cmp") {
        fHeading = map["h"].toDouble();
    }
    // Barometer
    if(map["type"].toString() == "s_bar") {
        m_vBarometer_s.append(time_s);
        m_vAirPressure.append(map["p"].toDouble() / 100000 );
        m_vAltitude_m.append(map["a"].toDouble() );
        m_vTemperature_C.append(map["t"].toDouble() );
        m_vClimbrate_ms.append(map["c"].toDouble() );
        m_vTemperature_samples.append(map["s"].toDouble() );

        m_sStatBarSensor.clear();
        *m_pStatBarStream << "Temperature: " << map["t"].toDouble() << " Celsius;\t Air pressure: " << map["p"].toDouble() << " Pascal";
        m_pStatusBar->showMessage(m_sStatBarSensor, 5000);
    }
    // GPS
    if(map["type"].toString() == "s_gps") {
        if(!m_pOptionTrackingEnabled->isChecked() ) {
            return;
        }
        float fLat = map["lat"].toDouble();
        float fLon = map["lon"].toDouble();
        m_pGMaps->sl_setQuad(fLat, fLon, fHeading);
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

void MainWindow::sl_updateStatusBar() {
    m_pStatusBar->showMessage(m_sStatBarSensor + "\t Current RC-JSON: " + m_sStatBarRC + "\t Current options: " + m_sStatBarOptions + "\t Average ping: " + m_sStatBarPing, 5000);
}

void MainWindow::sl_updateStatusBar(QString str, QString type) {
    if(type.contains("command") ) {
      m_sStatBarRC = str;
      
      // Update the current throttle bar
      QByteArray line;
      line.append(m_sStatBarRC);

      //qDebug() << "Parse JSON: " << line;
      QJsonDocument JSONDoc = QJsonDocument::fromJson(line);
      QVariantMap result = JSONDoc.toVariant().toMap();
      int iThr = result["t"].toInt();
      if(iThr < m_pRCThrottle->maximum() )
        m_pRCThrottle->setValue(iThr);
    }
    else if(type.contains("option") ) {
      m_sStatBarOptions = str;
    }
    sl_updateStatusBar();
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

void MainWindow::sl_recvCommand() {
    if(!m_bUdpSockCon)
        return;

    if(!m_pUdpSocket->bytesAvailable() ) {
        return;
    }

    char buffer[512];
    qint64 lineLength = m_pUdpSocket->read(buffer, sizeof(buffer) );
    m_udpCurLine = QByteArray(buffer, lineLength);
    QJsonDocument JSONDoc = QJsonDocument::fromJson(m_udpCurLine);
    QVariantMap result = JSONDoc.toVariant().toMap();

    if(result.empty() ) {
        return;
    }

    double fTimeElapsed_s = ((double)m_tSensorTime.elapsed() / 1000.f) + m_fSLTime_s;
    sl_UpdateSensorData(QPair<double, QVariantMap>(fTimeElapsed_s, result) );
}

void MainWindow::connectToHost(const QString & hostName, quint16 port, QIODevice::OpenMode openMode) {
    if(m_pUdpSocket->state() == QUdpSocket::ConnectedState) {
        m_pUdpSocket->close();
    }

    m_pUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    m_pUdpSocket->connectToHost(hostName, port, openMode);
    m_bUdpSockCon = m_pUdpSocket->waitForConnected(1000);

    if(m_pUdpSocket->state() == QUdpSocket::ConnectedState) {
        m_pUdpSocket->write(0, 0);
        qDebug("connectToHost: UDP socket connected");
        
        // Update graphs and ping service only necessary if connected to UDP socket
        int ping_ms = m_pConf->value("Ping_ms", PING_T_MS).toInt();
        m_pConf->setValue("Ping_ms", ping_ms);
        m_pingTimer.start(ping_ms);
        m_plotTimer.start(1000);
    }
    else {
        qDebug() << "connectToHost - Not connected: " << m_pUdpSocket->error();
    }

    m_pRCWidget->start();
    m_pRCWidget->setDisabled(false);
    m_pRCWidget->setFocus();
}
