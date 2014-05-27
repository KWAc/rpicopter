#include "piddialog.h"


QPIDConfig::QPIDConfig(QUdpSocket *pSock, QWidget *parent) : QWidget(parent) {
    if(pSock)
        m_pUdpSock = pSock;

    Setup();
    sl_Deactivate();
}

void QPIDConfig::sl_Activate() {
    m_bSend = true;
    m_pit_rkp_V->setDisabled(false);
    m_pit_rki_V->setDisabled(false);
    m_pit_rkd_V->setDisabled(false);
    m_pit_rimax_V->setDisabled(false);

    m_rol_rkp_V->setDisabled(false);
    m_rol_rki_V->setDisabled(false);
    m_rol_rkd_V->setDisabled(false);
    m_rol_rimax_V->setDisabled(false);

    m_yaw_rkp_V->setDisabled(false);
    m_yaw_rki_V->setDisabled(false);
    m_yaw_rkd_V->setDisabled(false);
    m_yaw_rimax_V->setDisabled(false);

    m_thr_rkp_V->setDisabled(false);
    m_thr_rki_V->setDisabled(false);
    m_thr_rkd_V->setDisabled(false);
    m_thr_rimax_V->setDisabled(false);

    m_acc_rkp_V->setDisabled(false);
    m_acc_rki_V->setDisabled(false);
    m_acc_rkd_V->setDisabled(false);
    m_acc_rimax_V->setDisabled(false);

    m_pit_skp_V->setDisabled(false);
    m_rol_skp_V->setDisabled(false);
    m_yaw_skp_V->setDisabled(false);
    m_thr_skp_V->setDisabled(false);
    m_acc_skp_V->setDisabled(false);
}

void QPIDConfig::sl_Deactivate() {
    m_bSend = false;
    m_pit_rkp_V->setDisabled(true);
    m_pit_rki_V->setDisabled(true);
    m_pit_rkd_V->setDisabled(true);
    m_pit_rimax_V->setDisabled(true);

    m_rol_rkp_V->setDisabled(true);
    m_rol_rki_V->setDisabled(true);
    m_rol_rkd_V->setDisabled(true);
    m_rol_rimax_V->setDisabled(true);

    m_yaw_rkp_V->setDisabled(true);
    m_yaw_rki_V->setDisabled(true);
    m_yaw_rkd_V->setDisabled(true);
    m_yaw_rimax_V->setDisabled(true);

    m_thr_rkp_V->setDisabled(true);
    m_thr_rki_V->setDisabled(true);
    m_thr_rkd_V->setDisabled(true);
    m_thr_rimax_V->setDisabled(true);

    m_acc_rkp_V->setDisabled(true);
    m_acc_rki_V->setDisabled(true);
    m_acc_rkd_V->setDisabled(true);
    m_acc_rimax_V->setDisabled(true);

    m_pit_skp_V->setDisabled(true);
    m_rol_skp_V->setDisabled(true);
    m_yaw_skp_V->setDisabled(true);
    m_thr_skp_V->setDisabled(true);
    m_acc_skp_V->setDisabled(true);
}

void QPIDConfig::sl_setPIDs(QVariantMap map) {
    if(this->isVisible() )
        return;

    if(map.contains("p_rkp") )
        m_pit_rkp_V->setValue(map["p_rkp"].toDouble() );
    if(map.contains("p_rki") )
        m_pit_rki_V->setValue(map["p_rki"].toDouble() );
    if(map.contains("p_rkd") )
        m_pit_rkd_V->setValue(map["p_rkd"].toDouble() );
    if(map.contains("p_rimax") )
        m_pit_rimax_V->setValue(map["p_rimax"].toDouble() );

    if(map.contains("r_rkp") )
        m_rol_rkp_V->setValue(map["r_rkp"].toDouble() );
    if(map.contains("r_rki") )
        m_rol_rki_V->setValue(map["r_rki"].toDouble() );
    if(map.contains("r_rkd") )
        m_rol_rkd_V->setValue(map["r_rkd"].toDouble() );
    if(map.contains("r_rimax") )
        m_rol_rimax_V->setValue(map["r_rimax"].toDouble() );

    if(map.contains("y_rkp") )
        m_yaw_rkp_V->setValue(map["y_rkp"].toDouble() );
    if(map.contains("y_rki") )
        m_yaw_rki_V->setValue(map["y_rki"].toDouble() );
    if(map.contains("y_rkd") )
        m_yaw_rkd_V->setValue(map["y_rkd"].toDouble() );
    if(map.contains("y_rimax") )
        m_yaw_rimax_V->setValue(map["y_rimax"].toDouble() );

    if(map.contains("t_rkp") )
        m_thr_rkp_V->setValue(map["t_rkp"].toDouble() );
    if(map.contains("t_rki") )
        m_thr_rki_V->setValue(map["t_rki"].toDouble() );
    if(map.contains("t_rkd") )
        m_thr_rkd_V->setValue(map["t_rkd"].toDouble() );
    if(map.contains("t_rimax") )
        m_thr_rimax_V->setValue(map["t_rimax"].toDouble() );

    if(map.contains("a_rkp") )
        m_acc_rkp_V->setValue(map["a_rkp"].toDouble() );
    if(map.contains("a_rki") )
        m_acc_rki_V->setValue(map["a_rki"].toDouble() );
    if(map.contains("a_rkd") )
        m_acc_rkd_V->setValue(map["a_rkd"].toDouble() );
    if(map.contains("a_rimax") )
        m_acc_rimax_V->setValue(map["a_rimax"].toDouble() );

    if(map.contains("p_skp") )
        m_pit_skp_V->setValue(map["p_skp"].toDouble() );
    if(map.contains("r_skp") )
        m_rol_skp_V->setValue(map["r_skp"].toDouble() );
    if(map.contains("y_skp") )
        m_yaw_skp_V->setValue(map["y_skp"].toDouble() );
    if(map.contains("t_skp") )
        m_thr_skp_V->setValue(map["t_skp"].toDouble() );
    if(map.contains("a_skp") )
        m_acc_skp_V->setValue(map["a_skp"].toDouble() );
}

void QPIDConfig::Setup() {
    s_pit_rkp = "Pit Rkp: ",  s_pit_rki = "Pit Rki: ",  s_pit_rkd = "Pit Rkd: ",  s_pit_rimax = "Pit RImax: ";
    s_rol_rkp = "Rol Rkp: ",  s_rol_rki = "Rol Rki: ",  s_rol_rkd = "Rol Rkd: ",  s_rol_rimax = "Rol RImax: ";
    s_yaw_rkp = "Yaw Rkp: ",  s_yaw_rki = "Yaw Rki: ",  s_yaw_rkd = "Yaw Rkd: ",  s_yaw_rimax = "Yaw RImax: ";
    s_thr_rkp = "Thr Rkp: ",  s_thr_rki = "Thr Rki: ",  s_thr_rkd = "Thr Rkd: ",  s_thr_rimax = "Thr Rimax: ";
    s_acc_rkp = "Acc Rkp: ",  s_acc_rki = "Acc Rki: ",  s_acc_rkd = "Acc Rkd: ",  s_acc_rimax = "Acc Rimax: ";
    s_pit_skp = "Pit Skp: ";
    s_rol_skp = "Rol Skp: ";
    s_yaw_skp = "Yaw Skp: ";
    s_thr_skp = "Thr Skp: ";
    s_acc_skp = "Acc Skp: ";

    m_pit_rkp = new QLabel(s_pit_rkp);
    m_pit_rki = new QLabel(s_pit_rki);
    m_pit_rkd = new QLabel(s_pit_rkd);
    m_pit_rimax = new QLabel(s_pit_rimax);

    m_rol_rkp = new QLabel(s_rol_rkp);
    m_rol_rki = new QLabel(s_rol_rki);
    m_rol_rkd = new QLabel(s_rol_rkd);
    m_rol_rimax = new QLabel(s_rol_rimax);

    m_yaw_rkp = new QLabel(s_yaw_rkp);
    m_yaw_rki = new QLabel(s_yaw_rki);
    m_yaw_rkd = new QLabel(s_yaw_rkd);
    m_yaw_rimax = new QLabel(s_yaw_rimax);

    m_thr_rkp = new QLabel(s_thr_rkp);
    m_thr_rki = new QLabel(s_thr_rki);
    m_thr_rkd = new QLabel(s_thr_rkd);
    m_thr_rimax = new QLabel(s_thr_rimax);

    m_thr_akp = new QLabel(s_acc_rkp);
    m_thr_aki = new QLabel(s_acc_rki);
    m_thr_akd = new QLabel(s_acc_rkd);
    m_thr_aimax = new QLabel(s_acc_rimax);

    m_pit_skp = new QLabel(s_pit_skp);
    m_rol_skp = new QLabel(s_rol_skp);
    m_yaw_skp = new QLabel(s_yaw_skp);
    m_thr_skp = new QLabel(s_thr_skp);
    m_acc_skp = new QLabel(s_acc_skp);

    m_pit_rkp_V = new QDoubleSpinBox();
    m_pit_rkp_V->setRange(0, 25);
    m_pit_rki_V = new QDoubleSpinBox();
    m_pit_rki_V->setRange(0, 25);
    m_pit_rkd_V = new QDoubleSpinBox();
    m_pit_rkd_V->setRange(0, 0.1);
    m_pit_rkd_V->setDecimals(4);
    m_pit_rimax_V = new QDoubleSpinBox();
    m_pit_rimax_V->setRange(0, 250);

    m_rol_rkp_V = new QDoubleSpinBox();
    m_rol_rkp_V->setRange(0, 25);
    m_rol_rki_V = new QDoubleSpinBox();
    m_rol_rki_V->setRange(0, 25);
    m_rol_rkd_V = new QDoubleSpinBox();
    m_rol_rkd_V->setRange(0, 0.1);
    m_rol_rkd_V->setDecimals(4);
    m_rol_rimax_V = new QDoubleSpinBox();
    m_rol_rimax_V->setRange(0, 250);

    m_yaw_rkp_V = new QDoubleSpinBox();
    m_yaw_rkp_V->setRange(0, 25);
    m_yaw_rki_V = new QDoubleSpinBox();
    m_yaw_rki_V->setRange(0, 25);
    m_yaw_rkd_V = new QDoubleSpinBox();
    m_yaw_rkd_V->setRange(0, 0.1);
    m_yaw_rkd_V->setDecimals(4);
    m_yaw_rimax_V = new QDoubleSpinBox();
    m_yaw_rimax_V->setRange(0, 250);

    m_thr_rkp_V = new QDoubleSpinBox();
    m_thr_rkp_V->setRange(0, 25);
    m_thr_rki_V = new QDoubleSpinBox();
    m_thr_rki_V->setRange(0, 25);
    m_thr_rkd_V = new QDoubleSpinBox();
    m_thr_rkd_V->setRange(0, 0.1);
    m_thr_rkd_V->setDecimals(4);
    m_thr_rimax_V = new QDoubleSpinBox();
    m_thr_rimax_V->setRange(0, 250);

    m_acc_rkp_V = new QDoubleSpinBox();
    m_acc_rkp_V->setRange(0, 25);
    m_acc_rki_V = new QDoubleSpinBox();
    m_acc_rki_V->setRange(0, 25);
    m_acc_rkd_V = new QDoubleSpinBox();
    m_acc_rkd_V->setRange(0, 0.1);
    m_acc_rkd_V->setDecimals(4);
    m_acc_rimax_V = new QDoubleSpinBox();
    m_acc_rimax_V->setRange(0, 250);

    m_pit_skp_V = new QDoubleSpinBox();
    m_pit_skp_V->setRange(0, 25);
    m_rol_skp_V = new QDoubleSpinBox();
    m_rol_skp_V->setRange(0, 25);
    m_yaw_skp_V = new QDoubleSpinBox();
    m_yaw_skp_V->setRange(0, 25);
    m_thr_skp_V = new QDoubleSpinBox();
    m_thr_skp_V->setRange(0, 25);
    m_acc_skp_V = new QDoubleSpinBox();
    m_acc_skp_V->setRange(0, 25);

    m_pit_rkp_V->setSingleStep(0.1);
    m_pit_rki_V->setSingleStep(0.1);
    m_pit_rkd_V->setSingleStep(0.0005);
    m_pit_rimax_V->setSingleStep(1);

    m_rol_rkp_V->setSingleStep(0.1);
    m_rol_rki_V->setSingleStep(0.1);
    m_rol_rkd_V->setSingleStep(0.0005);
    m_rol_rimax_V->setSingleStep(1);

    m_yaw_rkp_V->setSingleStep(0.1);
    m_yaw_rki_V->setSingleStep(0.1);
    m_yaw_rkd_V->setSingleStep(0.0005);
    m_yaw_rimax_V->setSingleStep(1);

    m_thr_rkp_V->setSingleStep(0.1);
    m_thr_rki_V->setSingleStep(0.1);
    m_thr_rkd_V->setSingleStep(0.0005);
    m_thr_rimax_V->setSingleStep(1);

    m_acc_rkp_V->setSingleStep(0.1);
    m_acc_rki_V->setSingleStep(0.1);
    m_acc_rkd_V->setSingleStep(0.0005);
    m_acc_rimax_V->setSingleStep(1);

    m_pit_skp_V->setSingleStep(0.1);
    m_rol_skp_V->setSingleStep(0.1);
    m_yaw_skp_V->setSingleStep(0.1);
    m_thr_skp_V->setSingleStep(0.1);
    m_acc_skp_V->setSingleStep(0.1);

    m_pit_rkp_V->setValue(0);
    m_pit_rki_V->setValue(0);
    m_pit_rkd_V->setValue(0);
    m_pit_rimax_V->setValue(0);

    m_rol_rkp_V->setValue(0);
    m_rol_rki_V->setValue(0);
    m_rol_rkd_V->setValue(0);
    m_rol_rimax_V->setValue(0);

    m_yaw_rkp_V->setValue(0);
    m_yaw_rki_V->setValue(0);
    m_yaw_rkd_V->setValue(0);
    m_yaw_rimax_V->setValue(0);

    m_thr_rkp_V->setValue(0);
    m_thr_rki_V->setValue(0);
    m_thr_rkd_V->setValue(0);
    m_thr_rimax_V->setValue(0);

    m_acc_rkp_V->setValue(0);
    m_acc_rki_V->setValue(0);
    m_acc_rkd_V->setValue(0);
    m_acc_rimax_V->setValue(0);

    m_pit_skp_V->setValue(0);
    m_rol_skp_V->setValue(0);
    m_yaw_skp_V->setValue(0);
    m_thr_skp_V->setValue(0);
    m_acc_skp_V->setValue(0);

    QGridLayout *pLayout = new QGridLayout(this);
    pLayout->setHorizontalSpacing(8);

    pLayout->addWidget(m_pit_rkp, 0, 0);
    pLayout->addWidget(m_pit_rki, 0, 2);
    pLayout->addWidget(m_pit_rkd, 0, 4);
    pLayout->addWidget(m_pit_rimax, 0, 6);

    pLayout->addWidget(m_rol_rkp, 1, 0);
    pLayout->addWidget(m_rol_rki, 1, 2);
    pLayout->addWidget(m_rol_rkd, 1, 4);
    pLayout->addWidget(m_rol_rimax, 1, 6);

    pLayout->addWidget(m_yaw_rkp, 2, 0);
    pLayout->addWidget(m_yaw_rki, 2, 2);
    pLayout->addWidget(m_yaw_rkd, 2, 4);
    pLayout->addWidget(m_yaw_rimax, 2, 6);

    pLayout->addWidget(m_thr_rkp, 3, 0);
    pLayout->addWidget(m_thr_rki, 3, 2);
    pLayout->addWidget(m_thr_rkd, 3, 4);
    pLayout->addWidget(m_thr_rimax, 3, 6);

    pLayout->addWidget(m_thr_akp, 4, 0);
    pLayout->addWidget(m_thr_aki, 4, 2);
    pLayout->addWidget(m_thr_akd, 4, 4);
    pLayout->addWidget(m_thr_aimax, 4, 6);

    pLayout->addWidget(m_pit_skp, 5, 0);
    pLayout->addWidget(m_rol_skp, 6, 0);
    pLayout->addWidget(m_yaw_skp, 7, 0);
    pLayout->addWidget(m_thr_skp, 8, 0);
    pLayout->addWidget(m_acc_skp, 9, 0);

    pLayout->addWidget(m_pit_rkp_V, 0, 1);
    pLayout->addWidget(m_pit_rki_V, 0, 3);
    pLayout->addWidget(m_pit_rkd_V, 0, 5);
    pLayout->addWidget(m_pit_rimax_V, 0, 7);

    pLayout->addWidget(m_rol_rkp_V, 1, 1);
    pLayout->addWidget(m_rol_rki_V, 1, 3);
    pLayout->addWidget(m_rol_rkd_V, 1, 5);
    pLayout->addWidget(m_rol_rimax_V, 1, 7);

    pLayout->addWidget(m_yaw_rkp_V, 2, 1);
    pLayout->addWidget(m_yaw_rki_V, 2, 3);
    pLayout->addWidget(m_yaw_rkd_V, 2, 5);
    pLayout->addWidget(m_yaw_rimax_V, 2, 7);

    pLayout->addWidget(m_thr_rkp_V, 3, 1);
    pLayout->addWidget(m_thr_rki_V, 3, 3);
    pLayout->addWidget(m_thr_rkd_V, 3, 5);
    pLayout->addWidget(m_thr_rimax_V, 3, 7);

    pLayout->addWidget(m_acc_rkp_V, 4, 1);
    pLayout->addWidget(m_acc_rki_V, 4, 3);
    pLayout->addWidget(m_acc_rkd_V, 4, 5);
    pLayout->addWidget(m_acc_rimax_V, 4, 7);

    pLayout->addWidget(m_pit_skp_V, 5, 1);
    pLayout->addWidget(m_rol_skp_V, 6, 1);
    pLayout->addWidget(m_yaw_skp_V, 7, 1);
    pLayout->addWidget(m_thr_skp_V, 8, 1);
    pLayout->addWidget(m_acc_skp_V, 9, 1);

    m_pButOK = new QPushButton(tr("OK") );
    m_pButCancel = new QPushButton(tr("Cancel"));
    pLayout->addWidget(m_pButOK, 10, 6);
    pLayout->addWidget(m_pButCancel, 10, 7);

    connect(m_pButCancel, SIGNAL(pressed() ), this, SLOT(close() ) );
    connect(m_pButOK, SIGNAL(pressed() ), this, SLOT(close() ) );
    connect(m_pButOK, SIGNAL(pressed() ), this, SLOT(sl_sendPIDs() ) );

    this->setLayout(pLayout);
}

void QPIDConfig::sl_sendPIDs() {
    sendPIDs(getPIDS() );
}

void QPIDConfig::sendPIDs(PIDS p) {
    if(!m_bSend){
        qDebug() << "PIDs not emitted";
        return;
    }

    QString com = "";
    QTextStream stream(&com);
    stream  << "{\"type\":\"pid\",\"p_rkp\":" << p.m_pit_rkp << ",\"p_rki\":" << p.m_pit_rki << ",\"p_rkd\":" << p.m_pit_rkd << ",\"p_rimax\":" << p.m_pit_rimax
                             << ",\"r_rkp\":" << p.m_rol_rkp << ",\"r_rki\":" << p.m_rol_rki << ",\"r_rkd\":" << p.m_rol_rkd << ",\"r_rimax\":" << p.m_rol_rimax
                             << ",\"y_rkp\":" << p.m_yaw_rkp << ",\"y_rki\":" << p.m_yaw_rki << ",\"y_rkd\":" << p.m_yaw_rkd << ",\"y_rimax\":" << p.m_yaw_rimax
                             << ",\"t_rkp\":" << p.m_thr_rkp << ",\"t_rki\":" << p.m_thr_rki << ",\"t_rkd\":" << p.m_thr_rkd << ",\"t_rimax\":" << p.m_thr_rimax
                             << ",\"a_rkp\":" << p.m_acc_rkp << ",\"a_rki\":" << p.m_acc_rki << ",\"a_rkd\":" << p.m_acc_rkd << ",\"a_rimax\":" << p.m_acc_rimax
                             << ",\"p_skp\":" << p.m_pit_skp
                             << ",\"r_skp\":" << p.m_rol_skp
                             << ",\"y_skp\":" << p.m_yaw_skp
                             << ",\"t_skp\":" << p.m_thr_skp
                             << ",\"a_skp\":" << p.m_acc_skp << "}";

    m_pUdpSock->write(com.toLocal8Bit(), com.length() );
    qDebug() << com;
}

PIDS QPIDConfig::getPIDS() {
    PIDS p;

    p.m_acc_rimax = m_acc_rimax_V->value();
    p.m_acc_rkd = m_acc_rkd_V->value();
    p.m_acc_rki = m_acc_rki_V->value();
    p.m_acc_rkp = m_acc_rkp_V->value();
    p.m_acc_skp = m_acc_skp_V->value();

    p.m_pit_rimax = m_pit_rimax_V->value();
    p.m_pit_rkd = m_pit_rkd_V->value();
    p.m_pit_rki = m_pit_rki_V->value();
    p.m_pit_rkp = m_pit_rkp_V->value();
    p.m_pit_skp = m_pit_skp_V->value();

    p.m_rol_rimax = m_rol_rimax_V->value();
    p.m_rol_rkd = m_rol_rkd_V->value();
    p.m_rol_rki = m_rol_rki_V->value();
    p.m_rol_rkp = m_rol_rkp_V->value();
    p.m_rol_skp = m_rol_skp_V->value();

    p.m_thr_rimax = m_thr_rimax_V->value();
    p.m_thr_rkd = m_thr_rkd_V->value();
    p.m_thr_rki = m_thr_rki_V->value();
    p.m_thr_rkp = m_thr_rkp_V->value();
    p.m_thr_skp = m_thr_skp_V->value();

    p.m_yaw_rimax = m_yaw_rimax_V->value();
    p.m_yaw_rkd = m_yaw_rkd_V->value();
    p.m_yaw_rki = m_yaw_rki_V->value();
    p.m_yaw_rkp = m_yaw_rkp_V->value();
    p.m_yaw_skp = m_yaw_skp_V->value();

    return p;
}
