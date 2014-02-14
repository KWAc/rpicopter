#include "qpiddialog.h"


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
    m_pit_rimax_V->setDisabled(false);
    m_rol_rkp_V->setDisabled(false);
    m_rol_rki_V->setDisabled(false);
    m_rol_rimax_V->setDisabled(false);
    m_yaw_rkp_V->setDisabled(false);
    m_yaw_rki_V->setDisabled(false);
    m_yaw_rimax_V->setDisabled(false);
    m_pit_skp_V->setDisabled(false);
    m_rol_skp_V->setDisabled(false);
    m_yaw_skp_V->setDisabled(false);
}

void QPIDConfig::sl_Deactivate() {
    m_bSend = false;
    m_pit_rkp_V->setDisabled(true);
    m_pit_rki_V->setDisabled(true);
    m_pit_rimax_V->setDisabled(true);
    m_rol_rkp_V->setDisabled(true);
    m_rol_rki_V->setDisabled(true);
    m_rol_rimax_V->setDisabled(true);
    m_yaw_rkp_V->setDisabled(true);
    m_yaw_rki_V->setDisabled(true);
    m_yaw_rimax_V->setDisabled(true);
    m_pit_skp_V->setDisabled(true);
    m_rol_skp_V->setDisabled(true);
    m_yaw_skp_V->setDisabled(true);
}

void QPIDConfig::sl_setPIDs(QVariantMap map) {
    if(this->isVisible() )
        return;

    m_pit_rkp_V->setValue(map["pit_rkp"].toDouble() );
    m_pit_rki_V->setValue(map["pit_rki"].toDouble() );
    m_pit_rimax_V->setValue(map["pit_rimax"].toDouble() );
    m_rol_rkp_V->setValue(map["rol_rkp"].toDouble() );
    m_rol_rki_V->setValue(map["rol_rki"].toDouble() );
    m_rol_rimax_V->setValue(map["rol_rimax"].toDouble() );
    m_yaw_rkp_V->setValue(map["yaw_rkp"].toDouble() );
    m_yaw_rki_V->setValue(map["yaw_rki"].toDouble() );
    m_yaw_rimax_V->setValue(map["yaw_rimax"].toDouble() );
    m_pit_skp_V->setValue(map["pit_skp"].toDouble() );
    m_rol_skp_V->setValue(map["rol_skp"].toDouble() );
    m_yaw_skp_V->setValue(map["yaw_skp"].toDouble() );
}

void QPIDConfig::Setup() {
    s_pit_rkp = "Pitch Rkp: ",  s_pit_rki = "Pitch Rki: ",  s_pit_rimax = "Pitch RImax: ";
    s_rol_rkp = "Roll Rkp: ",   s_rol_rki = "Roll Rki: ",   s_rol_rimax = "Roll RImax: ";
    s_yaw_rkp = "Yaw Rkp: ",    s_yaw_rki = "Yaw Rki: ",    s_yaw_rimax = "Yaw RImax: ";
    s_pit_skp = "Pitch Skp: ",  s_rol_skp = "Roll Skp: ",   s_yaw_skp = "Yaw Skp: ";

    m_pit_rkp = new QLabel(s_pit_rkp);
    m_pit_rki = new QLabel(s_pit_rki);
    m_pit_rimax = new QLabel(s_pit_rimax);
    m_rol_rkp = new QLabel(s_rol_rkp);
    m_rol_rki = new QLabel(s_rol_rki);
    m_rol_rimax = new QLabel(s_rol_rimax);
    m_yaw_rkp = new QLabel(s_yaw_rkp);
    m_yaw_rki = new QLabel(s_yaw_rki);
    m_yaw_rimax = new QLabel(s_yaw_rimax);
    m_pit_skp = new QLabel(s_pit_skp);
    m_rol_skp = new QLabel(s_rol_skp);
    m_yaw_skp = new QLabel(s_yaw_skp);

    m_pit_rkp_V = new QDoubleSpinBox();
    m_pit_rkp_V->setRange(0, 25);
    m_pit_rki_V = new QDoubleSpinBox();
    m_pit_rki_V->setRange(0, 25);
    m_pit_rimax_V = new QDoubleSpinBox();
    m_pit_rimax_V->setRange(0, 100);
    m_rol_rkp_V = new QDoubleSpinBox();
    m_rol_rkp_V->setRange(0, 25);
    m_rol_rki_V = new QDoubleSpinBox();
    m_rol_rki_V->setRange(0, 25);
    m_rol_rimax_V = new QDoubleSpinBox();
    m_rol_rimax_V->setRange(0, 100);
    m_yaw_rkp_V = new QDoubleSpinBox();
    m_yaw_rkp_V->setRange(0, 25);
    m_yaw_rki_V = new QDoubleSpinBox();
    m_yaw_rki_V->setRange(0, 25);
    m_yaw_rimax_V = new QDoubleSpinBox();
    m_yaw_rimax_V->setRange(0, 100);
    m_pit_skp_V = new QDoubleSpinBox();
    m_pit_skp_V->setRange(0, 25);
    m_rol_skp_V = new QDoubleSpinBox();
    m_rol_skp_V->setRange(0, 25);
    m_yaw_skp_V = new QDoubleSpinBox();
    m_yaw_skp_V->setRange(0, 25);

    m_pit_rkp_V->setSingleStep(0.1);
    m_pit_rki_V->setSingleStep(0.1);
    m_pit_rimax_V->setSingleStep(0.1);
    m_rol_rkp_V->setSingleStep(0.1);
    m_rol_rki_V->setSingleStep(0.1);
    m_rol_rimax_V->setSingleStep(0.1);
    m_yaw_rkp_V->setSingleStep(0.1);
    m_yaw_rki_V->setSingleStep(0.1);
    m_yaw_rimax_V->setSingleStep(0.1);
    m_pit_skp_V->setSingleStep(0.1);
    m_rol_skp_V->setSingleStep(0.1);
    m_yaw_skp_V->setSingleStep(0.1);

    m_pit_rkp_V->setValue(0);
    m_pit_rki_V->setValue(0);
    m_pit_rimax_V->setValue(0);
    m_rol_rkp_V->setValue(0);
    m_rol_rki_V->setValue(0);
    m_rol_rimax_V->setValue(0);
    m_yaw_rkp_V->setValue(0);
    m_yaw_rki_V->setValue(0);
    m_yaw_rimax_V->setValue(0);
    m_pit_skp_V->setValue(0);
    m_rol_skp_V->setValue(0);
    m_yaw_skp_V->setValue(0);

    QGridLayout *pLayout = new QGridLayout(this);
    pLayout->setHorizontalSpacing(8);

    pLayout->addWidget(m_pit_rkp, 0, 0);
    pLayout->addWidget(m_pit_rki, 0, 2);
    pLayout->addWidget(m_pit_rimax, 0, 4);
    pLayout->addWidget(m_rol_rkp, 1, 0);
    pLayout->addWidget(m_rol_rki, 1, 2);
    pLayout->addWidget(m_rol_rimax, 1, 4);
    pLayout->addWidget(m_yaw_rkp, 2, 0);
    pLayout->addWidget(m_yaw_rki, 2, 2);
    pLayout->addWidget(m_yaw_rimax, 2, 4);
    pLayout->addWidget(m_pit_skp, 3, 0);
    pLayout->addWidget(m_rol_skp, 3, 2);
    pLayout->addWidget(m_yaw_skp, 3, 4);

    pLayout->addWidget(m_pit_rkp_V, 0, 1);
    pLayout->addWidget(m_pit_rki_V, 0, 3);
    pLayout->addWidget(m_pit_rimax_V, 0, 5);
    pLayout->addWidget(m_rol_rkp_V, 1, 1);
    pLayout->addWidget(m_rol_rki_V, 1, 3);
    pLayout->addWidget(m_rol_rimax_V, 1, 5);
    pLayout->addWidget(m_yaw_rkp_V, 2, 1);
    pLayout->addWidget(m_yaw_rki_V, 2, 3);
    pLayout->addWidget(m_yaw_rimax_V, 2, 5);
    pLayout->addWidget(m_pit_skp_V, 3, 1);
    pLayout->addWidget(m_rol_skp_V, 3, 3);
    pLayout->addWidget(m_yaw_skp_V, 3, 5);

    m_pButOK = new QPushButton(tr("OK") );
    m_pButCancel = new QPushButton(tr("Cancel"));
    pLayout->addWidget(m_pButOK, 4, 4);
    pLayout->addWidget(m_pButCancel, 4, 5);

    connect(m_pButCancel, SIGNAL(pressed() ), this, SLOT(close() ) );
    connect(m_pButOK, SIGNAL(pressed() ), this, SLOT(close() ) );
    connect(m_pButOK, SIGNAL(pressed() ), this, SLOT(sl_sendPIDs() ) );

    this->setLayout(pLayout);
}

void QPIDConfig::sl_sendPIDs() {
    if(!m_bSend){
        qDebug() << "PIDs not emitted";
        return;
    }

    QString com = "";
    QTextStream stream(&com);
    stream  << "{\"type\":\"pid\",\"pit_rkp\":" << m_pit_rkp_V->value() << ",\"pit_rki\":" << m_pit_rki_V->value() << ",\"pit_rimax\":" << m_pit_rimax_V->value()
                             << ",\"rol_rkp\":" << m_rol_rkp_V->value() << ",\"rol_rki\":" << m_rol_rki_V->value() << ",\"rol_rimax\":" << m_rol_rimax_V->value()
                             << ",\"yaw_rkp\":" << m_yaw_rkp_V->value() << ",\"yaw_rki\":" << m_yaw_rki_V->value() << ",\"yaw_rimax\":" << m_yaw_rimax_V->value()
                             << ",\"pit_skp\":" << m_pit_skp_V->value() << ",\"rol_skp\":" << m_rol_skp_V->value() << ",\"yaw_skp\":" << m_yaw_skp_V->value() << "}";

    for(int i = 0; i < 16; i++)
        m_pUdpSock->write(com.toLocal8Bit(), com.length() );
    qDebug() << com;
}
