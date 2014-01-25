#include "qpiddockwidget.h"
#include <qjson/parser.h>


QPIDDockWidget::QPIDDockWidget(const QString & title, QWidget * parent, Qt::WindowFlags flags) : QDockWidget(title, parent, flags) {
    Setup();
}

QPIDDockWidget::QPIDDockWidget(QWidget * parent, Qt::WindowFlags flags )  : QDockWidget(parent, flags) {
    Setup();
}

void QPIDDockWidget::sl_setPIDs(QVariantMap map) {
    m_pit_rkp_V->setText(map["pit_rkp"].toString() );
    m_pit_rki_V->setText(map["pit_rki"].toString() );
    m_pit_rimax_V->setText(map["pit_rimax"].toString() );
    m_rol_rkp_V->setText(map["rol_rkp"].toString() );
    m_rol_rki_V->setText(map["rol_rki"].toString() );
    m_rol_rimax_V->setText(map["rol_rimax"].toString() );
    m_yaw_rkp_V->setText(map["yaw_rkp"].toString() );
    m_yaw_rki_V->setText(map["yaw_rki"].toString() );
    m_yaw_rimax_V->setText(map["yaw_rimax"].toString() );
    m_pit_skp_V->setText(map["pit_skp"].toString() );
    m_rol_skp_V->setText(map["rol_skp"].toString() );
    m_yaw_skp_V->setText(map["yaw_skp"].toString() );
}

void QPIDDockWidget::Setup() {
    this->setWidget(&m_PIDs);

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

    m_pit_rkp_V = new QLabel("n.a.");
    m_pit_rki_V = new QLabel("n.a.");
    m_pit_rimax_V = new QLabel("n.a.");
    m_rol_rkp_V = new QLabel("n.a.");
    m_rol_rki_V = new QLabel("n.a.");
    m_rol_rimax_V = new QLabel("n.a.");
    m_yaw_rkp_V = new QLabel("n.a.");
    m_yaw_rki_V = new QLabel("n.a.");
    m_yaw_rimax_V = new QLabel("n.a.");
    m_pit_skp_V = new QLabel("n.a.");
    m_rol_skp_V = new QLabel("n.a.");
    m_yaw_skp_V = new QLabel("n.a.");

    QGridLayout *pLayout = new QGridLayout(&m_PIDs);
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

    m_PIDs.setLayout(pLayout);
}
