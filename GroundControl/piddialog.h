#ifndef Q_PIDDIAL
#define Q_PIDDIAL

#include <QtWidgets>
#include <QUdpSocket>


class QPIDConfig : public QWidget {
Q_OBJECT
private:
    bool m_bSend;

    QUdpSocket *m_pUdpSock;

    QPushButton *m_pButOK;
    QPushButton *m_pButCancel;

    QLabel *m_pit_rkp, *m_pit_rki, *m_pit_rkd, *m_pit_rimax;
    QLabel *m_rol_rkp, *m_rol_rki, *m_rol_rkd, *m_rol_rimax;
    QLabel *m_yaw_rkp, *m_yaw_rki, *m_yaw_rkd, *m_yaw_rimax;
    QLabel *m_thr_rkp, *m_thr_rki, *m_thr_rkd, *m_thr_rimax;
    QLabel *m_thr_akp, *m_thr_aki, *m_thr_akd, *m_thr_aimax;
    QLabel *m_pit_skp, *m_rol_skp, *m_yaw_skp, *m_thr_skp, *m_acc_skp;

    QDoubleSpinBox *m_pit_rkp_V, *m_pit_rki_V, *m_pit_rkd_V, *m_pit_rimax_V;
    QDoubleSpinBox *m_rol_rkp_V, *m_rol_rki_V, *m_rol_rkd_V, *m_rol_rimax_V;
    QDoubleSpinBox *m_yaw_rkp_V, *m_yaw_rki_V, *m_yaw_rkd_V, *m_yaw_rimax_V;
    QDoubleSpinBox *m_thr_rkp_V, *m_thr_rki_V, *m_thr_rkd_V, *m_thr_rimax_V;
    QDoubleSpinBox *m_acc_rkp_V, *m_acc_rki_V, *m_acc_rkd_V, *m_acc_rimax_V;
    QDoubleSpinBox *m_pit_skp_V, *m_rol_skp_V, *m_yaw_skp_V, *m_thr_skp_V, *m_acc_skp_V;

    void Setup();

    QString s_pit_rkp, s_pit_rki, s_pit_rkd, s_pit_rimax;
    QString s_rol_rkp, s_rol_rki, s_rol_rkd, s_rol_rimax;
    QString s_yaw_rkp, s_yaw_rki, s_yaw_rkd, s_yaw_rimax;
    QString s_thr_rkp, s_thr_rki, s_thr_rkd, s_thr_rimax;
    QString s_acc_rkp, s_acc_rki, s_acc_rkd, s_acc_rimax;
    QString s_pit_skp, s_rol_skp, s_yaw_skp, s_thr_skp, s_acc_skp;

public:
    QPIDConfig(QUdpSocket *pSock, QWidget *parent = NULL);

public slots:
    void sl_Activate();
    void sl_Deactivate();
    void sl_setPIDs(QVariantMap map);
    void sl_sendPIDs();
};


#endif
