#ifndef Q_PIDDOCK
#define Q_PIDDOCK

#include <QtWidgets>


class QPIDDockWidget : public QDockWidget
{
Q_OBJECT
private:
    QWidget m_PIDs;
    
    QLabel *m_pit_rkp, *m_pit_rki, *m_pit_rimax;
    QLabel *m_rol_rkp, *m_rol_rki, *m_rol_rimax;
    QLabel *m_yaw_rkp, *m_yaw_rki, *m_yaw_rimax;
    QLabel *m_pit_skp, *m_rol_skp, *m_yaw_skp;

    QLabel *m_pit_rkp_V, *m_pit_rki_V, *m_pit_rimax_V;
    QLabel *m_rol_rkp_V, *m_rol_rki_V, *m_rol_rimax_V;
    QLabel *m_yaw_rkp_V, *m_yaw_rki_V, *m_yaw_rimax_V;
    QLabel *m_pit_skp_V, *m_rol_skp_V, *m_yaw_skp_V;

    void Setup();

    QString s_pit_rkp, s_pit_rki, s_pit_rimax;
    QString s_rol_rkp, s_rol_rki, s_rol_rimax;
    QString s_yaw_rkp, s_yaw_rki, s_yaw_rimax;
    QString s_pit_skp, s_rol_skp, s_yaw_skp;

public:
    QPIDDockWidget(const QString & title, QWidget * parent = 0, Qt::WindowFlags flags = 0);
    QPIDDockWidget(QWidget * parent = 0, Qt::WindowFlags flags = 0 );

    QWidget *getPIDFrame() {
        return &m_PIDs;
    }

public slots:
    void sl_setPIDs(QVariantMap map);
};

#endif
