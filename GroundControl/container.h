#ifndef CONTAINER_h
#define CONTAINER_h

#include <QtWidgets>


struct PIDS {
    double m_pit_rkp, m_pit_rki, m_pit_rkd, m_pit_rimax;
    double m_rol_rkp, m_rol_rki, m_rol_rkd, m_rol_rimax;
    double m_yaw_rkp, m_yaw_rki, m_yaw_rkd, m_yaw_rimax;
    double m_thr_rkp, m_thr_rki, m_thr_rkd, m_thr_rimax;
    double m_acc_rkp, m_acc_rki, m_acc_rkd, m_acc_rimax;
    double m_pit_skp, m_rol_skp, m_yaw_skp, m_thr_skp, m_acc_skp;
};

struct RANGE {
    void setF1 ();
    void setF2 ();
    void setF3 ();
    void setF4 ();

    RANGE();

    int ROL_MIN;
    int ROL_MAX;

    int PIT_MIN;
    int PIT_MAX;

    int YAW_MIN;
    int YAW_MAX;

    int THR_MIN;
    int THR_MAX;

    int THR_80P;
};

class RC_COM {
private:
    char    m_cRadioCommand[8];
    char    m_cWiFiCommand[512];
public:
    RC_COM();

    float ROL;
    float PIT;
    float YAW;
    float THR;

    int calc_chksum(char *str);

    QString str_makeWiFiCommand();
    QPair<int, char*> cstr_makeRadioCommand ();
};

class DRIFT_CAL {
private:
    char    m_cWiFiCommand[512];

public:
    DRIFT_CAL();
    
    float ROL;
    float PIT;

    QString str_makeWiFiCommand();
    QPair<int, char*> cstr_makeWiFiCommand ();
};

#endif
