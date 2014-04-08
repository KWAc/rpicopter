#ifndef CONTAINER_h
#define CONTAINER_h

#include <QtWidgets>


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
