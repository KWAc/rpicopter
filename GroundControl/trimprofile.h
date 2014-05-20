#ifndef TRIMPROFILE_H
#define TRIMPROFILE_H

#include <QtWidgets>


typedef QPair<double, double> trim; // roll, pitch
typedef QList<trim> trims;

class TrimProfile : public QWidget
{
Q_OBJECT
private:
    QSettings *m_pConf;
    QComboBox *m_pProfileSelector;

    void setupUI();
    void createDefaultProfile();
    void emitCurrentIndex();

    trims m_lTrims;

private slots:
    void sl_IndexChanged(int index);
    void sl_setProfile();

public:
    TrimProfile(QSettings *pConf, QWidget *parent = NULL);
    ~TrimProfile() {};

    void loadConfig();
    void saveConfig();

public slots:
    void sl_createProfile();
    void sl_deleteProfile();
    void sl_updateTrim(float roll, float pitch);

signals:
    void si_trimChanged(float roll, float pitch);
};

#endif // TRIMPROFILE_H
