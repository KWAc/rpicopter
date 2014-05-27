#include "trimprofile.h"
#include "cassert"


TrimProfile::TrimProfile(QSettings *pConf, QWidget *parent) : QWidget(parent) {
    m_pConf = pConf;
    setupUI();
}

void TrimProfile::setupUI() {
    this->setMinimumWidth(340);
    this->setWindowTitle("Trim profile manager");

    QPushButton *pButCreateNewProfile = new QPushButton("New");
    QPushButton *pButDeleteCurProfile = new QPushButton("Delete");
    QPushButton *pButOK = new QPushButton("OK");
    QPushButton *pButCancel = new QPushButton("Cancel");

    // Create layout
    m_pProfileSelector = new QComboBox;
    QGridLayout *m_pMainLayout = new QGridLayout(this);
    QLabel *pTitle = new QLabel("Profile list:");

    // Add widgets to layout
    m_pMainLayout->addWidget(pTitle, 0, 0);
    m_pMainLayout->addWidget(m_pProfileSelector, 1, 0);
    m_pMainLayout->addWidget(pButCreateNewProfile, 1, 1);
    m_pMainLayout->addWidget(pButDeleteCurProfile, 1, 2);
    m_pMainLayout->addWidget(pButOK, 2, 1);
    m_pMainLayout->addWidget(pButCancel, 2, 2);

    m_pMainLayout->setColumnStretch(0, 1);

    setLayout(m_pMainLayout);

    connect(pButCreateNewProfile, SIGNAL(pressed() ), this, SLOT(sl_createProfile() ) );
    connect(pButDeleteCurProfile, SIGNAL(pressed() ), this, SLOT(sl_deleteProfile() ) );

    connect(pButOK, SIGNAL(pressed() ), this, SLOT(sl_setProfile() ) );
    connect(pButOK, SIGNAL(pressed() ), this, SLOT(close() ) );
    connect(pButCancel, SIGNAL(pressed() ), this, SLOT(close() ) );

    connect(m_pProfileSelector, SIGNAL(currentIndexChanged(int) ), this, SLOT(sl_IndexChanged(int) ) );
}

void TrimProfile::saveConfig() {
    m_pConf->beginWriteArray("trims");

    assert(m_pProfileSelector->count() == m_lTrims.size() );
    for(int i = 0; i < m_pProfileSelector->count(); i++) {
        QString sName = m_pProfileSelector->itemText(i);

        m_pConf->setArrayIndex(i);
        m_pConf->setValue("name", sName);
        m_pConf->setValue("trimPitch", m_lTrims.at(i).second);
        m_pConf->setValue("trimRoll", m_lTrims.at(i).first);
    }
    m_pConf->endArray();
}

void TrimProfile::sl_savePIDs(PIDS p) {
    m_pConf->beginWriteArray("trims");
    m_pConf->setArrayIndex(m_pProfileSelector->currentIndex() );

    m_pConf->setValue("p_rkp", p.m_pit_rkp);
    m_pConf->setValue("p_rki", p.m_pit_rki);
    m_pConf->setValue("p_rkd", p.m_pit_rkd);
    m_pConf->setValue("p_rimax", p.m_pit_rimax);

    m_pConf->setValue("r_rkp", p.m_rol_rkp);
    m_pConf->setValue("r_rki", p.m_rol_rki);
    m_pConf->setValue("r_rkd", p.m_rol_rkd);
    m_pConf->setValue("r_rimax", p.m_rol_rimax);

    m_pConf->setValue("y_rkp", p.m_yaw_rkp);
    m_pConf->setValue("y_rki", p.m_yaw_rki);
    m_pConf->setValue("y_rkd", p.m_yaw_rkd);
    m_pConf->setValue("y_rimax", p.m_yaw_rimax);

    m_pConf->setValue("t_rkp", p.m_thr_rkp);
    m_pConf->setValue("t_rki", p.m_thr_rki);
    m_pConf->setValue("t_rkd", p.m_thr_rkd);
    m_pConf->setValue("t_rimax", p.m_thr_rimax);

    m_pConf->setValue("a_rkp", p.m_acc_skp);
    m_pConf->setValue("a_rki", p.m_acc_rki);
    m_pConf->setValue("a_rkd", p.m_acc_rkd);
    m_pConf->setValue("a_rimax", p.m_acc_rimax);

    m_pConf->setValue("p_skp", p.m_pit_skp);
    m_pConf->setValue("r_skp", p.m_rol_skp);
    m_pConf->setValue("y_skp", p.m_yaw_skp);
    m_pConf->setValue("t_skp", p.m_thr_skp);
    m_pConf->setValue("a_skp", p.m_acc_skp);

    m_pConf->endArray();
}


PIDS TrimProfile::loadPIDs() {
    PIDS p;

    m_pConf->beginReadArray("trims");
    m_pConf->setArrayIndex(m_pProfileSelector->currentIndex() );

    p.m_pit_rkp = m_pConf->value("p_rkp").toDouble();
    p.m_pit_rki = m_pConf->value("p_rki").toDouble();
    p.m_pit_rkd = m_pConf->value("p_rkd").toDouble();
    p.m_pit_rimax = m_pConf->value("p_rimax").toDouble();

    p.m_rol_rkp = m_pConf->value("r_rkp").toDouble();
    p.m_rol_rki = m_pConf->value("r_rki").toDouble();
    p.m_rol_rkd = m_pConf->value("r_rkd").toDouble();
    p.m_rol_rimax = m_pConf->value("r_rimax").toDouble();

    p.m_yaw_rkp = m_pConf->value("y_rkp").toDouble();
    p.m_yaw_rki = m_pConf->value("y_rki").toDouble();
    p.m_yaw_rkd = m_pConf->value("y_rkd").toDouble();
    p.m_yaw_rimax = m_pConf->value("y_rimax").toDouble();

    p.m_thr_rkp = m_pConf->value("t_rkp").toDouble();
    p.m_thr_rki = m_pConf->value("t_rki").toDouble();
    p.m_thr_rkd = m_pConf->value("t_rkd").toDouble();
    p.m_thr_rimax = m_pConf->value("t_rimax").toDouble();

    p.m_acc_skp = m_pConf->value("a_rkp").toDouble();
    p.m_acc_rki = m_pConf->value("a_rki").toDouble();
    p.m_acc_rkd = m_pConf->value("a_rkd").toDouble();
    p.m_acc_rimax = m_pConf->value("a_rimax").toDouble();

    p.m_pit_skp = m_pConf->value("p_skp").toDouble();
    p.m_rol_skp = m_pConf->value("r_skp").toDouble();
    p.m_yaw_skp = m_pConf->value("y_skp").toDouble();
    p.m_thr_skp = m_pConf->value("t_skp").toDouble();
    p.m_acc_skp = m_pConf->value("a_skp").toDouble();

    m_pConf->endArray();

    return p;
}
void TrimProfile::createDefaultProfile() {
    QString sName = "default";

    m_pConf->beginWriteArray("trims");
    m_pConf->setArrayIndex(0);
    m_pConf->setValue("name", sName);
    m_pConf->setValue("trimPitch", 0.f);
    m_pConf->setValue("trimRoll", 0.f);
    m_pConf->endArray();

    m_pProfileSelector->addItem(sName);
    m_lTrims.append(trim(0.f, 0.f) );
}

void TrimProfile::loadConfig() {
    int size = m_pConf->beginReadArray("trims");
    m_pProfileSelector->clear();
    for(int i = 0; i < size; i++) {
        m_pConf->setArrayIndex(i);
        QString sName = m_pConf->value("name").toString();

        m_pProfileSelector->addItem(sName);
        m_lTrims.append(trim(m_pConf->value("trimRoll").toDouble(), m_pConf->value("trimPitch").toDouble()) );
    }
    m_pConf->endArray();

    if(!size) {
        createDefaultProfile();
    }

    // Load last active profile
    m_pProfileSelector->setCurrentIndex(m_pConf->value("pmanager_lindex", 0).toInt() );

    emitCurrentIndex();
}

void TrimProfile::sl_createProfile() {
    bool ok;
    QString profile = QInputDialog::getText(this, tr("Enter profile name"),
                                         tr("Profile name:"), QLineEdit::Normal,
                                         QString(), &ok);
    if (ok && !profile.isEmpty()) {
        m_lTrims.append(trim(0.f, 0.f) );
        m_pProfileSelector->addItem(profile);
        saveConfig();
    }
}

void TrimProfile::sl_IndexChanged(int index) {
    m_pConf->setValue("pmanager_lindex", index);
}

void TrimProfile::sl_setProfile() {
    assert(m_pProfileSelector->count() == m_lTrims.size() );

    float fRoll = m_lTrims.at(m_pProfileSelector->currentIndex() ).first;
    float fPitch = m_lTrims.at(m_pProfileSelector->currentIndex() ).second;
    emit si_trimChanged(fRoll, fPitch);
}

void TrimProfile::sl_updateTrim(float roll, float pitch) {
    int index = m_pProfileSelector->currentIndex();
    m_lTrims[index].first = roll;
    m_lTrims[index].second = pitch;
    saveConfig();
}

void TrimProfile::emitCurrentIndex() {
    assert(m_pProfileSelector->count() == m_lTrims.size() );
    float fRoll = m_lTrims.at(m_pProfileSelector->currentIndex() ).first;
    float fPitch = m_lTrims.at(m_pProfileSelector->currentIndex() ).second;
    emit si_trimChanged(fRoll, fPitch);
}

void TrimProfile::sl_deleteProfile() {
    int index = m_pProfileSelector->currentIndex();
    m_pProfileSelector->removeItem(index);
    m_lTrims.removeAt(index);

    // Remove the proper elements from config file
    m_pConf->beginWriteArray("trims");
    m_pConf->setArrayIndex(index);
    m_pConf->remove("name");
    m_pConf->remove("trimRoll");
    m_pConf->remove("trimPitch");
    m_pConf->endArray();

    // If all profiles were deleted
    // Create default
    if(!m_pProfileSelector->count() ) {
        createDefaultProfile();
    }

    // Save new config
    saveConfig();
    emitCurrentIndex();
}
