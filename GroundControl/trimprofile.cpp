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

void TrimProfile::createDefaultProfile() {
    QString sName = "default";

    m_pConf->setValue("name", sName);
    m_pConf->setValue("trimPitch", 0.f);
    m_pConf->setValue("trimRoll", 0.f);

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
    }
}

void TrimProfile::sl_IndexChanged(int index) {
    assert(m_pProfileSelector->count() == m_lTrims.size() );

    float fRoll = m_lTrims.at(index).first;
    float fPitch = m_lTrims.at(index).second;
    emit si_trimChanged(fRoll, fPitch);
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
