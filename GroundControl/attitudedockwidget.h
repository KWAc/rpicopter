#ifndef Q_ATTITUDEDOCK
#define Q_ATTITUDEDOCK

#include <QtWidgets>
#include "attitudeindicator.h"



class QAttitudeDockWidget : public QDockWidget
{
private:
    qAttitudeIndicator m_AttitudeIndicator;
    QLabel m_AttitudeLabel;
    QGroupBox *m_pGroupBox;
    QVBoxLayout *m_pLayout;

    void Setup() {
        m_pGroupBox = new QGroupBox(this);
        m_pLayout = new QVBoxLayout(m_pGroupBox);
        m_pLayout->addWidget(&m_AttitudeLabel);
        m_pLayout->addWidget(&m_AttitudeIndicator);

        m_pGroupBox->setLayout(m_pLayout);
    
        this->setWidget(m_pGroupBox);
        this->setMaximumWidth(232);
    }

public:
    QAttitudeDockWidget(const QString & title, QWidget * parent = 0, Qt::WindowFlags flags = 0) : QDockWidget(title, parent, flags) {
        Setup();
    }

    QAttitudeDockWidget (QWidget * parent = 0, Qt::WindowFlags flags = 0 )  : QDockWidget(parent, flags) {
        Setup();
    }

    qAttitudeIndicator *GetIndicator() {
        return &m_AttitudeIndicator;
    }
    
    void SetAttitude(QString text) {
      m_AttitudeLabel.setText(text);
    }
};

#endif
