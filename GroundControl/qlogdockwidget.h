#ifndef Q_LOGDOCK
#define Q_LOGDOCK

#include <QtGui>



class QLogDockWidget : public QDockWidget
{
private:
    QTextEdit m_TextLog;

    void Setup() {
        this->setWidget(&m_TextLog);
        this->setMinimumSize(320, 240);
        m_TextLog.setTextInteractionFlags(Qt::NoTextInteraction);
    }


public:
    QLogDockWidget(const QString & title, QWidget * parent = 0, Qt::WindowFlags flags = 0) : QDockWidget(title, parent, flags) {
        Setup();
    }

    QLogDockWidget (QWidget * parent = 0, Qt::WindowFlags flags = 0 )  : QDockWidget(parent, flags) {
        Setup();
    }

    QTextEdit *getTextEdit() {
        return &m_TextLog;
    }
};

#endif
