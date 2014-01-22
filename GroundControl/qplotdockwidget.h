#ifndef PLOTDOCK_H
#define PLOTDOCK_H

#include <QtGui>
#include "qcustomplot.h"



class QPlotDockWidget : public QDockWidget
{
private:
    QCustomPlot m_PlotWidget;

    void Setup(const unsigned int &graphs) {
        for(unsigned int i = 0; i < graphs; i++) {
            m_PlotWidget.addGraph();
        }
        this->setWidget(&m_PlotWidget);
    }

    void initWidget() {
        this->setMinimumSize(320, 160);
    }

public:
    QPlotDockWidget(const QString & title, const unsigned int &graphs, QWidget * parent = 0, Qt::WindowFlags flags = 0) : QDockWidget(title, parent, flags) {
        Setup(graphs);
        initWidget();
    }

    QPlotDockWidget (const unsigned int &graphs, QWidget * parent = 0, Qt::WindowFlags flags = 0 )  : QDockWidget(parent, flags) {
        Setup(graphs);
        initWidget();
    }

    QCustomPlot *GetGraph() {
        return &m_PlotWidget;
    }
};

#endif
