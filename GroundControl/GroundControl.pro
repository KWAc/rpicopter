#-------------------------------------------------
#
# Project created by QtCreator 2014-01-19T11:33:19
#
#-------------------------------------------------

QT          += core gui network serialport printsupport

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET   =  GroundControl
TEMPLATE =  app

SOURCES +=  main.cpp\
            mainwindow.cpp \
            qcustomplot.cpp \
            qattitudeindicator.cpp \
            qpiddockwidget.cpp \
            qpiddialog.cpp\
            qrcwidget.cpp \

HEADERS  += mainwindow.h\
            qcustomplot.h \
            qplotdockwidget.h \
            qattitudeindicator.h \
            qattitudedockwidget.h \
            qrcwidget.h \
            qlogdockwidget.h \
            qpiddockwidget.h \
            qpiddialog.h
