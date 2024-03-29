#-------------------------------------------------
#
# Project created by QtCreator 2014-01-19T11:33:19
#
#-------------------------------------------------

QT      += serialport core gui network printsupport webkitwidgets
CONFIG  += console

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET   =  GroundControl
TEMPLATE =  app

SOURCES +=  main.cpp\
            mainwindow.cpp \
            qcustomplot.cpp \
            attitudeindicator.cpp \
            piddockwidget.cpp \
            piddialog.cpp\
            rcwidget.cpp \
            absframe.cpp \
            container.cpp\
            gmaps.cpp \
            trimprofile.cpp

HEADERS  += mainwindow.h\
            qcustomplot.h \
            plotdockwidget.h \
            attitudeindicator.h \
            attitudedockwidget.h \
            rcwidget.h \
            logdockwidget.h \
            piddockwidget.h \
            piddialog.h \
            absframe.h \
            container.h \
            gmaps.h \
            trimprofile.h

linux:LIBS += -lsfml-window \

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../SFML-2.1/lib/ -lsfml-window
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../SFML-2.1/lib/ -lsfml-window

INCLUDEPATH += $$PWD/../../../../../SFML-2.1/include
DEPENDPATH += $$PWD/../../../../../SFML-2.1/include
