#include "mainwindow.h"
#include <QApplication>
#include <QHostAddress>



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.connectToHost("192.168.42.1", 7000);
    w.show();

    return a.exec();
}

