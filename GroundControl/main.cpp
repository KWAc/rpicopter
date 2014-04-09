#include "mainwindow.h"
#include <QApplication>
#include <QHostAddress>
#include <QSettings>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QSettings config("config.ini", QSettings::IniFormat);
    QString sHostIP = config.value("HostIP", "192.168.42.1").toString();
    int iHostPort = config.value("HostPort", 7000).toInt();

    qDebug() << "Host IP: " << sHostIP << " and port: " << iHostPort;
    
    if(!config.contains("HostIP") )
      config.setValue("HostIP", sHostIP);
    if(!config.contains("HostPort") )
      config.setValue("HostPort", iHostPort);

    MainWindow w(&config);
    w.connectToHost(sHostIP, iHostPort);
    w.show();

    return a.exec();
}

