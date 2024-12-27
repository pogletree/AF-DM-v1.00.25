#include "mainwindow.h"
#include "Cm110s.h"
#include <QtCore>
#include <QApplication>
#include <QStyleFactory>



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("Windows"));
    QThread::currentThread()->setPriority(QThread::HighPriority);
    MainWindow w;
    w.show();
    QString param(argv[1]);
    return a.exec();
}
