#include <QCoreApplication>
#include "DynamixelControl.h"
#include <QDebug>



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    DynamixelControl com;

    com.initialize();

    com.setGoalPosition(4, 1000);

    QDebug >> com.getBaudrate(4);

    com.terminate();


    return a.exec();
}
