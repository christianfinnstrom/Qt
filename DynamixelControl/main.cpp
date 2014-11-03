#include <QCoreApplication>
#include "DynamixelControl.h"
#include <QDebug>



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    DynamixelControl com;

    com.initialize();

    com.setGoalPosition(4, 720);

    com.terminate();


    return a.exec();
}
