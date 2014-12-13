#include <QCoreApplication>
#include "ActuatorControl.h"
#include "sensorcontrol.h"
#include <QDebug>



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //ActuatorControl com;
    //com.initialize();
    SensorControl sen;
    sen.initialize();

    //com.setGoalPosition(4, 200);
    qDebug() << sen.getIRLeftFireData(100);




//        com.toggleJointMode(4,0

    sen.terminate();


    return a.exec();
}
