#ifndef SENSORCONTROL_H
#define SENSORCONTROL_H
#include <QMap>
#include <QList>
#include <QString>
#include <iterator>
#include <algorithm>

class SensorControl
{
public:

    static int initialize(void);
    static void terminate(void);
    static int readFromDxl(int id, int address);
    static void writeToDxl(int id, int address, int value);
    static int getModelNumber(int id);
    static int getVersionOfFirmware(int id);
    static int getID(int id);
    static void setID(int id, int newID);
    static int getBaudrate(int id);
    static void setBaudrate(int id, int newBaud);
    static int getReturnDelayTime(int id);
    static void setReturnDelayTime(int id, int newReturnDelayTime);
    static int getStatusReturnLevel(int id);
    static void setStatusReturnLevel(int id, int value);
    static int getIRLeftFireData(int id);
    static int getIRCenterFireData(int id);
    static int getIRRightFireData(int id);
    static int getLightLeftData(int id);
    static int getLightCenterData(int id);
    static int getLightRightData(int id);
    static int getIRObstacleDetected(int id);
    static int getLightDetected(int id);
    static int getSoundData(int id);
    static int getSoundDataMaxHold(int id);
    static void setSoundDataMaxHold(int id, int value);
    static int getSoundDetectedCount(int id);
    static void setSoundDetected(int id, int value);
    static int getSoundDetectedTime(int id);
    static void setSoundDetectedTime(int id, int value);
    static int getBuzzerData0(int id);
    static void setBuzzerData0(int id, int noteAddress);
    static int getBuzzerData1(int id);
    static void setBuzzerData1(int id, int value);
    static int getRegistered(int id);
    static void setRegistered(int id, int value);
    static int getIRRemoconArrived(int id);
    static int getLock(int id);
    static void setLock(int id, int value);
    static int getRemoconRXData(int id);
    static int getRemoconTXData(int id);
    static void setRemoconTXData(int id, int value);
    static int getIRObstacleDetectCompareRD(int id);
    static void setIRObstacleDetectCompareRD(int id, int value);
    static int getLightDetectCompareRD(int id);
    static void setLightDetectCompareRD(int id, int value);

    static int getCurrentBuzzerNote(int id);
    static void playBuzzerNote(int id, int noteAddress);
    static int getBuzzerRingingTime(int id);
    static void setBuzzerRingingTime(int id, int value);
    static void ResetSoundDataMaxHold(int id);

    private:

    static void writeByteToDxl(int id, int address, int value);
    static void writeWordToDxl(int id, int address, int value);
    static int readByteFromDxl(int id, int address);
    static int readWordFromDxl(int id, int address);

    static QMap<QString, int> createSensorDictionary(void);
    static QList<int> createSingleByteSensorAddresses(void);
    static bool isSingleByteSensorAddress(int address);


};

#endif // SENSORCONTROL_H
