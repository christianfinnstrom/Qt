#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <QMap>
#include <QList>
#include <QString>
#include <iterator>
#include <algorithm>



class ActuatorControl
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
    static int getCWAngleLimit(int id);
    static void setCWAngleLimit(int id, int newCWAngleLimit);
    static int getCCWAngleLimit(int id);
    static void setCCWAngleLimit(int id, int newCCWAngleLimit);
    static int getTheHighestLimitTemperature(int id);
    static void setTheHighestLimitTemperature(int id, int value);
    static int getTheLowestLimitVoltage(int id);
    static void setTheLowestLimitVoltage(int id, int value);
    static int getTheHighestLimitVoltage(int id);
    static void setTheHighestLimitVoltage(int id, int value);
    static int getMaxTorque(int id);
    static void setMaxTorque(int id, int value);
    static int getStatusReturnLevel(int id);
    static void setStatusReturnLevel(int id, int value);
    static int getAlarmLED(int id);
    static void setAlarmLED(int id, int value);
    static int getAlarmShutdown(int id);
    static void setAlarmShutdown(int id, int value);
    static int getTorqueEnable(int id);
    static void setTorqueEnable(int id, int value);
    static int getLED(int id);
    static void setLED(int id, int value);
    static int getCWComplianceMargin(int id);
    static void setCWComplianceMargin(int id, int value);
    static int getCCWComplianceMargin(int id);
    static void setCCWComplianceMargin(int id, int value);
    static int getCWComplianceSlope(int id);
    static void setCWComplianceSlope(int id, int value);
    static int getCCWComplianceSlope(int id);
    static void setCCWComplianceSlope(int id, int value);
    static int getGoalPosition(int id);
    static void setGoalPosition(int id, int value);
    static int getMovingSpeed(int id);
    static void setMovingSpeed(int id, int value);
    static int getTorqueLimit(int id);
    static void setTorqueLimit(int id, int value);
    static int getPresentPosition(int id);
    static int getPresentSpeed(int id);
    static int getPresentVoltage(int id);
    static int getPresentTemperature(int id);
    static int getRegistered(int id);
    static int getMoving(int id);
    static int getLock(int id);
    static void setLock(int id, int value);
    static int getPunch(int id);
    static void setPunch(int id, int value);
    static void torqueEnableSwitch(int id);
    static bool isInstructionRegistered(int id);
    static bool isMoving(int id);
    static int getPresentLoad(int id);
    static bool isEEPROMLocked(int id);
    static void toggleWheelMode(int id);
    static void toggleJointMode(int id, int newCWAngleLimit, int newCCWAngleLimit);
    static int getGoalPositionAngular(int id);
    static void setGoalPositionAngular(int id, int angularPosition);
    static int getPresentPositionAngular(int id);
    static int getMovementMode(int id);

private:
    static void writeByteToDxl(int id, int address, int value);
    static void writeWordToDxl(int id, int address, int value);
    static int readByteFromDxl(int id, int address);
    static int readWordFromDxl(int id, int address);

    static int angularValueFromDxlValue(int value);
    static int angularValueToDxlValue(int value);

    static QMap<QString, int> createDictionary(void);
    static QList<int> createSingleByteAddresses(void);
    static bool isSingleByteAddress(int address);

};

#endif // ACTUATORCONTROL_H
