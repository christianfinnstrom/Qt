#include "DynamixelControl.h"
#include "dynamixel_control.h"
#include <QMap>
#include <QList>
#include <QString>
#include <algorithm>
#include <iterator>

#define DLL_API __declspec(dllexport)



const int DEFAULT_PORTNUM = 3;
const int DEFAULT_BAUDNUM = 1;

QMap<QString, int> createDictionary(void){

    QMap<QString, int> controlTableDictionary;

    controlTableDictionary["model number(l)"] = 0;
    controlTableDictionary["model number(h)"] = 1;
    controlTableDictionary["version of firmware"] = 2;
    controlTableDictionary["id"] = 3;
    controlTableDictionary["baud rate"] = 4;
    controlTableDictionary["return delay time"] = 5;
    controlTableDictionary["cw angle limit(l)"] = 6;
    controlTableDictionary["cw angle limit(h)"] = 7;
    controlTableDictionary["ccw angle limit(l)"] = 8;
    controlTableDictionary["ccw angle limit(h)"] = 9;
    controlTableDictionary["the highest limit temperature"] = 11;
    controlTableDictionary["the lowest limit voltage"] = 12;
    controlTableDictionary["the highest limit voltage"] = 13;
    controlTableDictionary["max torque(l)"] = 14;
    controlTableDictionary["max torque(h)"] = 15;
    controlTableDictionary["status return level"] = 16;
    controlTableDictionary["alarm led"] = 17;
    controlTableDictionary["alarm shutdown"] = 18;
    controlTableDictionary["torque enable"] = 24;
    controlTableDictionary["led"] = 25;
    controlTableDictionary["cw compliance margin"] = 26;
    controlTableDictionary["ccw compliance margin"] = 27;
    controlTableDictionary["cw compliance slope"] = 28;
    controlTableDictionary["ccw compliance slope"] = 29;
    controlTableDictionary["goal position(l)"] = 30;
    controlTableDictionary["goal position(h)"] = 31;
    controlTableDictionary["moving speed(l)"] = 32;
    controlTableDictionary["moving speed(h)"] = 33;
    controlTableDictionary["torque limit(l)"] = 34;
    controlTableDictionary["torque limit(h)"] = 35;
    controlTableDictionary["present position(l)"] = 36;
    controlTableDictionary["present position(h)"] = 37;
    controlTableDictionary["present speed(l)"] = 38;
    controlTableDictionary["present speed(h)"] = 39;
    controlTableDictionary["present load(l)"] = 40;
    controlTableDictionary["present load(h)"] = 41;
    controlTableDictionary["present voltage"] = 42;
    controlTableDictionary["present temperature"] = 43;
    controlTableDictionary["registered"] = 44;
    controlTableDictionary["moving"] = 46;
    controlTableDictionary["lock"] = 47;
    controlTableDictionary["punch(l)"] = 48;
    controlTableDictionary["punch(h)"] = 49;

    return controlTableDictionary;

}

QList<int> createSingleByteAddresses(void){
    QList<int> singleByteAddresses;
    return singleByteAddresses << 2 << 3 << 4 << 5 << 11 << 12 << 13 << 16 << 17 << 18 << 24 << 25 << 26 << 27 << 28 << 29 << 42 << 43 << 44 << 46 << 47;
}

QMap<QString, int> controlTableDictionary = createDictionary();
QList<int> singleByteAddresses = createSingleByteAddresses();



/**
* Attempts to initialize the communication devices
* @return 1 if success, 0 if failure
*/
int DynamixelControl::initialize(void){
    return dxl_initialize(2,1);
}

/**
 * Terminates the communication devices
 */
void DynamixelControl::terminate(){
    dxl_terminate();
}


/**
* Reads a byte or word from the Dynamixel actuator
* @param id Dynamixel actuator ID
* @param address Memory address to read from (see Control Table)
* @return Value at the memory address
*/
int DynamixelControl::readFromDxl(int id, int address){
    if (isSingleByteAddress(address)) return readByteFromDxl(id, address);
    else return readWordFromDxl(id, address);
}


/**
* Writes a byte or word to the Dynamixel actuator
* (checks whether the input is a byte or word before execution)
* @param id Dynamixel actuator ID
* @param address Memory address to write to (see Control Table)
* @param value Value to write
*/
void DynamixelControl::writeToDxl(int id, int address, int value){
    if (isSingleByteAddress(address)) return writeByteToDxl(id, address, value);
    else return writeWordToDxl(id, address, value);
}


/**
* Returns the model number of the Dynamixel
* @param id Dynamixel actuator ID
* @return Model number
*/
int DynamixelControl::getModelNumber(int id){
    return readFromDxl(id, controlTableDictionary["version of firmware"]);
}


/**
* Returns firmware version
* @param id Dynamixel actuator ID
* @return Firmware version
*/
int DynamixelControl::getVersionOfFirmware(int id){
    return readFromDxl(id, controlTableDictionary["model number(l)"]);
}


/**
* Returns the ID of the actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID (to check)
* @return Dynamixel actuator ID, range: 0-254
*/
int DynamixelControl::getID(int id){
    return readFromDxl(id, controlTableDictionary["id"]);
}


/**
* Sets the ID parameter on the Dynamixel actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID
* @param newID New ID value, range: 0-254
*/
void DynamixelControl::setID(int id, int newID){
    if (newID < 0) newID = 0;
    if (newID > 254) newID = 254;
    writeToDxl(id, controlTableDictionary["id"], newID);
}


/**
* Returns the baudrate
* The Baud Rate represents the communication speed (0-254).
* @param id Dynamixel actuator ID
* @return Baudrate, range: 0-254
*/
int DynamixelControl::getBaudrate(int id){
    return readFromDxl(id, controlTableDictionary["baud rate"]);
}


/**
* Sets the baudrate
* The Baud Rate represents the communication speed (0-254).
*
* @param id Dynamixel actuator ID
* @param newBaud New baudrate value, range: 0-254
*/
void DynamixelControl::setBaudrate(int id, int newBaud){
    if (newBaud < 0) newBaud = 0;
    if (newBaud > 254) newBaud = 254;
    writeToDxl(id, controlTableDictionary["baud rate"], newBaud);
}


/**
   * Returns Return Delay Time
   * Return Delay Time is the delay time from an Instruction Packet is
   * transmitted, until a Status Packet is received (0-254).
   * Unit: 2 usec
   * @param id Dynamixel actuator ID
   * @return Return Delay Time, range: 0-254
   */
int DynamixelControl::getReturnDelayTime(int id){
    return readFromDxl(id, controlTableDictionary["return delay time"]);
}


/**
    * Set Return Delay Time
    * Return Delay Time is the delay time from an Instruction Packet is
    * transmitted, until a Status Packet is received (0-254).
    * Unit: 2 usec
    * @param id Dynamixel actuator ID
    * @param newReturnDelayTime New Return Delay Time value, range: 0-254
    */
void DynamixelControl::setReturnDelayTime(int id, int newReturnDelayTime){
    if (newReturnDelayTime < 0) newReturnDelayTime = 0;
    if (newReturnDelayTime > 254) newReturnDelayTime = 254;
    writeToDxl(id, controlTableDictionary["return delay time"], newReturnDelayTime);
}


/**
* Returns the CW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode (servo).
* @param id Dynamixel actuator ID
* @return CW Angle Limit
*/
int DynamixelControl::getCWAngleLimit(int id){
    return readFromDxl(id, controlTableDictionary["cw angle limit(l)"]);
}


/**
* Sets the CW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode (servo)
* @param id Dynamixel actuator ID
* @param newCWAngleLimit New CW Angle Limit value
*/
void DynamixelControl::setCWAngleLimit(int id, int newCWAngleLimit){
    // Only checks if the input values are too low, values over 2047 may be used to enter Multi-turn Mode:
    if (newCWAngleLimit < 0) newCWAngleLimit = 0;
    writeToDxl(id, controlTableDictionary["cw angle limit(l)"], newCWAngleLimit);
}


/**
* Returns the CCW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode (servo).
* @param id Dynamixel actuator ID
* @return CCW Angle Limit
*/
int DynamixelControl::getCCWAngleLimit(int id){
    return readFromDxl(id, controlTableDictionary["ccw angle limit(l)"]);
}


/**
* Sets the CCW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode.
* @param id Dynamixel actuator ID
* @param newCCWAngleLimit New CCW Angle Limit value
*/
void DynamixelControl::setCCWAngleLimit(int id, int newCCWAngleLimit){
    // Only checks if the input values are too low, values over 2047 may be used to enter Multi-turn Mode:
    if (newCCWAngleLimit < 0) newCCWAngleLimit = 0;
    writeToDxl(id, controlTableDictionary["ccw angle limit(l)"], newCCWAngleLimit);
}


/**
* Returns the Highest Limit Temperature
* NB! Should not be changed from its default value (70).
* @param id Dynamixel actuator ID
* @return Highest Limit Temperature
*/
int DynamixelControl::getTheHighestLimitTemperature(int id){
    return readFromDxl(id, controlTableDictionary["the highest limit temperature"]);
}


/**
* Sets the Highest Limit Temperature
* NB! Should not be changed from its default value (70).
* @param id Dynamixel actuator ID
* @param valu New Highest Limit Temperature value
*/
void DynamixelControl::setTheHighestLimitTemperature(int id, int value){
    writeToDxl(id, controlTableDictionary["the highest limit temperature"], value);
}


/**
* Returns the Lowest Limit Voltage
* Lowest Limit Voltage is the lowest value in the voltage operation range.
* Valid values: 50 - 250.
* Unit: 0.1V
* @param id Dynamixel actuator ID
* @return Lowest Limit Voltage
*/
int DynamixelControl::getTheLowestLimitVoltage(int id){
    return readFromDxl(id, controlTableDictionary["the lowest limit voltage"]);
}


/**
* Sets the Lowest Limit Voltage
* Lowest Limit Voltage is the lowest value in the voltage operation range.
* Valid values: 50 - 250.
* Unit: 0.1V
* @param id Dynamixel actuator ID
* @param value New Lowest Limit Voltage value, range: 50-250
*/
void DynamixelControl::setTheLowestLimitVoltage(int id, int value){
    if (value < 50) value = 50;
    if (value > 250) value = 250;
    writeToDxl(id, controlTableDictionary["the lowest limit voltage"], value);
}


/**
* Returns the Highest Limit Voltage
* Highest Limit Voltage is the highest value in the voltage operation range.
* Valid values: 50 - 250.
* Unit: 0.1V
* @param id Dynamixel actuator ID
* @return Highest Limit Voltage
*/
int DynamixelControl::getTheHighestLimitVoltage(int id){
   return readFromDxl(id, controlTableDictionary["the highest limit voltage"]);
}


/**
* Sets the Highest Limit Voltage
* Highest Limit Voltage is the highest value in the voltage operation range.
* Valid values: 50 - 250.
* Unit: 0.1V
* @param id Dynamixel actuator ID
* @param value New Highest Limit Voltage value, range: 50-250
*/
void DynamixelControl::setTheHighestLimitVoltage(int id, int value){
    if (value < 50) value = 50;
    if (value > 250) value = 250;
    writeToDxl(id, controlTableDictionary["the highest limit voltage"], value);
}


/**
* Returns Max Torque
* How much torque the actuator produces.
* Valid values: 0 - 1023 (0% - 100%).
* @param id Dynamixel actuator ID
* @return Max Torque, range: 0-1023
*/
int DynamixelControl::getMaxTorque(int id){
   return readFromDxl(id, controlTableDictionary["max torque(l)"]);
}


/**
* Sets the Max Torque
* How much torque the actuator produces.
* Valid values: 0 - 1023 (0% - 100%).
* @param id Dynamixel actuator ID
* @param value New Max Torque value, range: 0-1023
*/
void DynamixelControl::setMaxTorque(int id, int value){
   if (value < 0) value = 0;
   if (value > 1023) value = 1023;
   writeToDxl(id, controlTableDictionary["max torque(l)"], value);
}


/**
* Returns Status Return Level
* Decides how to return the Status Packet.
* Value 0: No return against all commands (except PING)
* Value 1: Return only for the READ command
* Value 2: Return for all commands
* @param id Dynamixel actuator ID
* @return Status Return Level, 0, 1 or 2
*/
int DynamixelControl::getStatusReturnLevel(int id){
    return readFromDxl(id, controlTableDictionary["status return level"]);
}


/**
* Sets the Status Return Level
* Decides how to return the Status Packet.
* Value 0: No return against all commands (except PING)
* Value 1: Return only for the READ command
* Value 2: Return for all commands
* @param id Dynamixel actuator ID
* @param value New Status Return Level value, 0, 1 or 2
*/
void DynamixelControl::setStatusReturnLevel(int id, int value){
    if (value < 0 || value > 3) return;
    else writeToDxl(id, controlTableDictionary["status return level"], value);
}


/**
* Returns Alarm LED status
* @param id Dynamixel actuator ID
* @return 0 if off, 1 else
*/
int DynamixelControl::getAlarmLED(int id){
    return readFromDxl(id, controlTableDictionary["alarm led"]);
}


/**
* Sets the Alarm LED
* Off: 0, on: 1
* @param id Dynamixel actuator ID
* @param value New Alarm LED status value, 0 or 1
*/
void DynamixelControl::setAlarmLED(int id, int value){
    if (value != 0 || value != 1) return;
    else writeToDxl(id, controlTableDictionary["alarm led"], value);
}


/**
* Returns Alarm Shutdown status
* The Dynamixel can protect itself by detecting errors during operation.
* At shutdown, Torque limit is set to 0.
* The settings depend on values in a byte; each bit decides whether
* the error corresponding to that byte-position is to be detected or not (logic OR on each bit):
*
* Bit 7: 0
* Bit 6: Instruction error
* Bit 5: Overload error
* Bit 4: CheckSum error
* Bit 3: Range error
* Bit 2: Overheating error
* Bit 1: Angle limit error
* Bit 0: Input voltage error
*
* Example: 0X05 (00000101) will turn on both Input voltage error and Overheating error.
* @param id Dynamixel actuator ID
* @return See description
*/
int DynamixelControl::getAlarmShutdown(int id){
    return readFromDxl(id, controlTableDictionary["alarm shutdown"]);
}


/**
* Sets the Alarm Shutdown
* The Dynamixel can protect itself by detecting errors during operation.
* At shutdown, Torque limit is set to 0.
* The settings depend on values in a byte; each bit decides whether
* the error corresponding to that byte-position is to be detected or not (logic OR on each bit):
*
* Bit 7: 0
* Bit 6: Instruction error
* Bit 5: Overload error
* Bit 4: CheckSum error
* Bit 3: Range error
* Bit 2: Overheating error
* Bit 1: Angle limit error
* Bit 0: Input voltage error
*
* Example: 0X05 (00000101) will turn on both Input voltage error and Overheating error.
* @param id Dynamixel actuator ID
* @param value New Alarm Shutdown value (see description)
*/
void DynamixelControl::setAlarmShutdown(int id, int value){
    writeToDxl(id, controlTableDictionary["alarm shutdown"], value);
}


/**
* Returns Torque Enable
* @param id Dynamixel actuator ID
* @return Off: 0, on: 1
*/
int DynamixelControl::getTorqueEnable(int id){
    return readFromDxl(id, controlTableDictionary["torque enable"]);
}


/**
* Sets the Torque Enable status
* @param id Dynamixel actuator ID
* @param value Off: 0, on: 1
*/
void DynamixelControl::setTorqueEnable(int id, int value){
    if (value != 0 || value != 1) return;
    else writeToDxl(id, controlTableDictionary["torque enable"], value);
}


/**
* Returns LED status
* Based on values in a byte (logic OR on each bit)
* @param id Dynamixel actuator ID
* @return Bit 2: BLUE LED, Bit 1: GREEN, Bit 0: RED LED
*/
int DynamixelControl::getLED(int id){
    return readFromDxl(id, controlTableDictionary["led"]);
}


/**
* Sets the LED status
* Based on values in a byte (logic OR on each bit)
* @param id Dynamixel actuator ID
* @param value Bit 2: BLUE LED, Bit 1: GREEN, Bit 0: RED LED
*/
void DynamixelControl::setLED(int id, int value){
    writeToDxl(id, controlTableDictionary["led"], value);
}


/**
* Returns the CW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @return CW Compliance Margin, range: 0-255
*/
int DynamixelControl::getCWComplianceMargin(int id){
    return readFromDxl(id, controlTableDictionary["cw compliance margin"]);
}


/**
* Sets the CW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @param value New CW Compliance Margin value, range: 0-255
*/
void DynamixelControl::setCWComplianceMargin(int id, int value){
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    writeToDxl(id, controlTableDictionary["cw compliance margin"], value);
}


/**
*  Returns the CCW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @return CCW Compliance Margin, range: 0-255
*/
int DynamixelControl::getCCWComplianceMargin(int id){
    return readFromDxl(id, controlTableDictionary["ccw compliance margin"]);
}


/**
* Sets the CCW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @param value New CCW Compliance Margin value, range: 0-255
*/
void DynamixelControl::setCCWComplianceMargin(int id, int value){
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    writeToDxl(id, controlTableDictionary["ccw compliance margin"], value);
}


/**
* Returns the CW Compliance Slope
* Sets the level of torque near the goal position.
* There are seven levels; higher value means more flexibility:
*
* 2, 4, 8, 16, 32, 64, 128
*
* @param id Dynamixel actuator ID
* @return CW Compliance Slope (see description)
*/
int DynamixelControl::getCWComplianceSlope(int id){
    return readFromDxl(id, controlTableDictionary["cw compliance slope"]);
}


/**
* Sets the CW Compliance Slope
* Sets the level of torque near the goal position.
* There are seven levels; higher value means more flexibility:
*
* 2, 4, 8, 16, 32, 64, 128
* @param id Dynamixel actuator ID
* @param value New CW Compliance Slope value (2, 4, 8, 16, 32, 64, 128)
*/
void DynamixelControl::setCWComplianceSlope(int id, int value){
    if (value < 0) value = 0;
    if (value > 255) value = 254;
    writeToDxl(id, controlTableDictionary["cw compliance slope"], value);
}


/**
* Returns the CCW Compliance Slope
* Sets the level of torque near the goal position.
* There are seven levels; higher value means more flexibility:
*
* 2, 4, 8, 16, 32, 64, 128
* @param id Dynamixel actuator ID
* @return CCW Compliance Slope (see description)
*/
int DynamixelControl::getCCWComplianceSlope(int id){
    return readFromDxl(id, controlTableDictionary["ccw compliance slope"]);
}


/**
* Sets the CCW Compliance Slope
* Sets the level of torque near the goal position.
* There are seven levels; higher value means more flexibility:
*
* 2, 4, 8, 16, 32, 64, 128
* @param id Dynamixel actuator ID
* @param value New CCW Compliance Slope value (2, 4, 8, 16, 32, 64, 128)
*/
void DynamixelControl::setCCWComplianceSlope(int id, int value){
   if (value < 0) value = 0;
   if (value > 255) value = 254;
   writeToDxl(id, controlTableDictionary["ccw compliance slope"], value);
}


/**
* Returns the Goal Position
*
* 0-1023, the unit is 0.29 degrees.
*
* If Goal Position is out of range, Alarm Limit Error will be triggered, and
* Alarm LED/Alarm Shutdown will be executed.
* @param id Dynamixel actuator ID
* @return Goal Position, range: 0-1023
*/
int DynamixelControl::getGoalPosition(int id){
    return readFromDxl(id, controlTableDictionary["goal position(l)"]);
}


/**
* Sets the Goal Position
* 0-1023, the unit is 0.29 degrees.
* If Goal Position is out of range, Alarm Limit Error will be triggered, and
* Alarm LED/Alarm Shutdown will be executed.
* @param id Dynamixel actuator ID
* <param value New Goal Position value, range: 0-1023
*/
void DynamixelControl::setGoalPosition(int id, int value){
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    writeToDxl(id, controlTableDictionary["goal position(l)"], value);
}


/**
* Returns the Moving Speed
* Range and unit of the value varies, depending on operation mode:
*
* JOINT MODE - range: 0-1023, unit: 0.111rpm, example: value 300 --> 33.3rpm
* WHEEL MODE - range: 0-2047 (0-1023 CCW, 1024-2047 CW), unit: 0.1%
* @param id Dynamixel actuator ID
* @return Moving Speed (see description)
*/
int DynamixelControl::getMovingSpeed(int id){
    return readFromDxl(id, controlTableDictionary["moving speed(l)"]);
}


/**
* Sets the Moving Speed
* Range and unit of the value varies, depending on operation mode:
*
* JOINT MODE - range: 0-1023, unit: 0.111rpm, example: value 300 --> 33.3rpm
* WHEEL MODE - range: 0-2047 (0-1023 CCW, 1024-2047 CW), unit: 0.1%
* @param id Dynamixel actuator ID
* <param value New Moving Speed value (see description)
*/
void DynamixelControl::setMovingSpeed(int id, int value){
    if (value < 0) value = 0;
    else{
        if (getMovementMode(id) == 0){ // WHEEL MODE
            if(value < 2047) value = 2047;
        }
        else{ // JOINT MODE
            if(value < 1023) value = 1023;
        }
    }
    writeToDxl(id, controlTableDictionary["moving speed(l)"], value);
    }


/**
* Returns the Torque Limit
* Range: 0-1023, unit: 0.1%
* @param id Dynamixel actuator ID
* @return Torque Limit
*/
int DynamixelControl::getTorqueLimit(int id){
    return readFromDxl(id, controlTableDictionary["torque limit(l)"]);
}


/**
* Sets the Torque Limit
* Range: 0-1023, unit: 0.1%
* @param id Dynamixel actuator ID
* <param value New Torque Limit value, range: 0-1023
*/
void DynamixelControl::setTorqueLimit(int id, int value){
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    writeToDxl(id, controlTableDictionary["torque limit(l)"], value);
}


/**
* Returns Present Position
* Range: 0-1023, unit: 0.29 degrees
* @param id Dynamixel actuator ID
* @return Present Position
*/
int DynamixelControl::getPresentPosition(int id){
    return readFromDxl(id, controlTableDictionary["present position(l)"]);
}


/**
* Returns the Present Speed
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Units: JOINT MODE: 0.111rpm, WHEEL MODE: 0.1%
* @param id Dynamixel actuator ID
* @return Present Speed (see description)
*/
int DynamixelControl::getPresentSpeed(int id){
    return readFromDxl(id, controlTableDictionary["present speed(l)"]);
}


/**
* Returns the Present Load
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Unit: 0.1%
* @param id Dynamixel actuator ID
* @return Present Load
*/
int DynamixelControl::getPresentLoad(int id){
    return readFromDxl(id, controlTableDictionary["present load(l)"]);
}


/**
* Returns the Present Load
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Unit: 0.1%
* @param id Dynamixel actuator ID
* @return Present Load
*/
int DynamixelControl::getPresentVoltage(int id){
    return readFromDxl(id, controlTableDictionary["present "]);
}


/**
* Returns Present Temperature
* Internal temperature of the Dynamixel
* Unit: Degrees Celsius
* @param id Dynamixel actuator ID
* @return Present Temperature
*/
int DynamixelControl::getPresentTemperature(int id){
    return readFromDxl(id, controlTableDictionary["present temperature"]);
}


/**
* Returns whether Instruction is registered
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int DynamixelControl::getRegistered(int id){
    return readFromDxl(id, controlTableDictionary["registered"]);
}


/**
* Returns whether there is any movement
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int DynamixelControl::getMoving(int id){
    return readFromDxl(id, controlTableDictionary["moving"]);
}


/**
* Returns whether EEPROM is locked
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int DynamixelControl::getLock(int id){
    return readFromDxl(id, controlTableDictionary["lock"]);
}


/**
* Sets the EEPROM lock
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @param value Lock: 1, unlock: 0
*/
void DynamixelControl::setLock(int id, int value){
    if (value != 0 || value != 1) return;
    else writeToDxl(id, controlTableDictionary["lock"], value);
}


/**
* Returns the Punch
* Punch is the minimum voltage that will be applied to the motor when the position
* is just outside the compliance margin. It is needed to overcome internal gear friction.
* Default value: 32
* @param id Dynamixel actuator ID
* @return Punch
*/
int DynamixelControl::getPunch(int id){
    return readFromDxl(id, controlTableDictionary["punch(l)"]);
}


/**
* Sets the Punch
* Punch is the minimum voltage that will be applied to the motor when the position
* is just outside the compliance margin. It is needed to overcome internal gear friction.
* Default value: 32
* @param id
* @param value New Punch value, range: 32-1023
*/
void DynamixelControl::setPunch(int id, int value){
    if (value < 32) value = 32;
    if (value > 1023) value = 1023;
    writeToDxl(id, controlTableDictionary["punch(l)"], value);
}



// ADDITIONAL METHODS

/**
* Toggles the Torque on or off
* @param id Dynamixel actuator ID
*/
void DynamixelControl::torqueEnableSwitch(int id){
    int status = getTorqueEnable(id);
    if (status > 0) setTorqueEnable(id, 0); // if on, turn off
    else setTorqueEnable(id, 1); // if off, turn on
}


/**
* Returns whether an Instruction has been registered or not
* @param id Dynamixel actuator ID
* @return true/false
*/
bool DynamixelControl::isInstructionRegistered(int id){
    return (getRegistered(id) > 0 ? true : false);
}


/**
* Returns whether an Dynamixel actuator is moving or not
* @param  actuator ID
* @return true/false
*/
bool DynamixelControl::isMoving(int id){
    return (getMoving(id) > 0 ? true : false);
}


/**
* Returns whether EEPROM is locked or not
* EEPROM is a memory area (addresses 0-18) that can be locked
* @param id
* @return true/false
*/
bool DynamixelControl::isEEPROMLocked(int id){
    return (getLock(id) > 0 ? true : false);
}


/**
* Turns on WHEEL MODE on the Dynamixel actuator
* WHEEL MODE: The actuator rotates 360 degrees like a regular motor
* @param id
*/
void DynamixelControl::toggleWheelMode(int id){
    setCWAngleLimit(id, 0);
    setCCWAngleLimit(id, 0);
}


/**
* Turns on JOINT MODE on the Dynamixel actuator
* JOINT MODE: The actuator moves at a set angle range (CW angle limit, CCW angle limit)
* @param id Dynamixel actuator ID
* @param newCWAngleLimit New CW Angle Limit
* @param newCCWAngleLimit New CCW Angle Limit
*/
void DynamixelControl::toggleJointMode(int id, int newCWAngleLimit, int newCCWAngleLimit){
    setCWAngleLimit(id, newCWAngleLimit);
    setCCWAngleLimit(id, newCCWAngleLimit);
}


/**
* Returns the goal position as an angular value
* @param id Dynamixel actuator ID
* @return Angular goal position value
*/
int DynamixelControl::getGoalPositionAngular(int id){
    return angularValueFromDxlValue(getGoalPosition(id));
}


/**
* Sets the goal position based on angular input
* @param id Dynamixel actuator ID
* @param angularPosition Angular goal position value
*/
void DynamixelControl::setGoalPositionAngular(int id, int angularPosition){
    setGoalPosition(id, angularValueToDxlValue(angularPosition));
}


/**
* Returns the present position as an angular value
* @param id Dynamixel actuator ID
* @return Position as angular value
*/
int DynamixelControl::getPresentPositionAngular(int id){
    return angularValueFromDxlValue(getPresentPosition(id));
}


/**
* Returns Dynamixel actuator movement mode (WHEEL MODE or JOINT MODE (servo))
* @param id Dynamixel actuator ID
* @return WHEEL MDOE: 0, JOINT MODE: 1
*/
int DynamixelControl::getMovementMode(int id){
    if (getCWAngleLimit(id) == 0 && getCCWAngleLimit(id) == 0) return 0; // WHEEL MODE
    else return 1; // JOINT MODE
}



/*
 * The following subroutines are used internally:
 */


void DynamixelControl::writeByteToDxl(int id, int address, int value){
    dxl_write_byte(id, address, value);
}

void DynamixelControl::writeWordToDxl(int id, int address, int value){
    dxl_write_word(id, address, value);
}

int DynamixelControl::readByteFromDxl(int id, int address){
    return dxl_read_byte(id, address);
}

int DynamixelControl::readWordFromDxl(int id, int address){
    return dxl_read_word(id, address);
}

int DynamixelControl::angularValueFromDxlValue(int value){
    return (int)(value * 0.29); // 0.29 degrees*DxlPositionValue
}

int DynamixelControl::angularValueToDxlValue(int value){
    return (int)(value / 0.29); // 0.29 degrees/DxlPositionValue
}




bool DynamixelControl::isSingleByteAddress(int address){
    return singleByteAddresses.contains(address);
}

