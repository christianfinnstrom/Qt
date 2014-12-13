#include "ActuatorControl.h"
#include "dynamixel_control.h"
#include <QMap>
#include <QList>
#include <QString>
#include <algorithm>
#include <iterator>

#define DLL_API __declspec(dllexport)



const int DEFAULT_PORTNUM = 3;
const int DEFAULT_BAUDNUM = 1;

// INTERNAL SUBROUTINES (private): ******************************************************************

/**
 * @brief createDictionary : Generates a dictionary that maps control table parameter name (key) to control table memory address(value)
 * @return
 */
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


/**
 * @brief createSingleByteAddresses : QList containing Dynamixel memory addresses that are single byte. Used to avoid overwriting (writing a word to single byte address)
 * @return
 */
QList<int> createSingleByteAddresses(void){
    QList<int> singleByteAddresses;
    return singleByteAddresses << 2 << 3 << 4 << 5 << 11 << 12 << 13 << 16 << 17 << 18 << 24 << 25 << 26 << 27 << 28 << 29 << 42 << 43 << 44 << 46 << 47;
}

/**
 * @brief controlTableDictionary : Dictionary that maps control table name (key) to control table memory address (value)
 */
QMap<QString, int> controlTableDictionary = createDictionary();
/**
 * @brief singleByteAddresses : List containing Dynamixel memory addresses that are single byte. Used to avoid overwriting (writing a word to single byte address)
 */
QList<int> singleByteAddresses = createSingleByteAddresses();




// CONTROL TABLE SUBROUTINES: ******************************************************************

/**
* Attempts to initialize the communication devices
* @return 1 if success, 0 if failure
*/
int ActuatorControl::initialize(void){
    return dxl_initialize(2,1);
}


/**
 * Terminates the communication devices
 */
void ActuatorControl::terminate(){
    dxl_terminate();
}


/**
* Reads a byte or word from the Dynamixel actuator
* @param id Dynamixel actuator ID
* @param address Memory address to read from (see Control Table)
* @return Value at the memory address
*/
int ActuatorControl::readFromDxl(int id, int address){
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
void ActuatorControl::writeToDxl(int id, int address, int value){
    if (isSingleByteAddress(address)) return writeByteToDxl(id, address, value);
    else return writeWordToDxl(id, address, value);
}


/**
* Returns the model number of the Dynamixel
* @param id Dynamixel actuator ID
* @return Model number
*/
int ActuatorControl::getModelNumber(int id){
    return readFromDxl(id, controlTableDictionary["version of firmware"]);
}


/**
* Returns firmware version
* @param id Dynamixel actuator ID
* @return Firmware version
*/
int ActuatorControl::getVersionOfFirmware(int id){
    return readFromDxl(id, controlTableDictionary["model number(l)"]);
}


/**
* Returns the ID of the actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID (to check)
* @return Dynamixel actuator ID, range: 0-254
*/
int ActuatorControl::getID(int id){
    return readFromDxl(id, controlTableDictionary["id"]);
}


/**
* Sets the ID parameter on the Dynamixel actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID
* @param newID New ID value, range: 0-254
*/
void ActuatorControl::setID(int id, int newID){
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
int ActuatorControl::getBaudrate(int id){
    return readFromDxl(id, controlTableDictionary["baud rate"]);
}


/**
* Sets the baudrate
* The Baud Rate represents the communication speed (0-254).
*
* @param id Dynamixel actuator ID
* @param newBaud New baudrate value, range: 0-254
*/
void ActuatorControl::setBaudrate(int id, int newBaud){
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
int ActuatorControl::getReturnDelayTime(int id){
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
void ActuatorControl::setReturnDelayTime(int id, int newReturnDelayTime){
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
int ActuatorControl::getCWAngleLimit(int id){
    return readFromDxl(id, controlTableDictionary["cw angle limit(l)"]);
}


/**
* Sets the CW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode (servo)
* @param id Dynamixel actuator ID
* @param newCWAngleLimit New CW Angle Limit value
*/
void ActuatorControl::setCWAngleLimit(int id, int newCWAngleLimit){
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
int ActuatorControl::getCCWAngleLimit(int id){
    return readFromDxl(id, controlTableDictionary["ccw angle limit(l)"]);
}


/**
* Sets the CCW Angle Limit
* If value is set to 0, Wheel Mode is chosen. Other values, Joint Mode.
* @param id Dynamixel actuator ID
* @param newCCWAngleLimit New CCW Angle Limit value
*/
void ActuatorControl::setCCWAngleLimit(int id, int newCCWAngleLimit){
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
int ActuatorControl::getTheHighestLimitTemperature(int id){
    return readFromDxl(id, controlTableDictionary["the highest limit temperature"]);
}


/**
* Sets the Highest Limit Temperature
* NB! Should not be changed from its default value (70).
* @param id Dynamixel actuator ID
* @param valu New Highest Limit Temperature value
*/
void ActuatorControl::setTheHighestLimitTemperature(int id, int value){
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
int ActuatorControl::getTheLowestLimitVoltage(int id){
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
void ActuatorControl::setTheLowestLimitVoltage(int id, int value){
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
int ActuatorControl::getTheHighestLimitVoltage(int id){
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
void ActuatorControl::setTheHighestLimitVoltage(int id, int value){
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
int ActuatorControl::getMaxTorque(int id){
   return readFromDxl(id, controlTableDictionary["max torque(l)"]);
}


/**
* Sets the Max Torque
* How much torque the actuator produces.
* Valid values: 0 - 1023 (0% - 100%).
* @param id Dynamixel actuator ID
* @param value New Max Torque value, range: 0-1023
*/
void ActuatorControl::setMaxTorque(int id, int value){
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
int ActuatorControl::getStatusReturnLevel(int id){
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
void ActuatorControl::setStatusReturnLevel(int id, int value){
    if (value < 0 || value > 3) return;
    else writeToDxl(id, controlTableDictionary["status return level"], value);
}


/**
* Returns Alarm LED status
* @param id Dynamixel actuator ID
* @return 0 if off, 1 else
*/
int ActuatorControl::getAlarmLED(int id){
    return readFromDxl(id, controlTableDictionary["alarm led"]);
}


/**
* Sets the Alarm LED
* Off: 0, on: 1
* @param id Dynamixel actuator ID
* @param value New Alarm LED status value, 0 or 1
*/
void ActuatorControl::setAlarmLED(int id, int value){
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
int ActuatorControl::getAlarmShutdown(int id){
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
void ActuatorControl::setAlarmShutdown(int id, int value){
    writeToDxl(id, controlTableDictionary["alarm shutdown"], value);
}


/**
* Returns Torque Enable
* @param id Dynamixel actuator ID
* @return Off: 0, on: 1
*/
int ActuatorControl::getTorqueEnable(int id){
    return readFromDxl(id, controlTableDictionary["torque enable"]);
}


/**
* Sets the Torque Enable status
* @param id Dynamixel actuator ID
* @param value Off: 0, on: 1
*/
void ActuatorControl::setTorqueEnable(int id, int value){
    if (value != 0 || value != 1) return;
    else writeToDxl(id, controlTableDictionary["torque enable"], value);
}


/**
* Returns LED status
* Based on values in a byte (logic OR on each bit)
* @param id Dynamixel actuator ID
* @return Bit 2: BLUE LED, Bit 1: GREEN, Bit 0: RED LED
*/
int ActuatorControl::getLED(int id){
    return readFromDxl(id, controlTableDictionary["led"]);
}


/**
* Sets the LED status
* Based on values in a byte (logic OR on each bit)
* @param id Dynamixel actuator ID
* @param value Bit 2: BLUE LED, Bit 1: GREEN, Bit 0: RED LED
*/
void ActuatorControl::setLED(int id, int value){
    writeToDxl(id, controlTableDictionary["led"], value);
}


/**
* Returns the CW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @return CW Compliance Margin, range: 0-255
*/
int ActuatorControl::getCWComplianceMargin(int id){
    return readFromDxl(id, controlTableDictionary["cw compliance margin"]);
}


/**
* Sets the CW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @param value New CW Compliance Margin value, range: 0-255
*/
void ActuatorControl::setCWComplianceMargin(int id, int value){
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
int ActuatorControl::getCCWComplianceMargin(int id){
    return readFromDxl(id, controlTableDictionary["ccw compliance margin"]);
}


/**
* Sets the CCW Compliance Margin
* The margin designates the area around the goal position that receives no torque
* @param id Dynamixel actuator ID
* @param value New CCW Compliance Margin value, range: 0-255
*/
void ActuatorControl::setCCWComplianceMargin(int id, int value){
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
int ActuatorControl::getCWComplianceSlope(int id){
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
void ActuatorControl::setCWComplianceSlope(int id, int value){
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
int ActuatorControl::getCCWComplianceSlope(int id){
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
void ActuatorControl::setCCWComplianceSlope(int id, int value){
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
int ActuatorControl::getGoalPosition(int id){
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
void ActuatorControl::setGoalPosition(int id, int value){
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
int ActuatorControl::getMovingSpeed(int id){
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
void ActuatorControl::setMovingSpeed(int id, int value){
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
int ActuatorControl::getTorqueLimit(int id){
    return readFromDxl(id, controlTableDictionary["torque limit(l)"]);
}


/**
* Sets the Torque Limit
* Range: 0-1023, unit: 0.1%
* @param id Dynamixel actuator ID
* <param value New Torque Limit value, range: 0-1023
*/
void ActuatorControl::setTorqueLimit(int id, int value){
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
int ActuatorControl::getPresentPosition(int id){
    return readFromDxl(id, controlTableDictionary["present position(l)"]);
}


/**
* Returns the Present Speed
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Units: JOINT MODE: 0.111rpm, WHEEL MODE: 0.1%
* @param id Dynamixel actuator ID
* @return Present Speed (see description)
*/
int ActuatorControl::getPresentSpeed(int id){
    return readFromDxl(id, controlTableDictionary["present speed(l)"]);
}


/**
* Returns the Present Load
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Unit: 0.1%
* @param id Dynamixel actuator ID
* @return Present Load
*/
int ActuatorControl::getPresentLoad(int id){
    return readFromDxl(id, controlTableDictionary["present load(l)"]);
}


/**
* Returns the Present Load
* Range: 0-2047 (0-1023 CCW, 1024-2047 CW)
* Unit: 0.1%
* @param id Dynamixel actuator ID
* @return Present Load
*/
int ActuatorControl::getPresentVoltage(int id){
    return readFromDxl(id, controlTableDictionary["present "]);
}


/**
* Returns Present Temperature
* Internal temperature of the Dynamixel
* Unit: Degrees Celsius
* @param id Dynamixel actuator ID
* @return Present Temperature
*/
int ActuatorControl::getPresentTemperature(int id){
    return readFromDxl(id, controlTableDictionary["present temperature"]);
}


/**
* Returns whether Instruction is registered
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int ActuatorControl::getRegistered(int id){
    return readFromDxl(id, controlTableDictionary["registered"]);
}


/**
* Returns whether there is any movement
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int ActuatorControl::getMoving(int id){
    return readFromDxl(id, controlTableDictionary["moving"]);
}


/**
* Returns whether EEPROM is locked
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int ActuatorControl::getLock(int id){
    return readFromDxl(id, controlTableDictionary["lock"]);
}


/**
* Sets the EEPROM lock
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @param value Lock: 1, unlock: 0
*/
void ActuatorControl::setLock(int id, int value){
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
int ActuatorControl::getPunch(int id){
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
void ActuatorControl::setPunch(int id, int value){
    if (value < 32) value = 32;
    if (value > 1023) value = 1023;
    writeToDxl(id, controlTableDictionary["punch(l)"], value);
}



// ADDITIONAL SUBROUTINES ******************************************************************

/**
* Toggles the Torque on or off
* @param id Dynamixel actuator ID
*/
void ActuatorControl::torqueEnableSwitch(int id){
    int status = getTorqueEnable(id);
    if (status > 0) setTorqueEnable(id, 0); // if on, turn off
    else setTorqueEnable(id, 1); // if off, turn on
}


/**
* Returns whether an Instruction has been registered or not
* @param id Dynamixel actuator ID
* @return true/false
*/
bool ActuatorControl::isInstructionRegistered(int id){
    return (getRegistered(id) > 0 ? true : false);
}


/**
* Returns whether an Dynamixel actuator is moving or not
* @param  actuator ID
* @return true/false
*/
bool ActuatorControl::isMoving(int id){
    return (getMoving(id) > 0 ? true : false);
}


/**
* Returns whether EEPROM is locked or not
* EEPROM is a memory area (addresses 0-18) that can be locked
* @param id
* @return true/false
*/
bool ActuatorControl::isEEPROMLocked(int id){
    return (getLock(id) > 0 ? true : false);
}


/**
* Turns on WHEEL MODE on the Dynamixel actuator
* WHEEL MODE: The actuator rotates 360 degrees like a regular motor
* @param id
*/
void ActuatorControl::toggleWheelMode(int id){
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
void ActuatorControl::toggleJointMode(int id, int newCWAngleLimit, int newCCWAngleLimit){
    setCWAngleLimit(id, newCWAngleLimit);
    setCCWAngleLimit(id, newCCWAngleLimit);
}


/**
* Returns the goal position as an angular value
* @param id Dynamixel actuator ID
* @return Angular goal position value
*/
int ActuatorControl::getGoalPositionAngular(int id){
    return angularValueFromDxlValue(getGoalPosition(id));
}


/**
* Sets the goal position based on angular input
* @param id Dynamixel actuator ID
* @param angularPosition Angular goal position value
*/
void ActuatorControl::setGoalPositionAngular(int id, int angularPosition){
    setGoalPosition(id, angularValueToDxlValue(angularPosition));
}


/**
* Returns the present position as an angular value
* @param id Dynamixel actuator ID
* @return Position as angular value
*/
int ActuatorControl::getPresentPositionAngular(int id){
    return angularValueFromDxlValue(getPresentPosition(id));
}


/**
* Returns Dynamixel actuator movement mode (WHEEL MODE or JOINT MODE (servo))
* @param id Dynamixel actuator ID
* @return WHEEL MDOE: 0, JOINT MODE: 1
*/
int ActuatorControl::getMovementMode(int id){
    if (getCWAngleLimit(id) == 0 && getCCWAngleLimit(id) == 0) return 0; // WHEEL MODE
    else return 1; // JOINT MODE
}



// INTERNAL SUBROUTINES (private) ******************************************************************

void ActuatorControl::writeByteToDxl(int id, int address, int value){
    dxl_write_byte(id, address, value);
}

void ActuatorControl::writeWordToDxl(int id, int address, int value){
    dxl_write_word(id, address, value);
}

int ActuatorControl::readByteFromDxl(int id, int address){
    return dxl_read_byte(id, address);
}

int ActuatorControl::readWordFromDxl(int id, int address){
    return dxl_read_word(id, address);
}

int ActuatorControl::angularValueFromDxlValue(int value){
    return (int)(value * 0.29); // 0.29 degrees*DxlPositionValue
}

int ActuatorControl::angularValueToDxlValue(int value){
    return (int)(value / 0.29); // 0.29 degrees/DxlPositionValue
}

bool ActuatorControl::isSingleByteAddress(int address){
    return singleByteAddresses.contains(address);
}

