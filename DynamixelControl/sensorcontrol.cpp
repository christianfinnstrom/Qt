#include "sensorcontrol.h"
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

QMap<QString, int> createSensorDictionary(void){

    QMap<QString, int> sensorControlTableDictionary;

    sensorControlTableDictionary["model number(l)"] = 0;
    sensorControlTableDictionary["model number(h)"] = 1;
    sensorControlTableDictionary["version of firmware"] = 2;
    sensorControlTableDictionary["id"] = 3;
    sensorControlTableDictionary["baud rate"] = 4;
    sensorControlTableDictionary["return delay time"] = 5;
    sensorControlTableDictionary["status return level"] = 16;
    sensorControlTableDictionary["ir left fire data"] = 26;
    sensorControlTableDictionary["ir center fire data"] = 27;
    sensorControlTableDictionary["ir right fire data"] = 28;
    sensorControlTableDictionary["light left data"] = 29;
    sensorControlTableDictionary["light center data"] = 30;
    sensorControlTableDictionary["light right data"] = 31;
    sensorControlTableDictionary["ir obstacle detected"] = 32;
    sensorControlTableDictionary["light detected"] = 33;
    sensorControlTableDictionary["sound data"] = 35;
    sensorControlTableDictionary["sound data max hold"] = 36;
    sensorControlTableDictionary["sound detected count"] = 37;
    sensorControlTableDictionary["sound detected time(l)"] = 38;
    sensorControlTableDictionary["sound detected time(h)"] = 39;
    sensorControlTableDictionary["buzzer data 0"] = 40;
    sensorControlTableDictionary["buzzer data 1"] = 41;
    sensorControlTableDictionary["registered"] = 44;
    sensorControlTableDictionary["ir remocon arrived"] = 46;
    sensorControlTableDictionary["lock"] = 47;
    sensorControlTableDictionary["remocon rx data 0"] = 48;
    sensorControlTableDictionary["remocon rx data 1"] = 49;
    sensorControlTableDictionary["remocon tx data 0"] = 50;
    sensorControlTableDictionary["remocon tx data 1"] = 51;
    sensorControlTableDictionary["ir obstacle detect comparerd"] = 52;
    sensorControlTableDictionary["light detect comparerd"] = 53;


    return sensorControlTableDictionary;
}

/**
 * @brief createSensorsingleByteSensorAddresses : QList containing Dynamixel memory addresses that are single byte. Used to avoid overwriting (writing a word to single byte address)
 * @return
 */
QList<int> createSingleByteSensorAddresses(void){
    QList<int> singleByteSensorAddresses;
    return singleByteSensorAddresses << 2 << 3 << 4 << 5 << 16 << 26 << 27 << 28 << 29 << 30 << 31 << 32 << 33 << 35 << 36 << 37 << 40 << 41 << 44 << 46 << 47 << 52 << 53;
}

/**
 * @brief sensorControlTableDictionary : Dictionary that maps control table name (key) to control table memory address (value)
 */
QMap<QString, int> sensorControlTableDictionary = createSensorDictionary();
/**
 * @brief singleByteSensorAddresses : List containing Dynamixel memory addresses that are single byte. Used to avoid overwriting (writing a word to single byte address)
 */
QList<int> singleByteSensorAddresses = createSingleByteSensorAddresses();



// CONTROL TABLE SUBROUTINES: ******************************************************************
/**
* Attempts to initialize the communication devices
* @return 1 if success, 0 if failure
*/
int SensorControl::initialize(void){
    return dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM);
}


/**
 * Terminates the communication devices
 */
void SensorControl::terminate(){
    dxl_terminate();
}


/**
* Reads a byte or word from the Dynamixel actuator
* @param id Dynamixel actuator ID
* @param address Memory address to read from (see Control Table)
* @return Value at the memory address
*/
int SensorControl::readFromDxl(int id, int address){
    if (isSingleByteSensorAddress(address)) return readByteFromDxl(id, address);
    else return readWordFromDxl(id, address);
}


/**
* Writes a byte or word to the Dynamixel actuator
* (checks whether the input is a byte or word before execution)
* @param id Dynamixel actuator ID
* @param address Memory address to write to (see Control Table)
* @param value Value to write
*/
void SensorControl::writeToDxl(int id, int address, int value){
    if (isSingleByteSensorAddress(address)) return writeByteToDxl(id, address, value);
    else return writeWordToDxl(id, address, value);
}


/**
* Returns the model number of the Dynamixel
* @param id Dynamixel actuator ID
* @return Model number
*/
int SensorControl::getModelNumber(int id){
    return readFromDxl(id, sensorControlTableDictionary["version of firmware"]);
}


/**
* Returns firmware version
* @param id Dynamixel actuator ID
* @return Firmware version
*/
int SensorControl::getVersionOfFirmware(int id){
    return readFromDxl(id, sensorControlTableDictionary["model number(l)"]);
}


/**
* Returns the ID of the actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID (to check)
* @return Dynamixel actuator ID, range: 0-254
*/
int SensorControl::getID(int id){
    return readFromDxl(id, sensorControlTableDictionary["id"]);
}


/**
* Sets the ID parameter on the Dynamixel actuator
* 254 is the Broadcast ID
* @param id Dynamixel actuator ID
* @param newID New ID value, range: 0-254
*/
void SensorControl::setID(int id, int newID){
    if (newID < 0) newID = 0;
    if (newID > 254) newID = 254;
    writeToDxl(id, sensorControlTableDictionary["id"], newID);
}


/**
* Returns the baudrate
* The Baud Rate represents the communication speed (0-254).
* @param id Dynamixel actuator ID
* @return Baudrate, range: 0-254
*/
int SensorControl::getBaudrate(int id){
    return readFromDxl(id, sensorControlTableDictionary["baud rate"]);
}


/**
* Sets the baudrate
* The Baud Rate represents the communication speed (0-254).
*
* @param id Dynamixel actuator ID
* @param newBaud New baudrate value, range: 0-254
*/
void SensorControl::setBaudrate(int id, int newBaud){
    if (newBaud < 0) newBaud = 0;
    if (newBaud > 254) newBaud = 254;
    writeToDxl(id, sensorControlTableDictionary["baud rate"], newBaud);
}


/**
* Returns Return Delay Time
* Return Delay Time is the delay time from an Instruction Packet is
* transmitted, until a Status Packet is received (0-254).
* Unit: 2 usec
* @param id Dynamixel actuator ID
* @return Return Delay Time, range: 0-254
*/
int SensorControl::getReturnDelayTime(int id){
    return readFromDxl(id, sensorControlTableDictionary["return delay time"]);
}


/**
* Set Return Delay Time
* Return Delay Time is the delay time from an Instruction Packet is
* transmitted, until a Status Packet is received (0-254).
* Unit: 2 usec
* @param id Dynamixel actuator ID
* @param newReturnDelayTime New Return Delay Time value, range: 0-254
*/
void SensorControl::setReturnDelayTime(int id, int newReturnDelayTime){
    if (newReturnDelayTime < 0) newReturnDelayTime = 0;
    if (newReturnDelayTime > 254) newReturnDelayTime = 254;
    writeToDxl(id, sensorControlTableDictionary["return delay time"], newReturnDelayTime);
}


/**
* Returns Status Return Level
* Decides how to return the Status Packet.
* Value 0: No return against all commands (except PING)
* Value 1: Return only for the read command
* Value 2: Return for all commands
* @param id Dynamixel actuator ID
* @return Status Return Level, 0, 1 or 2
*/
int SensorControl::getStatusReturnLevel(int id){
    return readFromDxl(id, sensorControlTableDictionary["status return level"]);
}

/**
* sets the Status Return Level
* Decides how to return the Status Packet.
* Value 0: No return against all commands (except PING)
* Value 1: Return only for the read command
* Value 2: Return for all commands
* @param id Dynamixel actuator ID
* @param value New Status Return Level value, 0, 1 or 2
*/
void SensorControl::setStatusReturnLevel(int id, int value){
     if (value < 0 || value > 3) return;
     else writeToDxl(id, sensorControlTableDictionary["status return level"], value);
}


/**
* Left IR sensor value for distance measure.
* Infrared rays are emitted from the IR-emitting part. The sensors measure the amount of reflected rays.
* A higher value means that more rays are reflected, e.g. objects are located closer.
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getIRLeftFireData(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir left fire data"]);
}

/**
* Center IR sensor value for distance measure.
* Infrared rays are emitted from the IR-emitting part. The sensors measure the amount of reflected rays.
* A higher value means that more rays are reflected, e.g. objects are located closer.
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getIRCenterFireData(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir center fire data"]);
}

/**
* Right IR sensor value for distance measure.
* Infrared rays are emitted from the IR-emitting part. The sensors measure the amount of reflected rays.
* A higher value means that more rays are reflected, e.g. objects are located closer.
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getIRRightFireData(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir right fire data"]);
}

/**
* Left Light Brightness Sensor
* Measures the amount of infrared rays.
* Similar to the distance measurement, but without any IR-self-emittion.
* A higher value means more brightness
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getLightLeftData(int id){
    return readFromDxl(id, sensorControlTableDictionary["light left data"]);
}

/**
* Center Light Brightness Sensor
* Measures the amount of infrared rays.
* Similar to the distance measurement, but without any IR-self-emittion.
* A higher value means more brightness
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getLightCenterData(int id){
    return readFromDxl(id, sensorControlTableDictionary["light center data"]);
}

/**
* Right Light Brightness Sensor
* Measures the amount of infrared rays.
* Similar to the distance measurement, but without any IR-self-emittion.
* A higher value means more brightness
* @param id Dynamixel sensor ID
* @return A value between 0-255
*/
int SensorControl::getLightRightData(int id){
    return readFromDxl(id, sensorControlTableDictionary["light right data"]);
}

/**
* Returns whether an object is detected within the defined range or not.
* If IR Distance Sensor value (IR Fire Data) is greater than the compare value, 1 is returned.
* The detection compare value can be changed by using the function setIRObstacleDetectCompareRD
* @param id Dynamixel sensor ID
* @return 0: no object detected within range, 1: object detected
*/
int SensorControl::getIRObstacleDetected(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir obstacle detected"]);
}


/**
* Returns whether the brightness sensor value (Light data) is greater than the compare value, or not.
* The detection compare value can be changed by using the function setIRObstacleDetectCompareRD
* @param id Dynamixel sensor ID
* @return 0: darker than reference value, 1: brighter than reference value
*/
int SensorControl::getLightDetected(int id){
    return readFromDxl(id, sensorControlTableDictionary["light detected"]);
}

/**
* Measures the level of sound being recorded by the microphone.
* The sound level is measured 3800 times/sec
* Returns a numerical value: If no sound, 127-128 is returned, and the
* value approaces 0 or 255 as it gets _louder_.
* @param id Dynamixel sensor ID
* @return No sound: 127-128. Louder sounds: values close to 0 or 255
*/
int SensorControl::getSoundData(int id){
    return readFromDxl(id, sensorControlTableDictionary["sound data"]);
}


/**
* Returns the maximum sound level
* If the sound level during sound measurement exceeds the current maximum,
* the value is updated (setSoundDataMaxHold).
* To make sure that the value will be updated, reset the value before measurement (set value to 0)!
* Sound levels beneath 128 are ignored. 255 is maximum.
* @param id Dynamixel sensor ID
* @return Maximum sound level
*/
int SensorControl::getSoundDataMaxHold(int id){
    return readFromDxl(id, sensorControlTableDictionary["sound data max hold"]);
}

/**
* sets the maximum sound level
* This function can be used to reset the maximum sound level value,
* to ensure that the value is up to date after a measurement, by setting the value to 0.
* (The maximum sound level is only updated if it exceeds the current value. Therefore,
* if the current value is louder than the loudest sound level during a measurement, the
* value will not be updated)
* @param id Dynamixel sensor ID
* @param value Maximum sound level value. To reset, send 0
*/
void SensorControl::setSoundDataMaxHold(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["sound data max hold"], value);
}

/**
* Returns the number of times a certain sound level is measured
* (See online manual for supplementary information)
* @param id Dynamixel sensor ID
* @return
*/
int SensorControl::getSoundDetectedCount(int id){
    return readFromDxl(id, sensorControlTableDictionary["sound detected count"]);
}

/**
* See online manual for information
* @param id Dynamixel sensor ID
* @param value
*/
void SensorControl::setSoundDetected(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["sound detected count"], value);
}

/**
* Saves the time of the moment a sound detection occurd.
* See online manual for supplementary information
* @param id Dynamixel sensor ID
* @return
*/
int SensorControl::getSoundDetectedTime(int id){
    return readFromDxl(id, sensorControlTableDictionary["sound detected time"]);
}

/**
* See online manual for information
* @param id <Dynamixel sensor ID/param>
* @param value
*/
void SensorControl::setSoundDetectedTime(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["sound detected time"], value);
}



/**
* Returns the current set buzzer note (noteAddress) on the Dynamixel sensor
* Note table:
* http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_28
* @param id Dynamixel sensor ID
* @return The current set buzzer note
*/
int SensorControl::getBuzzerData0(int id){
    return readFromDxl(id, sensorControlTableDictionary["buzzer data 0"]);
}


/**
* Plays the desired buzzer note (noteAddress) on the Dynamixel sensor
* Note table:
* http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_28
* @param id Dynamixel sensor ID
* @param noteAddress Buzzer note (see buzzer note table online)
*/
void SensorControl::setBuzzerData0(int id, int noteAddress){
    writeToDxl(id, sensorControlTableDictionary["buzzer data 0"], noteAddress);
}



/**
* Returns the current set buzzer ringing time.
* A returned value of 50 --> 5 seconds.
* @param id Dynamixel sensor ID
* @return Unit: 0.1 second
*/
int SensorControl::getBuzzerData1(int id){
    return readFromDxl(id, sensorControlTableDictionary["buzzer data 1"]);
}

/**
* set buzzer ringing time
* A value of 50 --> 5 seconds.
* @param id Dynamixel sensor ID
* @param value Ringing time. Unit: 0.1 second
*/
void SensorControl::setBuzzerData1(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["buzzer data 1"], value);
}


/**
* Returns whether Instruction is registered
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int SensorControl::getRegistered(int id){
    return readFromDxl(id, sensorControlTableDictionary["registered"]);
}

/**
* See online manual for instructions
* @param id Dynamixel sensor ID
* @param value
*/
void SensorControl::setRegistered(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["registered"], value);
}

/**
* IR remote controller communication status.
* AX-S1 can communicate through its infrared emitters and sensors.
* If data is received, the value is updated to '2', e.g. 2 bytes are received.
* If these data are read, the value is set back to '0'.
* @param id Dynamixel sensor ID
* @return 2: new, unread data. 0: no new data
*/
int SensorControl::getIRRemoconArrived(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir remocon arrived"]);
}

/**
* Returns whether EEPROM is locked
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @return False: 0, true: 1
*/
int SensorControl::getLock(int id){
    return readFromDxl(id, sensorControlTableDictionary["lock"]);
}

/**
* sets the EEPROM lock
* EEPROM is a memory area (addresses 0-18) that can be locked from modification
* @param id Dynamixel actuator ID
* @param value Lock: 1, unlock: 0
*/
void SensorControl::setLock(int id, int value){
     if (value != 0 || value != 1) return;
     else writeToDxl(id, sensorControlTableDictionary["lock"], value);
}

/**
* Returns received Remocon sensor data (IR remote control data).
* If this data is read, remocon arrived data will be set to 0 (marked as read).
* @param id Dynamixel sensor ID
* @return Received Remocon data
*/
int SensorControl::getRemoconRXData(int id){
    return readFromDxl(id, sensorControlTableDictionary["remocon rx data 0"]);
}

/**
* Returns Remocon data (IR remote control data) to be transmitted
* @param id Dynamixel sensor ID
* @return Remocon data to be transmitted
*/
int SensorControl::getRemoconTXData(int id){
    return readFromDxl(id, sensorControlTableDictionary["remocon tx data 0"]);
}

/**
* set the Remocon data (IR remote control data) to be transmitted
* 2 bytes of data can be transmitted
* @param id Dynamixel sensor ID
* @param value Value to transmit via IR
*/
void SensorControl::setRemoconTXData(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["remocon tx data 0"], value);
}

/**
* Returns the current IR obstacle detection compare value
* This value is used in the IRObstacleDetected method.
* If '0': low sensitive mode; used for close range.
* See online manual for supplementary information:
* http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_34
* @param id Dynamixel sensor ID
* @return The current IR detection compare value
*/
int SensorControl::getIRObstacleDetectCompareRD(int id){
    return readFromDxl(id, sensorControlTableDictionary["ir obstacle detect comparerd"]);
}


/**
* sets the current IR obstacle detection compare value
* This value is used in the IRObstacleDetected method.
* See online manual for supplementary information:
* http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_34
* @param id
* @param value
*/
void SensorControl::setIRObstacleDetectCompareRD(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["ir obstacle detect compared"], value);
}

/**
* Returns the current Light detect compare value.
* This value is used in the getLightDetected method.
* @param id Dynamixel sensor ID
* @return Light detect compare value
*/
int SensorControl::getLightDetectCompareRD(int id){
    return readFromDxl(id, sensorControlTableDictionary["light detect comparerd"]);
}

/**
* sets the current Light detect compare value.
* This value is used in the getLightDetected method.
* @param id Dynamixel sensor ID
* @param value New light detect compare value
*/
void SensorControl::setLightDetectCompareRD(int id, int value){
    writeToDxl(id, sensorControlTableDictionary["light detect compared"], value);
}



/*
* ADDITIONAL METHODS for improved usability:
*/

/**
* Returns the current set buzzer note on the Dynamixel sensor.
* (This method runs GetBuzzerData0)
* See online manual for note table:
* http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_28
* @param id Dynamixel sensor ID
* @return The current buzzer note
*/
int SensorControl::getCurrentBuzzerNote(int id){
    return getBuzzerData0(id);
}


/**
* Play buzzer notes. Simple beep sounds can be made. 52 musical notes can be made
* in buzzer tones, and there are also whole- and halftones in each octave.
* (This method runs SetBuzzerData0).
* See online manual for note table:
* (http://support.robotis.com/en/product/auxdevice/sensor/dxl_ax_s1.htm#Ax_S1_Address_28)
* @param id Dynamixel sensor ID
* @param noteAddress Buzzer note to play
*/
void SensorControl::playBuzzerNote(int id, int noteAddress){
    setBuzzerData0(id, noteAddress);
}


/**
* Returns the current set buzzer ringing time.
* A returned value of 50 --> 5 seconds.
*
* @param id Dynamixel sensor ID
* @return Unit: 0.1 second
*/
int SensorControl::getBuzzerRingingTime(int id){
    return getBuzzerData1(id);
}

/**
* set buzzer ringing time
* A value of 50 --> 5 seconds.
*
* @param id Dynamixel sensor ID
* @param value Ringing time. Unit: 0.1 second
*/
void SensorControl::setBuzzerRingingTime(int id, int value){
    setBuzzerData1(id, value);
}

/**
* Resets the Sound Data Max Hold, so that it is prepared for a new measurement.
* @param id Dynamixel sensor ID
*/
void SensorControl::ResetSoundDataMaxHold(int id){
    setSoundDataMaxHold(id, 0);
}

// INTERNAL SUBROUTINES (private) ******************************************************************

void SensorControl::writeByteToDxl(int id, int address, int value){
    dxl_write_byte(id, address, value);
}

void SensorControl::writeWordToDxl(int id, int address, int value){
    dxl_write_word(id, address, value);
}

int SensorControl::readByteFromDxl(int id, int address){
    return dxl_read_byte(id, address);
}

int SensorControl::readWordFromDxl(int id, int address){
    return dxl_read_word(id, address);
}

bool SensorControl::isSingleByteSensorAddress(int address){
    return singleByteSensorAddresses.contains(address);
}

