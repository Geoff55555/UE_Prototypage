/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>



//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 1024 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

class BNO080
{
  public:
	bool beginSPI(int user_CSPin, int user_WAKPin, int user_INTPin, int user_RSTPin, int user_spiPortSpeed, int user_spiPort);

	void enableDebugging(); //Turn on debug printing

	void softReset();	  //Try to reset the IMU via software
	uint8_t resetReason(); //Query the IMU for the reason it last reset

	float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

	bool waitForSPI(); //Delay based polling for INT pin to go low
	bool receivePacket(void);
	bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
	void printPacket(void); //Prints the current shtp header and data packets

	void enableRotationVector(uint16_t timeBetweenReports);
	void enableGameRotationVector(uint16_t timeBetweenReports);
	void enableAccelerometer(uint16_t timeBetweenReports);
	void enableLinearAccelerometer(uint16_t timeBetweenReports);
	void enableGyro(uint16_t timeBetweenReports);
	void enableMagnetometer(uint16_t timeBetweenReports);
	void enableStepCounter(uint16_t timeBetweenReports);
	void enableStabilityClassifier(uint16_t timeBetweenReports);
	void enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);

	bool dataAvailable(void);
	void parseInputReport(void);   //Parse sensor readings out of report
	void parseCommandReport(void); //Parse command responses out of report

	float getQuatI();
	float getQuatJ();
	float getQuatK();
	float getQuatReal();
	float getQuatRadianAccuracy();
	uint8_t getQuatAccuracy();

	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	void calibrateAccelerometer();
	void calibrateGyro();
	void calibrateMagnetometer();
	void calibratePlanarAccelerometer();
	void calibrateAll();
	void endCalibration();
	void saveCalibration();
	void requestCalibrationStatus(); //Sends command to get status
	bool calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

	uint32_t getTimeStamp();
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();

	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
	void sendCommand(uint8_t command);
	void sendCalibrateCommand(uint8_t thingToCalibrate);

	//Metadata functions
	int16_t getQ1(uint16_t recordID);
	int16_t getQ2(uint16_t recordID);
	int16_t getQ3(uint16_t recordID);
	float getResolution(uint16_t recordID);
	float getRange(uint16_t recordID);
	uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	uint8_t shtpData[MAX_PACKET_SIZE];
	uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

  private:
	//Variables

	bool _printDebug = false; //Flag to print debugging variables

	int _spiPort;		//The user's chosen SPI port
	int _spiPortSpeed;	//User defined port speed
	int _cs;			//Pins needed for SPI
	int _wake;
	int _int;
	int _rst;

	//These are the raw sensor values pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences; //Array that store the confidences of the 9 possible activities
	uint8_t calibrationStatus;	 //Byte R0 of ME Calibration Response

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
};