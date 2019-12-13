/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output accelerometer values

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <iomanip>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "SparkFun_BNO080_Library.h"

BNO080 myIMU;

//These pins can be any GPIO : modified with WiringPi pins
int imuCSPin = 3;
int imuINTPin = 7;
int imuWAKPin = 2;
int imuRSTPin = 1;
int spiPort = 0;
int spiSpeed = 3000000;

unsigned long startTime; //Used for calc'ing Hz
long measurements = 0; //Used for calc'ing Hz

unsigned long requestStartTime = 0;
unsigned long requestEndTime = 0;

unsigned long prevRequestStartTime = 0;

int main(int argc, char **argv)
{
	
	std::cout << "BNO080 Read Example\n";
	delay(2000);
	
	myIMU.enableDebugging();
	
	while(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, spiSpeed, spiPort) == false)
	{
	std::cout << "BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...\n";
	delay(1000);
	}

	//The IMU is now connected over SPI
	//Please see the other examples for library functions that you can call

	myIMU.enableRotationVector((uint16_t)10); //Send data update every x ms
	myIMU.printPacket();

	std::cout << "Rotation vector enabled\n";
	std::cout << "Output in form i, j, k, real, accuracy\n";

	startTime = micros();
	
	while(1)
	{
		//delay(10); //You can do many other things. We spend most of our time printing and delaying.
		//Look for reports from the IMU
		if (myIMU.dataAvailable() == true)
		{
			requestStartTime = micros();
			float quatI = myIMU.getQuatI();
			float quatJ = myIMU.getQuatJ();
			float quatK = myIMU.getQuatK();
			float quatReal = myIMU.getQuatReal();
			float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
			requestEndTime = micros();
			//measurements++;
			
			std::cout << std::fixed << std::setprecision(6) << quatI;
			std::cout << ",";
			std::cout << std::fixed << std::setprecision(6) << quatJ;
			std::cout << ",";
			std::cout << std::fixed << std::setprecision(6) << quatK;
			std::cout << ",";
			std::cout << std::fixed << std::setprecision(6) << quatReal;
			std::cout << "\n";
			//std::cout << std::fixed << std::setprecision(6) << quatRadianAccuracy;
			//std::cout << ",";
			//std::cout << std::fixed << std::setprecision(4) << ((float)requestEndTime / ((micros() - startTime) / 1000000.0));
			//std::cout << "Hz\n";
			//std::cout << std::fixed << std::setprecision(4) << ((float)measurements / ((micros() - startTime) / 1000000.0));
			//std::cout << "Hz\n";
			
			std::cout << "Time to get quaternions : ";
			std::cout << (float)requestEndTime - (float)requestStartTime;
			std::cout << "us\n";
			
			std::cout << "Time between two updates : ";
			std::cout << ((float)requestStartTime - (float)prevRequestStartTime)/1000.0;
			std::cout << "ms\n";
			
			prevRequestStartTime = requestStartTime;
			
			delay(6);
		}
	}
	return 0;
}
