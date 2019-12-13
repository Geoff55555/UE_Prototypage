/*
  ==============================================================================

    IMUManager.cpp
    Created: 10 Jul 2019 1:31:53pm
    Author:  root

  ==============================================================================
*/

#include "IMUManager.h"


#include <thread>
#include "HeadphoneInfo.h"


#define byte uint8_t

IMUManager::IMUManager(bool needToCalibrate)
{
	wasContinuousCalibrationActivated = false;
	isContinuousCalibrationActivated = false;
	autoCalibrationNeeded = false;
	saveCalibrationNeeded = false;

    killIMUThread = false;
	calibrationNeeded = needToCalibrate;
	std::thread(&IMUManager::imuFunc, this).detach();
}

IMUManager::~IMUManager()
{
	killIMUThread = true;
	while(IMUThreadExists) {}
}

void IMUManager::setHeadphoneInfo(std::shared_ptr<AHIA::HeadphoneInfo> _headphoneInfo)
{
	headphoneInfo = _headphoneInfo;
}

bool IMUManager::getHeadRotation(iem::Quaternion<float>& _headRotation)
{
	bool newValue = false;
	while(headRotationLFQForThread_S1.pop(_headRotation))
		newValue = true;
	return newValue;
}

void IMUManager::requestAutoCalibration()
{
	autoCalibrationNeeded = true;
}

void IMUManager::requestBeginCalibration()
{
	isContinuousCalibrationActivated = true;
}

void IMUManager::requestEndCalibration()
{
	isContinuousCalibrationActivated = false;
}

void IMUManager::requestSaveCalibration()
{
	saveCalibrationNeeded = true;
}

bool IMUManager::checkCalibration()
{
	//Enable Game Rotation Vector output -> mix accel + gyro
	myIMU.enableGameRotationVector(10); //Send data update every 100ms

	//Enable Magnetic Field output -> que magneto
	myIMU.enableMagnetometer(10); //Send data update every 100ms

	// regarde les 100 premieres valeurs
	int count = 0;
	int goodData = 0;
	while(count != 100)
	{
		if(myIMU.dataAvailable())
		{
			byte quatAccuracy = myIMU.getQuatAccuracy();
			byte magAccuracy = myIMU.getMagAccuracy();

			if(quatAccuracy == 3 && magAccuracy == 3)
				++goodData;

			++count;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(6));
	}

	// si le taux de haute precisions est assez eleve, considere l'IMU comme etant deja calibre
	std::cout << "[BNO080] percentage of data accurate enough : " << (double)goodData/(double)count << std::endl;
	if((double)goodData/(double)count >= 0.8)
	{
		return true;
	}


	return false;
}

void IMUManager::calibrateIMU()
{
	// indique que l'IMU est en train d'etre calibre
	if(auto headphoneInfoLocked = headphoneInfo.lock())
	{
		headphoneInfoLocked->setIMUCalibrationStatus(true);
	}


    bool calibrated = false;
    myIMU.calibrateAll();

    // pour la calibration, on active le magnetometre et accel/gyro separement

    //Enable Game Rotation Vector output -> mix accel + gyro
    myIMU.enableGameRotationVector(10); //Send data update every 100ms

    //Enable Magnetic Field output -> que magneto
    myIMU.enableMagnetometer(10); //Send data update every 100ms

    int numData = 0;
	int numHighQuatAccuracyData = 0;
	int numHighMagAccuracyData = 0;
	std::cout << "Begin calibration" << std::endl;

	// tant que l'IMU n'est pas calibre on repete l'operation
	while(!calibrated)
	{
		if(myIMU.dataAvailable())
		{
			byte quatAccuracy = myIMU.getQuatAccuracy();

			if(quatAccuracy == 3)
			{
				++numHighQuatAccuracyData;
			}

			byte magAccuracy = myIMU.getMagAccuracy();

			if(auto headphoneInfoLocked = headphoneInfo.lock())
			{
				headphoneInfoLocked->setOrientation(1, 0, 0, 0);
				headphoneInfoLocked->setIMUQuatAccuracy((int)quatAccuracy);
				headphoneInfoLocked->setIMUMagAccuracy((int)magAccuracy);
			}

			if(magAccuracy == 3)
				++numHighMagAccuracyData;

			++numData;
			if(numData == 1000)
			{
			    // si 80% des donnees fournies par les quaternions (gyro + accel)
				// et par le magnetometre seul sont corrects (High accuracy) -> on enregistre la calibration
				if((double)numHighQuatAccuracyData/(double)numData >= 0.8 && (double)numHighMagAccuracyData/(double)numData >= 0.8)
				{
					std::cout << "High enough : try to store calibration data" << std::endl;

					myIMU.saveCalibration();
					myIMU.requestCalibrationStatus();

					// essaie 100x de stocker les donnees de calibration
					int counter = 100;
					while(1)
					{
						--counter;

						// si apres 100x on a toujours pas reussi a stocker les donnees, on recommence depuis le debut
						if(counter == 0)
							break;

						if(myIMU.dataAvailable() == true)
						{
							if(myIMU.calibrationComplete() == true)
							{
								calibrated = true;
								std::cout << "Success" << std::endl;
								break;
							}
						}
						// laisser un peu de temps pour essayer d'enregistrer les donnees de calibration
						std::this_thread::sleep_for(std::chrono::milliseconds(1));
					}
					if(counter == 0)
						std::cout << "Calibration data failed to store, trying again" << std::endl;
				}
				else
				{
					std::cout << "Not high enough : quat = " << (double)numHighQuatAccuracyData/(double)numData
							<< " and mag = " << (double)numHighMagAccuracyData/(double)numData << std::endl;
				}
				
				// remise a zero du nombre total de donnees et du nombre de donnees avec bonne precision
				numData = 0;
				numHighQuatAccuracyData = 0;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(6));
	}
	if(!isContinuousCalibrationActivated)
	{
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!! STOP calibration !!!!!!!!!!!" << std::endl;
		myIMU.endCalibration();
		std::this_thread::sleep_for(std::chrono::milliseconds(6));
	}
	else
		std::cout << "Nope" << std::endl;
	std::cout << "End auto calibration" << std::endl;
}

void IMUManager::imuFunc()
{
    // fonction du THREAD de l'IMU

    IMUThreadExists = true;
    int numNoData = 0;

    // numero des pins utilisees (convention wiringPi)
    imuCSPin = 3;//21;
    imuINTPin = 7;//24;
    int imuWAKPin = 2;//22;
    imuRSTPin = 1;//23;
    spiPort = 0;
    spiSpeed = 3000000;  
    
    // indique que l'IMU n'est pas encore actif
    if(auto headphoneInfoLocked = headphoneInfo.lock())
    	headphoneInfoLocked->doesIMURun(false);

    // lance la communication SPI avec l'IMU
	while(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, spiSpeed, spiPort) == false)
	{
	    std::cout << "BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...\n";
	    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// indique que l'IMU est actif
	if(auto headphoneInfoLocked = headphoneInfo.lock())
		headphoneInfoLocked->doesIMURun(true);
		
	
	// check si l'IMU est deja calibre ou si il faut absolument calibrer au demarrage
	if(calibrationNeeded || !checkCalibration())
	    calibrateIMU();
	

	// pour changer apres de vecteur a envoyer (la calibration utilise un autre vecteu)
	myIMU.softReset();

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// relance le spi pour recuperer l'IMU
	while(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, spiSpeed, spiPort) == false)
	{
		std::cout << "BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}


	// garde le magnetometre actif pour continuer a verifier sa precision
	myIMU.enableMagnetometer(10);

	// quaternion = magnetometre + accel + gyro
	myIMU.enableRotationVector((uint16_t)10); //Send data update every x ms

	// indique que l'IMU n'est pas en train d'etre calibre
	if(auto headphoneInfoLocked = headphoneInfo.lock())
	{
		headphoneInfoLocked->setIMUCalibrationStatus(false);
	}

	while(1)
    {
		// verifie s'il faut arreter le thread
        if(killIMUThread == true)
        {
            break;
        }
        
        // verifie si demande de calibration
        if(autoCalibrationNeeded)
        {
        	calibrateIMU();
        	autoCalibrationNeeded = false;
        }

        // verifie si demande de calibration manuel (laisser en calibration continue jusqu'a la reception du signal de fin)
        if(isContinuousCalibrationActivated && !wasContinuousCalibrationActivated)
        {
        	std::cout << "!!!!!!!!!!!!! calibrate all !!!!!!!!!!" << std::endl;
        	myIMU.calibrateAll();
        	wasContinuousCalibrationActivated = true;
        }

        // verifie si fin de calibration manuelle
        if(!isContinuousCalibrationActivated && wasContinuousCalibrationActivated)
        {
        	std::cout << "!!!!!!!!!!!!! end calibration !!!!!!!!!!" << std::endl;
        	myIMU.endCalibration();
        	std::this_thread::sleep_for(std::chrono::milliseconds(6));
        	wasContinuousCalibrationActivated = false;
        }

        // verifie si demande de sauvegarde des donnees de calibration
        if(saveCalibrationNeeded)
        {
        	if(isContinuousCalibrationActivated)
        	{
        		myIMU.saveCalibration();
				myIMU.requestCalibrationStatus();
				int counter = 100;
				while(1)
				{
					--counter;
					if(counter == 0)
						break;
					if(myIMU.dataAvailable() == true)
					{
						if(myIMU.calibrationComplete() == true)
						{
							if(auto headphoneInfoLocked = headphoneInfo.lock())
								headphoneInfoLocked->Log("Calibration data successfully stored");
							break;
						}
					}
					// laisser un peu de temps pour essayer d'enregistrer les donnees de calibration
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
				if(counter == 0)
				{
					if(auto headphoneInfoLocked = headphoneInfo.lock())
						headphoneInfoLocked->Log("Calibration data failed to store, trying again");
				}
        	}
        	saveCalibrationNeeded = false;
        }

        // si nouvelle donnes, recuperer les infos
        if (myIMU.dataAvailable() == true)
        {
        	numNoData = 0;

        	// recupere le quaternion
		    float quatI = myIMU.getQuatI();
		    float quatJ = myIMU.getQuatJ();
		    float quatK = myIMU.getQuatK();
		    float quatReal = myIMU.getQuatReal();

		    // recupere la precision des quaternions et du magnetometre
		    byte sensorAccuracy = myIMU.getQuatAccuracy();
		    byte magAccuracy = myIMU.getMagAccuracy();

		    // conversion pour donner le decalage de l'axe X par rapport au Nord
		    iem::Quaternion<float> quaternion_S1(quatReal, quatI, quatJ, quatK);
		    float ypr[3];
		    quaternion_S1.toYPR(ypr);
		    ypr[0] -= (M_PI/2.0f);
		    quaternion_S1.fromYPR(ypr);

		    // envoie les info au headphoneInfo
		    if(auto headphoneInfoLocked = headphoneInfo.lock())
		    {
		    	headphoneInfoLocked->setOrientation(quaternion_S1.w, quaternion_S1.x, quaternion_S1.y, quaternion_S1.z);
		    	headphoneInfoLocked->setIMUQuatAccuracy((int)sensorAccuracy);
		    	headphoneInfoLocked->setIMUMagAccuracy((int)magAccuracy);
		    }
		    
		    headRotationLFQForThread_S1.push(quaternion_S1);

		}
		else
        {
			++numNoData;
			if(numNoData >= 50)
			{
				if(auto headphoneInfoLocked = headphoneInfo.lock())
				{
					std::cout << "50 x no data available, consider IMU lost" << std::endl;
					headphoneInfoLocked->Log("50 x no data available, consider IMU lost");
					headphoneInfoLocked->doesIMURun(false);
				}
				numNoData = 0;
			}

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(6));
    }

	// fin du thread de l'IMU

    myIMU.endCalibration();
    IMUThreadExists = false;
    
    if(auto headphoneInfoLocked = headphoneInfo.lock())
    	headphoneInfoLocked->Log("Closing IMU thread");

}
