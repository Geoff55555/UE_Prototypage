/*
  ==============================================================================

    IMUManager.h
    Created: 10 Jul 2019 1:31:53pm
    Author:  root

  ==============================================================================
*/

#pragma once

#include <memory>
#include <atomic>

#include "Sparkfun_BNO080.h"

#include "boost/lockfree/spsc_queue.hpp"

#include "resources/Quaternion.h"

namespace AHIA
{
	class HeadphoneInfo;
}
class GeometryManager;

class IMUManager
{
    /*
        IMUManager s'occupe de demarrer le BNO080, de le calibrer et de recuperer les quaternions representant l'orientation du casque

        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        !!! connexion SPI : sur la carte du BNO080
        !!! 	 	 	 	 	 	 il faut souder les pistes de PS1 ensemble pour activer la communication SPI
        !!!                          il faut dessouder les transistors de pull-up I2C (juste enlever l'etain)
        !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    */
    public:
        IMUManager(bool needToCalibrate);
        ~IMUManager();

        // HeadphoneInfo recoit les donnees de l'IMU
        void setHeadphoneInfo(std::shared_ptr<AHIA::HeadphoneInfo> _headphoneInfo);

        // renvoie la derniere valeur d'orientation de la tete connue !!! ne peut etre utilise que par un seul thread, car lockfreequeue single producer, single user !!!
        bool getHeadRotation(iem::Quaternion<float>& _headRotation);

        // fonctions pour les commandes de calibrations (vraiment executee dans le thread imuFunc)
        void requestAutoCalibration();
        void requestBeginCalibration();
        void requestEndCalibration();
        void requestSaveCalibration();

        // verifie si l'IMU a deja ete calibre
        bool checkCalibration();

    private:

        // instance de l'IMU
		BNO080 myIMU;

		// indique s'il faut calibrer l'IMU au demarrage
		bool calibrationNeeded = false;
            
		// calibre l'IMU
		void calibrateIMU();
            
		// fonction recuperant les quaternions, executee sur un thread en permanence
		void imuFunc();

        
        // configurations des pin de l'IMU (convention wiringPi)
        int imuCSPin = 21;
        int imuINTPin = 24;
        int imuWAKPin = 22;
        int imuRSTPin = 23;
        int spiPort = 0;
        int spiSpeed = 3000000;
        
        // indique s'il faut que le thread de l'IMU s'arrete
        std::atomic<bool> killIMUThread;

        // verifie que le thread s'est bien arrete
        std::atomic<bool> IMUThreadExists;
        
        std::atomic<bool> wasContinuousCalibrationActivated;
        std::atomic<bool> isContinuousCalibrationActivated;
        std::atomic<bool> autoCalibrationNeeded;
        std::atomic<bool> saveCalibrationNeeded;

        // lockfree queue stockant les nouvelles orientations
        boost::lockfree::spsc_queue<iem::Quaternion<float>, boost::lockfree::capacity<(1<<8)>> headRotationLFQForThread_S1;

        // reference vers le GeometryManager pour lui envoyer les quaternions
        std::weak_ptr<GeometryManager> geometryManager;

        // reference vers le headphoneInfo gerant les infos
        std::weak_ptr<AHIA::HeadphoneInfo> headphoneInfo;
};
