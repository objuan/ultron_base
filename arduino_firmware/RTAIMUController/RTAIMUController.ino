////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>


RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object
RTFusionRTQF fusion;                                  // the fusion object

//  SERIAL_PORT_SPEED defines the speed to use for the serial port

#define  SERIAL_PORT_SPEED  115200

uint8_t teapotPacket[32] = { '$', 0x05, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '\r', '\n' };

void setup()
{
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
 

    Wire.begin();

    imu = RTIMU::createIMU(&settings);                  // create the imu object
    
    Serial.print("RTArduLinkIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");


     // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
   
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}

void loop()
{ 
    unsigned char state;
    unsigned long now = millis();
    unsigned long delta;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;

        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        // build message
        /*RTArduLinkConvertLongToUC4(millis(), linkMessage.timestamp);
        linkMessage.gyro[0] = imu->getGyro().x();
        linkMessage.gyro[1] = imu->getGyro().y();
        linkMessage.gyro[2] = imu->getGyro().z();
        linkMessage.accel[0] = imu->getAccel().x();
        linkMessage.accel[1] = imu->getAccel().y();
        linkMessage.accel[2] = imu->getAccel().z();
        linkMessage.mag[0] = imu->getCompass().x();
        linkMessage.mag[1] = imu->getCompass().y();
        linkMessage.mag[2] = imu->getCompass().z();
        
        state = 0;
        if (imu->IMUGyroBiasValid())
            state |= RTARDULINKIMU_STATE_GYRO_BIAS_VALID;
        if (imu->getCalibrationValid())
            state |= RTARDULINKIMU_STATE_MAG_CAL_VALID;
            
        // send the message
        linkIMU.sendMessage(RTARDULINK_MESSAGE_IMU, state,
                (unsigned char *)(&linkMessage), sizeof(RTARDULINKIMU_MESSAGE));
                */
        int16_t decimals = 10000;

        RTQuaternion f = fusion.getFusionQPose();
       // const RTVector3 &fpose = fusion.getFusionPose();
        
        int16_t accelCount[3] = { 
          (int16_t)(imu->getAccel().x() * decimals),
          (int16_t)(imu->getAccel().y() * decimals),
         (int16_t)(imu->getAccel().z() * decimals)
        };
        

      int16_t gyroCount[3] = { 
          (int16_t)(imu->getGyro().x() * decimals),
          (int16_t)(imu->getGyro().y() * decimals),
         (int16_t)(imu->getGyro().z() * decimals)
        };
        
        int16_t magCount[3] = { 
          (int16_t)(imu->getCompass().x() * decimals),
          (int16_t)(imu->getCompass().y() * decimals),
         (int16_t)(imu->getCompass().z() * decimals)
        };
/*
         int16_t pose[3] = { 
          (int16_t)(fpose.x() * decimals),
          (int16_t)(fpose.y() * decimals),
         (int16_t)(fpose.z() * decimals)
        };
      */  
        
        int16_t fus[4] = { 
          (int16_t)(f.x() * decimals),
          (int16_t)(f.y() * decimals),
         (int16_t)(f.z() * decimals),
         (int16_t)(f.scalar() * decimals)
        };
        
        // send
         // ACC
        int cc=2;
        for(int i=0;i<3;i++)
        {
         teapotPacket[cc++] = accelCount[i]  >> 8;
         teapotPacket[cc++] =  accelCount[i] & 0xFF;
        }
        // Giro
         
         for(int i=0;i<3;i++)
        {
         teapotPacket[cc++] = gyroCount[i]  >> 8;
         teapotPacket[cc++] =  gyroCount[i] & 0xFF;
        }
      
         // mag
         
         for(int i=0;i<3;i++)
        {
         teapotPacket[cc++] = magCount[i]  >> 8;
         teapotPacket[cc++] =  magCount[i] & 0xFF;
        }

      // pose
     /*      for(int i=0;i<3;i++)
        {
         teapotPacket[cc++] = pose[i]  >> 8;
         teapotPacket[cc++] =  pose[i] & 0xFF;
        }
        */
         // fusion
         
        for(int i=0;i<4;i++)
        {
         teapotPacket[cc++] = fus[i]  >> 8;
         teapotPacket[cc++] =  fus[i] & 0xFF;
        }
       
         Serial.write(teapotPacket, 32);
       
       //RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
        //  RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
       //   RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
        //    RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
        //   Serial.println();
    }
     
}

