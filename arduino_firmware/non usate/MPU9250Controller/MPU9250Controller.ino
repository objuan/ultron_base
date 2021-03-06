/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#define SAMPLE_RATE_CONTROLLER 50

#include "quaternionFilters.h"
#include "MPU9250.h"

//#define CALIBRATE_MODE true   
//#define IR_CONTROL true
#define DEBUG_MODE true         // Set to false for basic data read
//#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
//int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;

int Mmode = 0x02;

int16_t mag_temp[3] = {0, 0, 0};
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
//float  magBias[3] = {50.61970901489258, 141.49842834472656, -106.39176177978516};
//float magScale[3]  = {1,1,1};   

//float  magBias[3] = {96.00, 69.82, -546.53};
//float magScale[3]  = {1,1,1}; 


//float  magBias[3] = {61.09, 174.54, -143.38};
//float  magBias[3] = {64.58,221.66, 109.64};
//float  magBias[3] = {59.34,137.89,91.09};
//float  magBias[3] = {48.87144.87,-111.33};
//float  magBias[3] = {66.32, 143.12, -104.58};
//float magScale[3]  = {1.02,1.05,0.94};   

/*
mag x min/max:-238 347
mag y min/max:-159 453
mag z min/max:-337 324
bias:76.80,256.57,-40.48
scale:0.99,0.98,1.03

 */

float magScale[3]  = {1,1,1};   
float  magBias[3] = {0,0,0};

int16_t  mag_min[3] = {-238, -159, - 337};
int16_t  mag_max[3] = {347, 453, 324};

void magcalMPU9250(float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

   Serial.print(Mmode);
  Serial.println(" -> Mag Calibration: Wave device in a figure eight until done!");
  delay(2000);
   Serial.println("begin");
   
    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
  //  readMagData(mag_temp);  // Read the mag data   
      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      myIMU.getMres();
      mag_temp[0] = myIMU.magCount[0];
      mag_temp[1] = myIMU.magCount[1];
      mag_temp[2] = myIMU.magCount[2];
   
      byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
      for (int jj = 0; jj < 3; jj++) {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
      if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
      if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    Serial.print("mag x min/max:"); Serial.print(mag_min[0]);  Serial.print(" "); Serial.println(mag_max[0]);
    Serial.print("mag y min/max:"); Serial.print(mag_min[1]); Serial.print(" ");Serial.println(mag_max[1]);
    Serial.print("mag z min/max:"); Serial.print(mag_min[2]); Serial.print(" ");Serial.println(mag_max[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

     Serial.print("Ray:"); Serial.println(avg_rad);
   Serial.print("bias:"); Serial.print(dest1[0]); Serial.print(",");Serial.print(dest1[1]) ;Serial.print(",");Serial.println(dest1[2]);

    Serial.print("scale:"); Serial.print(dest2[0]); Serial.print(",");Serial.print(dest2[1]) ;Serial.print(",");Serial.println(dest2[2]);

   Serial.println("Mag Calibration done!");
}

void magcalMPU9250_old(float * dest1, float * dest2) {
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
 
    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    sample_count = 128;
 
    for(ii = 0; ii < sample_count; ii++) {
        //readMagData(mag_temp);  // Read the mag data
        myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
        myIMU.getMres();
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
      delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    }

 
    Serial.print("mag x min/max:"); Serial.print(mag_min[0]);  Serial.print(" "); Serial.println(mag_max[0]);
    Serial.print("mag y min/max:"); Serial.print(mag_min[1]); Serial.print(" ");Serial.println(mag_max[1]);
    Serial.print("mag z min/max:"); Serial.print(mag_min[2]); Serial.print(" ");Serial.println(mag_max[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
 
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;
    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    Serial.print("bias:"); Serial.print(dest1[0]); Serial.print(",");Serial.println(dest1[1]) ;Serial.print(",");Serial.println(dest1[2]);

    Serial.print("scale:"); Serial.print(dest2[0]); Serial.print(",");Serial.println(dest2[1]) ;Serial.print(",");Serial.println(dest2[2]);

    Serial.println("Mag Calibration done!");
}  

// ===============================

//remote 
#include "RemoteController.h"

RemoteController remoteController;

long last_time_ms_controller;
long sample_rate_ms_controller;

//==================================================

uint8_t teapotPacket[28] = { '$', 0x04, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void setup()
{
  Wire.begin();
  
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  //pinMode(myLed, OUTPUT);
 // digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (true)//c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);



    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
  //  #if (DEBUG_MODE)
 //   {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
  //  }
  //  #endif

#if CALIBRATE_MODE
  magcalMPU9250(magBias, magScale);

    Serial.println("calibration data");
      
    Serial.println(magBias[0], 2);
  Serial.println(magBias[1], 2);
  Serial.println(magBias[2], 2);
   Serial.println("--");
   Serial.println(magScale[0], 2);
  Serial.println(magScale[1], 2);
  Serial.println(magScale[2], 2);
   Serial.println("--");
  
     Serial.println("calibration data fac");
      
    Serial.println(myIMU.magbias[0], 2);
  Serial.println(myIMU.magbias[1], 2);
  Serial.println(myIMU.magbias[2], 2);
   Serial.println("--");
   Serial.println(myIMU.magCalibration[0], 2);
  Serial.println(myIMU.magCalibration[1], 2);
  Serial.println(myIMU.magCalibration[2], 2);
   Serial.println("--");
 #endif
 
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }


// -------------------
#if (IR_CONTROL)

    remoteController.setup();
    sample_rate_ms_controller = 1000 / SAMPLE_RATE_CONTROLLER;
    last_time_ms_controller = millis();


#endif
Serial.println("DONE");

}

// ---------

void loop()
{
  #if CALIBRATE_MODE
 //    return;
   #endif
    // --- CONTROLLER
    #if (IR_CONTROL)

     long now = millis();
     if (now-last_time_ms_controller > sample_rate_ms_controller)
     {
        last_time_ms_controller = now;
        
        remoteController.loop();
     }
    // return;
  #endif
     
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();

  #if DEBUG_MODE

  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  float dest1[3] = {0, 0, 0};
  float dest2[3] = {0, 0, 0};
  
      myIMU.mx = (float)myIMU.magCount[0];
      myIMU.my = (float)myIMU.magCount[1];
      myIMU.mz = (float)myIMU.magCount[2];

       // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   //  Serial.print("Ray:"); Serial.println(avg_rad);
   // Serial.print("bias:"); Serial.print(dest1[0]); Serial.print(",");Serial.print(dest1[1]) ;Serial.print(",");Serial.println(dest1[2]);
   // Serial.print("scale:"); Serial.print(dest2[0]); Serial.print(",");Serial.print(dest2[1]) ;Serial.print(",");Serial.println(dest2[2]);


      myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*dest2[0] -
                 dest1[0];
      myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*dest2[1] -
                 dest1[1];
      myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*dest2[2] -
                 dest1[2];

     // Get actual magnetometer value, this depends on scale being set
     myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] ;
     myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1];
     myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2];
                 
     //  Serial.print("VALUE:"); Serial.print( myIMU.mx); Serial.print(",");Serial.print( myIMU.my) ;Serial.print(",");Serial.println( myIMU.mz);

  #endif
  
  #if !DEBUG_MODE
  
      // update bias
      myIMU.magbias[0] = magBias[0];
      myIMU.magbias[1] = magBias[1];
      myIMU.magbias[2] = magBias[2];
      myIMU.magCalibration[0] = magScale[0];
      myIMU.magCalibration[1] = magScale[1];
      myIMU.magCalibration[2] = magScale[2];
      
      
      // User environmental x-axis correction in milliGauss, should be
      // automatically calculated
      //myIMU.magbias[0] = +470.;
      // User environmental x-axis correction in milliGauss TODO axis??
      //myIMU.magbias[1] = +120.;
      // User environmental x-axis correction in milliGauss
      //myIMU.magbias[2] = +125.;
  
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                 myIMU.magbias[0];
      myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                 myIMU.magbias[1];
      myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                 myIMU.magbias[2];
   #endif
               
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

 #if !defined(DEBUG_MODE)
  // WORK OATH
  // -----------------
  // ACC
  int cc=2;
  for(int i=0;i<3;i++)
  {
   teapotPacket[cc++] = myIMU.accelCount[i]  >> 8;
   teapotPacket[cc++] =  myIMU.accelCount[i] & 0xFF;
  }
  // Giro
   
   for(int i=0;i<3;i++)
  {
   teapotPacket[cc++] = myIMU.gyroCount[i]  >> 8;
   teapotPacket[cc++] =  myIMU.gyroCount[i] & 0xFF;
  }

   // mag
   
   for(int i=0;i<3;i++)
  {
   teapotPacket[cc++] = myIMU.magCount[i]  >> 8;
   teapotPacket[cc++] =  myIMU.magCount[i] & 0xFF;
  }

   Serial.write(teapotPacket, 28);
   
  // ----------------
  
#else
  
  if (true) // percorso normale di lavoro
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(true)//SerialDebug)
      {
        // Print acceleration values in milligs!
     /*   Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");
*/
        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");
/*
        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
        */
      }


      myIMU.count = millis();
    //  digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(DEBUG_MODE)
      {
      /*  Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
        Serial.println(" mg");
        */
/*
        Serial.print("gx = "); Serial.print( myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
        Serial.println(" deg/s");
        */

        Serial.print("mx = "); Serial.print( (int)myIMU.mx );
        Serial.print(" my = "); Serial.print( (int)myIMU.my );
        Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
        Serial.println(" mG");
/*
        Serial.print("q0 = "); Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
        */
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      if(true)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.print(myIMU.roll, 2);

        Serial.print(" rate = ");
        Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (DEBUG_MODE)
  #endif
}
