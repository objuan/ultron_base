#include "sensors.h"

#include <sensor_msgs/Range.h>
sensor_msgs::Range range_msg;
ros::Publisher pub_range( ROS_TOPIC_RANGE_SENSOR, &range_msg);

// ===================================

#ifdef IMU_ENABLE

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


//#define OUTPUT_READABLE_YAWPITCHROLL

// ================ ROS ===================


#include <ultron_kernel/RobotIMU.h>

ultron_kernel::RobotIMU imu_msg;
ros::Publisher pub_imu( ROS_TOPIC_IMU, &imu_msg);

// =========== MPU =================

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void SensorManager::pre_setup_mpu() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        TWBR = 12; //24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

void SensorManager::setup_mpu() {
  

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    #ifdef DEBUG_IMU
    Serial.println(F("Initializing I2C devices..."));
    #else
   // (*nh).loginfo("Initializing I2C devices...");
    #endif
    mpu.initialize();

  //  pinMode(IMU_INT_PIN, INPUT);

  // configure LED for output
  //  pinMode(IMU_LED_PIN, OUTPUT);

  #ifdef DEBUG_IMU
    Serial.println(F("Initializing I2C devices..1"));     
    #endif
}
  
void SensorManager::init_mpu()
{

    // verify connection
    #ifdef DEBUG_IMU
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

       // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    #else
    
    (*nh).loginfo("Testing device connections...");
    (*nh).loginfo(mpu.testConnection() ? ("MPU6050 connection successful") : ("MPU6050 connection failed"));
    
    #endif


    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity

// from MPU6050_calibration
// Data is printed as: acelX acelY acelZ giroX giroY giroZ

//    893  -5643 975 49  5 -18
    
    mpu.setXAccelOffset(893);
    mpu.setYAccelOffset(-5643);
    mpu.setZAccelOffset(975);
    mpu.setXGyroOffset(49);
    mpu.setYGyroOffset(5);
    mpu.setZGyroOffset(-18);
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        #ifdef DEBUG_IMU
        Serial.println(F("Enabling DMP..."));
        #else
        (*nh).loginfo("Enabling DMP...");
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef DEBUG_IMU
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt IMU_INT_PIN)..."));
        #else
        (*nh).loginfo("Enabling interrupt detection (Arduino external interrupt IMU_INT_PIN)...");
        #endif
        
        attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        #ifdef DEBUG_IMU
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        #else
        (*nh).loginfo("DMP ready! Waiting for first interrupt...");
        #endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef DEBUG_IMU
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #else
         
        sprintf(log_msg, "DMP Initialization failed (code: %s) ", devStatus);
        (*nh).logerror(log_msg);
        #endif
    }

    // configure LED for output
  //  pinMode(LED_PIN, OUTPUT);
}

void SensorManager::load_mpu() {
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
       // nh.spinOnce();
        delay(1);
       return;
    }
    

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

  if (false)
  {
      dtostrf(fifoCount,4,3,tmp_msg1);
      sprintf(log_msg, "fifoCount ( %s) ", tmp_msg1);
      (*nh).logerror(log_msg);
  }
        
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
       
         #ifdef DEBUG_IMU
        Serial.println(F("FIFO overflow!"));
        #else
       // (*nh).loginfo("MPU FIFO overflow!");
        #endif
         mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
    
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
          // FIX A LIBRARY BUG
          //fifoCount = mpu.getFIFOCount();
          delay(10);
          return;
        }

      // (*nh).loginfo("DONE!");
  
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

 #ifndef DEBUG_IMU


            imu_msg.teapotData[0] = fifoBuffer[0];
            imu_msg.teapotData[1] = fifoBuffer[1];
            imu_msg.teapotData[2] = fifoBuffer[4];
            imu_msg.teapotData[3] = fifoBuffer[5];
            imu_msg.teapotData[4] = fifoBuffer[8];
            imu_msg.teapotData[5] = fifoBuffer[9];
            imu_msg.teapotData[6] = fifoBuffer[12];
            imu_msg.teapotData[7] = fifoBuffer[13];
            // gyro values
            imu_msg.teapotData[8] = fifoBuffer[16];
            imu_msg.teapotData[9] = fifoBuffer[17];
            imu_msg.teapotData[10] = fifoBuffer[20];
            imu_msg.teapotData[11] = fifoBuffer[21];
            imu_msg.teapotData[12] = fifoBuffer[24];
            imu_msg.teapotData[13] = fifoBuffer[25];
            // accelerometer values
            imu_msg.teapotData[14] = fifoBuffer[28];
            imu_msg.teapotData[15] = fifoBuffer[29];
            imu_msg.teapotData[16] = fifoBuffer[32];
            imu_msg.teapotData[17] = fifoBuffer[33];
            imu_msg.teapotData[18] = fifoBuffer[36];
            imu_msg.teapotData[19] = fifoBuffer[37];
            
           // int16_t ax, ay, az, gx, gy, gz;
            //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
             
            //temperature
            int16_t temperature = mpu.getTemperature();
            imu_msg.teapotData[20] = temperature >> 8;
            imu_msg.teapotData[21] = temperature & 0xFF;

            /// PUBLISH

           // (*nh).loginfo("DONE!");
                
            //pub_imu.publish(&imu_msg);
 #else
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
             #ifdef DEBUG_IMU
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
            #endif
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
             #ifdef DEBUG_IMU
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
            #endif
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
             #ifdef DEBUG_IMU
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            #endif
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
             #ifdef DEBUG_IMU
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
            #endif
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
             #ifdef DEBUG_IMU
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
            #endif
        #endif
    
        #ifdef OUTPUT_TEAPOT
        
          // display quaternion values in InvenSense Teapot demo format:
          teapotPacket[2] = fifoBuffer[0];
          teapotPacket[3] = fifoBuffer[1];
          teapotPacket[4] = fifoBuffer[4];
          teapotPacket[5] = fifoBuffer[5];
          teapotPacket[6] = fifoBuffer[8];
          teapotPacket[7] = fifoBuffer[9];
          teapotPacket[8] = fifoBuffer[12];
          teapotPacket[9] = fifoBuffer[13];
          // gyro values
          teapotPacket[10] = fifoBuffer[16];
          teapotPacket[11] = fifoBuffer[17];
          teapotPacket[12] = fifoBuffer[20];
          teapotPacket[13] = fifoBuffer[21];
          teapotPacket[14] = fifoBuffer[24];
          teapotPacket[15] = fifoBuffer[25];
          // accelerometer values
          teapotPacket[16] = fifoBuffer[28];
          teapotPacket[17] = fifoBuffer[29];
          teapotPacket[18] = fifoBuffer[32];
          teapotPacket[19] = fifoBuffer[33];
          teapotPacket[20] = fifoBuffer[36];
          teapotPacket[21] = fifoBuffer[37];
          //temperature
          int16_t temperature = mpu.getTemperature();
          teapotPacket[22] = temperature >> 8;
          teapotPacket[23] = temperature & 0xFF;
          
          Serial.write(teapotPacket, 28);
          teapotPacket[25]++; // packetCount, loops at 0xFF on purpose
           
        #endif
         
           //  #ifdef DEBUG_IMU
          //  Serial.write(teapotPacket, 14);
          //  #endif
           // teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    #endif
        

        // blink LED to indicate activity
      //  blinkState = !blinkState;
      //  digitalWrite(IMU_LED_PIN, blinkState);
    }
}

#else
  // NO IMU

  void SensorManager::pre_setup_mpu() {
  }
  void SensorManager::setup_mpu() {
  }
  void SensorManager::load_mpu() {
  }
  void SensorManager::init_mpu() {
  }
#endif

// =========== SONAR =================

// HC-SR04

#define MAX_RANGE_CM 200
/*
 * -   FOV (full cone): horizontal ~21º, vertical ~4º
-   Spatial resolution (full cone): ~0.6-1.4º
-   Range: tested from 5 to 200 cm
-   Accuracy: absolute error ~0.035 cm/cm.
-   Precision: standard deviation ~0.1-0.5 cm

Wait 100ms between pings (about 10 pings/sec). 29ms should be the shortest delay between pings.

*/



// ============================

void SensorManager::setup(ros::NodeHandle *nh){
    if (nh!=NULL)
    {
       nh->advertise(pub_range);
       #ifdef IMU_ENABLE
       nh->advertise(pub_imu);
       #endif
    }

     setup_mpu();
}


void SensorManager:: init(ros::NodeHandle *nh)
{
  this->nh=nh;

  // in HZ
  float RANGE_SENSOR_RATE = 10;
  float IMU_SENSOR_RATE = 10;
  if (nh != NULL)
  {
    nh->getParam("~range_sensor_rate", &RANGE_SENSOR_RATE, 1);
    #ifdef IMU_ENABLE
    nh->getParam("~imu_sensor_rate", &IMU_SENSOR_RATE, 1);
    #endif

     range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

     // metri e rad
     range_msg.field_of_view = 0.1;
     range_msg.min_range = 0.02;
     range_msg.max_range = MAX_RANGE_CM * 0.01;
  }
  range_rate_ms = 1000 / RANGE_SENSOR_RATE;
  imu_rate_ms = 1000 / IMU_SENSOR_RATE;

    // dump
  if (nh != NULL)
  {
    dtostrf(range_rate_ms,4,3,tmp_msg1);
    dtostrf(imu_rate_ms,4,3,tmp_msg2);
   
    sprintf(log_msg, "PARAMS: range_rate_ms:%s imu_rate_ms:%s ", tmp_msg1,tmp_msg2);
    (*nh).loginfo(log_msg);
  }
  
  last_time_range = millis();
  last_time_imu = last_time_range;

  const char *input = RANGE_SENSOR_LIST;

  int trig[10];
  int echo[10];

   ultrasonicCount=0;

   // PARSE
   int i=0;
   char *p = (char *)input;
   char *str;
   while ((str = strtok_r(p, ",", &p)) != NULL) 
   {
      //Serial.println(str);
      if (i == 0) trig[ultrasonicCount] = atoi(str);
      if (i == 1) echo[ultrasonicCount] = atoi(str);
      if (i == 2) 
      {
        ultrasonicNames[ultrasonicCount] = new char[strlen(str)+1];
        strcpy(ultrasonicNames[ultrasonicCount],str);
        ultrasonicCount++;
        i=0;
      }
      else
        i++;
   }
  
  #ifdef DEBUG_RANGE

   Serial.print("ultrasonicCount: ");
   Serial.println(ultrasonicCount);
     
  for(int i=0;i<ultrasonicCount;i++)
  {
     Serial.println(trig[i]);
     Serial.println(echo[i]);
     Serial.println(ultrasonicNames[i]);
  }
     
  #endif

  for(int i=0;i<ultrasonicCount;i++)
  {
    //rangeList[i] = new  Ultrasonic(trig[i],echo[i]);  // (Trig PIN,Echo PIN)
    rangeList[i] = new  NewPing(trig[i],echo[i],MAX_RANGE_CM);  // (Trig PIN,Echo PIN)
  }

 init_mpu();
}

SensorManager::~SensorManager()
{
  for(int i=0;i<ultrasonicCount;i++)
  {
    delete rangeList[i];
    delete ultrasonicNames[i];
  }
}


void SensorManager::dump(){
 #ifdef DEBUG_RANGE
  for(int i=0;i<ultrasonicCount;i++)
  {
   Serial.print("READ ");
   Serial.print(ultrasonicNames[i]);
   Serial.print(": ");
   //Serial.print(rangeList[i]->Ranging(CM)); // CM or INC
   Serial.print(rangeList[i]->ping_cm()); // CM or INC
   Serial.println(" cm     " );
  }
  #endif
  
  #ifdef DEBUG_IMU
  load_mpu();
  #endif
}

bool sendIMU = false;

void SensorManager::tick()
{
  long now = millis();
   if ((now - last_time_range) > range_rate_ms) {
      last_time_range = now;

       range_msg.header.stamp = nh->now();
       for(int i=0;i<ultrasonicCount;i++)
       {
          range_msg.header.frame_id =  ultrasonicNames[i];
         // range_msg.range = rangeList[i]->Ranging(CM) ;
          range_msg.range = float(rangeList[i]->ping_cm()) * 0.01; // in metri
          pub_range.publish(&range_msg);
       }
   }

 #ifdef IMU_ENABLE
   if (sendIMU)
   {
      sendIMU=false;

      pub_imu.publish(&imu_msg);
   }
 #endif 

}

void SensorManager::post_tick()
{
  #ifdef IMU_ENABLE
   if ((now - last_time_imu) > imu_rate_ms) {
      last_time_imu = now;
      load_mpu();
      sendIMU=true;

  }
  #endif

}
   

