#ifndef SensorManager_h
#define SensorManager_h

#include "config.h"
#include <ros.h>
//#include <Ultrasonic.h>
#include <NewPing.h>

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class SensorManager
{

private:

    ros::NodeHandle *nh;

  //  Ultrasonic* rangeList[10];
    NewPing* rangeList[10];
    int ultrasonicCount;
    char *ultrasonicNames[10];

    long last_time_range;
    long last_time_imu;
    long range_rate_ms;
    long imu_rate_ms;
    
  public:  

    SensorManager(){
    }
    ~SensorManager();

    void pre_setup_mpu();
    void setup_mpu();
    void init_mpu();
    void load_mpu();
  
    void setup(ros::NodeHandle *nh);
    void init(ros::NodeHandle *nh);

    void dump();

    void tick(long now);
    void post_tick(long now);
    
};

#endif
