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

    long lastTime;
    long polling_rate_ms;
    
  public:  

    SensorManager(){
    }
    ~SensorManager();

    void setup(ros::NodeHandle *nh);
    void init(ros::NodeHandle *nh);

    void dump();

    void tick();
    
};

#endif
