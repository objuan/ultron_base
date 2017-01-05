
#include "sensors.h"
#include <sensor_msgs/Range.h>


sensor_msgs::Range range_msg;
ros::Publisher pub_range( ROS_TOPIC_RANGE_SENSOR, &range_msg);

void SensorManager::setup(ros::NodeHandle *nh){
     nh->advertise(pub_range);
}

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
 

void SensorManager:: init(ros::NodeHandle *nh)
{
  this->nh=nh;

  // in HZ
  float RANGE_SENSOR_RATE = 10;
  if (nh != NULL)
  {
    nh->getParam("~range_sensor_rate", &RANGE_SENSOR_RATE, 1);

 
     range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

     // metri e rad
     range_msg.field_of_view = 0.1;
     range_msg.min_range = 0.02;
     range_msg.max_range = MAX_RANGE_CM * 0.01;
  }
  polling_rate_ms = 1000 / RANGE_SENSOR_RATE;
  lastTime = millis();

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
  for(int i=0;i<ultrasonicCount;i++)
  {
   Serial.print("READ ");
   Serial.print(ultrasonicNames[i]);
   Serial.print(": ");
   //Serial.print(rangeList[i]->Ranging(CM)); // CM or INC
   Serial.print(rangeList[i]->ping_cm()); // CM or INC
   Serial.println(" cm     " );
  }
}

void SensorManager::tick()
{
   long now = millis();
   if ((now - lastTime) > polling_rate_ms) {
      lastTime = now;

       range_msg.header.stamp = nh->now();
       for(int i=0;i<ultrasonicCount;i++)
       {
          range_msg.header.frame_id =  ultrasonicNames[i];
         // range_msg.range = rangeList[i]->Ranging(CM) ;
          range_msg.range = float(rangeList[i]->ping_cm()) * 0.01; // in metri
          pub_range.publish(&range_msg);
       }
   }
}
   

