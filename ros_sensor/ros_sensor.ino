#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>



ros::NodeHandle nh;
rosserial_arduino::Adc adc_msg;
ros::Publisher sensor_pub("sensor_value", &adc_msg);

// can be used for getting rid of noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;


void setup() {
  nh.initNode();
  nh.advertise(sensor_pub);

}

void loop() {

  adc_msg.adc0 = averageAnalog(0);
  
  sensor_pub.publish(&adc_msg);
  nh.spinOnce();

}