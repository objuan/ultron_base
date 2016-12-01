#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>

#include "config.h"

// ----------------

ros::NodeHandle nh;

// ----------------
// BATTERY
// ----------------

float R1 = 20000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) – see text!
#include "batteryState.h"

sensor_msgs::BatteryState batteryState_msg;
ros::Publisher battery_pub1("bat1_value", &batteryState_msg);
ros::Publisher battery_pub2("bat2_value", &batteryState_msg);

// definizione delle baytterie
BatteryState battery_1("BAT_LIPO_1",A0, 1.0 / (R2/(R1+R2) ),sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
BatteryState battery_2("BAT_NUMH_2",A1, 1.0 / (R2/(R1+R2) ),sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH);

// ----------------
//  ENCODERS
// ----------------

#include "encoder_driver.h"

// ----------------
// MOTOR
// ----------------

#include "MDD10AMotorShieldV1.0.h"

/* PID parameters and functions */
#include "diff_controller.h"

MDD10AMotorShield drive(LEFT_ENG_DIR,LEFT_ENG_PWM, RIGHT_ENG_DIR,RIGHT_ENG_PWM);

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;
  
/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

// FROM DIFFERENCIAL API

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) 
  drive.setM1Speed(spd);
  else 
  drive.setM2Speed(spd);
}
  
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
}
  
// ----------------
//  ROS
// ----------------

rosserial_arduino::Adc adc_msg;
ros::Publisher sensor_pub("sensor_value", &adc_msg);

geometry_msgs::Twist twist_msg;
ros::Publisher twist_pub("ultron/cmd_vel", &twist_msg);

// can be used for getting rid of noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

// =================
// COMMANDS
// =================

//int moving=0;
 /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
long lastMotorCommand = AUTO_STOP_INTERVAL;
  
void twist_cmd(int leftTicksPerFrame,int rightTicksPerFrame)
{
    lastMotorCommand = millis();
    if (leftTicksPerFrame == 0 && rightTicksPerFrame == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else 
      moving = 1;
    leftPID.TargetTicksPerFrame = leftTicksPerFrame;
    rightPID.TargetTicksPerFrame = rightTicksPerFrame;
    
    #ifdef DEBUG_MODE
    Serial.println(arg1); 
    Serial.println(arg2); 
    #endif
    //Serial.println("OK"); 
}

void reset_cmd()
{
    resetEncoders();
    resetPID();
}


// ----------------
//  setup
// ----------------

long adc_timer;

void setup() {

#ifdef DEBUG_SERIAL_MODE
  Serial.begin(BAUDRATE);
  Serial.println("Init Message Test");
 #endif

// MOTOR
  drive.init();

  /// ROS
  nh.initNode();
  nh.advertise(sensor_pub);
  
  nh.advertise(battery_pub1);
  nh.advertise(battery_pub2);

  nh.advertise(twist_pub);

}

// ----------------
//  loop
// ----------------

void loop() {

  // battery

  float bat1 = battery_1.readValue();
  float bat2 = battery_2.readValue();
  
  sensor_msgs::BatteryState &batteryState_msg1 = battery_1.getMessage();
  battery_pub1.publish(&batteryState_msg1);

  sensor_msgs::BatteryState &batteryState_msg2 = battery_2.getMessage();
  battery_pub2.publish(&batteryState_msg2);

  //MOTOR
/*
   if(digitalRead(8)==1)
 msg.linear.x=-0.25;
 
 else if (digitalRead(4)==1)
 msg.linear.x=0.25;
 
 else if (digitalRead(8)==0 && digitalRead(4)==0)
 msg.linear.x=0;
 
 twist_pub.publish(&msg);
 */
  // ------------------
  
  adc_msg.adc0 = averageAnalog(0);
  
  sensor_pub.publish(&adc_msg);
  nh.spinOnce();

 
#ifdef DEBUG_SERIAL_MODE

  Serial.println(battery_1.toString());
  Serial.println(battery_2.toString());
  
#endif

 // delay(500);
}
