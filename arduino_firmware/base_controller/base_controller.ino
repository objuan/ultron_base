#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <husky_base/GetRoverInfo.h>
#include <husky_base/RoverOdom.h>
#include <husky_base/ResetPosition.h>

#include "config.h"
#include "motor_controller.h"

// ----------------

ros::NodeHandle nh;

char log_msg[1000];
char tmp_msg1[100];
char tmp_msg2[100];
char tmp_msg3[100];

float DIAGNOSTIC_RATE;
long lastDiagnosticTime;
long diagnostic_rate_ms;

// ----------------
// BATTERY
// ----------------

float R1 = 20000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) – see text!
#include "batteryState.h"

sensor_msgs::BatteryState batteryState_msg;
ros::Publisher battery_pub1("/petrorov/base/bat1_value", &batteryState_msg);
ros::Publisher battery_pub2("/petrorov/base/bat2_value", &batteryState_msg);

// definizione delle baytterie
BatteryState battery_1("BAT_LIPO_1",BATTERY_PIN_ANAG_A, 1.0 / (R2/(R1+R2) ),sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO);
BatteryState battery_2("BAT_NUMH_2",BATTERY_PIN_ANAG_B, 1.0 / (R2/(R1+R2) ),sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH);

// ----------------
//  ENCODERS
// ----------------

#include "encoder_driver.h"

// ----------------
// MOTOR
// ----------------

#include "MDD10AMotorShieldV1.0.h"

MDD10AMotorShield drive(LEFT_ENG_DIR,LEFT_ENG_PWM, RIGHT_ENG_DIR,RIGHT_ENG_PWM);

// FROM DIFFERENCIAL API

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == MOTOR_LEFT) 
    drive.setM1Speed(spd * MOTOR_LEFT_SIGN);
  else 
    drive.setM2Speed(spd * MOTOR_RIGHT_SIGN);
}
  
// A convenience function for setting both motor speeds
// Set speed for motor 1, speed is a number between -MOTOR_INPUT_LIMIT and MOTOR_INPUT_LIMIT
void setMotorSpeeds(int leftSpeed, int rightSpeed) {

#ifdef ROS_LOG_ENABLED  
  //sprintf(log_msg, "setMotorSpeeds (%d , %d)", leftSpeed,rightSpeed);
  //nh.loginfo(log_msg);
#endif

    setMotorSpeed(MOTOR_LEFT, leftSpeed);
    setMotorSpeed(MOTOR_RIGHT, rightSpeed);
}
  
// ----------------
//  SENSORS
// ----------------

rosserial_arduino::Adc adc_msg;
ros::Publisher sensor_pub(ROS_TOPIC_SENSOR, &adc_msg);

// can be used for getting rid of noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

// ----------------
//  MOTOR 
// ----------------
/*
 * La velocità arriva in metri/secondo, devo convertire questa velocità in 
 */
husky_base::RoverOdom odom_msg;
ros::Publisher motor_state_pub(ROS_TOPIC_MOTOR_STATE, &odom_msg);

MotorController motorController;

// The callback function
// the message contains the velocity of left and right motors
// twist_msg.linear.x = LEFT MOTOR
// twist_msg.linear.y = RIGHT MOTOR
// values are in m/sec
void cmd_vel_callback( const husky_base::RoverOdom& move_msg) {
 #ifdef ROS_LOG_ENABLED
  dtostrf(move_msg.left_speed,4,3,tmp_msg1);
  dtostrf(move_msg.right_speed,4,3,tmp_msg2);
  
 // sprintf(log_msg, "cmd_vel_callback (%s , %s)", tmp_msg1,tmp_msg2);
  //nh.loginfo(log_msg);
#endif

  motorController.setVelocity(move_msg.left_speed,move_msg.right_speed);
}

ros::Subscriber<husky_base::RoverOdom> cmd_vel_topic(ROS_TOPIC_VELOCITY_IN, &cmd_vel_callback);

// ----------------
//  SERVICE
// ----------------

// GET INFO SERVICE
const char* rover_name = ROVER_NAME;

void GetRoverInfo_callback(const husky_base::GetRoverInfo::Request & req, husky_base::GetRoverInfo::Response & res){
  res.info.name=rover_name; // ARDUINO STRING WORKAROUND
  res.info.wheel_diameter = motorController.WHEEL_DIAMETER;
  
  //nh.loginfo("GetRoverInfo_callback");
}

ros::ServiceServer<husky_base::GetRoverInfo::Request, husky_base::GetRoverInfo::Response> state_srv(ROS_TOPIC_GET_INFO_SRV,&GetRoverInfo_callback);


// RESET SERVICE

void ResetPosition_callback(const husky_base::ResetPosition::Request & req, husky_base::ResetPosition::Response & res){
    
  motorController.reset_cmd();
  nh.loginfo("ResetPosition_callback");
}

ros::ServiceServer<husky_base::ResetPosition::Request, husky_base::ResetPosition::Response> reset_position_srv(ROS_TOPIC_RESET_POS_SRV,&ResetPosition_callback);

// STOP SERVICE

void Stop_callback(const husky_base::ResetPosition::Request & req, husky_base::ResetPosition::Response & res){
    
  motorController.stopAll();
  nh.loginfo("Stop_callback");
}

ros::ServiceServer<husky_base::ResetPosition::Request, husky_base::ResetPosition::Response> stop_srv(ROS_TOPIC_STOP_SRV,&Stop_callback);


// ----------------
//  Service Client
// ----------------

ros::ServiceClient<husky_base::ResetPosition::Request, husky_base::ResetPosition::Response> connect_client(ROS_TOPIC_CONNECT);

// ----------------
//  setup
// ----------------

long adc_timer;

void setup() {

#ifdef DEBUG_MODE
  Serial.begin(BAUDRATE);
  Serial.println("Init Message Test");
  Serial.println("DEBUG MODE");
 #endif

#ifdef DEBUG_MODE
  drive.init();
  return;
 #endif
  
  /// ROS

  nh.getHardware()->setBaud(BAUDRATE);
  
  nh.initNode();

  nh.loginfo("PETROROV Serial setup");

  //// GENERAL SETTING
 
 // motorController.init(&nh);
  
  // PUBBLICAZIONI
  
  nh.advertise(sensor_pub);
  
  nh.advertise(battery_pub1);
  nh.advertise(battery_pub2);

  nh.advertise(motor_state_pub);

  // SOTTOSCIZIONI

  nh.subscribe(cmd_vel_topic);

  // SERVICE

  nh.advertiseService(state_srv);
  nh.advertiseService(reset_position_srv);
  nh.advertiseService(stop_srv);

  nh.serviceClient(connect_client);

  // info
  nh.loginfo("PETROROV Serial setup complete");
}

// ----------------
//  loop_debug
// ----------------

void loop_debug(){


#ifdef DEBUG_MOTOR

  int DEBUG_SIDE = MOTOR_RIGHT;

  for(int i=0;i<255;i+=3)
  {
      setMotorSpeed(DEBUG_SIDE, i);
      //setMotorSpeed(RIGHT, i);
      Serial.print("MOTOR SET");
      Serial.println(i);

#ifdef DEBUG_ENC
      long enc = readEncoder(DEBUG_SIDE);
      Serial.print("ENV VALUE ");
      Serial.println(enc);
#endif
      delay(100);

  }
      
#endif



#ifdef DEBUG_BATTERY

 // battery

  float bat1 = battery_1.readValue();
  float bat2 = battery_2.readValue();

  Serial.println(battery_1.toString());
  Serial.println(battery_2.toString());
  
#endif

}


bool _firstLoop = true;


//  chiamato la prima volta che e' connesso
// ----------------
// first_loop
// ----------------

void first_loop() {

  DIAGNOSTIC_RATE= 1;
  nh.getParam("~diagnostic_rate", &DIAGNOSTIC_RATE, 1);
  diagnostic_rate_ms = DIAGNOSTIC_RATE * 1000;
  lastDiagnosticTime = millis();

  //MOTOR
  motorController.init(&nh);
  
    // ports
  drive.init(&nh);

  nh.loginfo("Robot setup complete");

  // params

  
    // call parent service
/*
    husky_base::ResetPosition::Request request;
    husky_base::ResetPosition::Response response;

    connect_client.call(request,response);
    bool ok = response.retCode;

    nh.spinOnce();
    delay(100);
    nh.loginfo("Robot send complete");
*/
}


// ----------------
//  loop
// ----------------

void loop() {

#ifdef DEBUG_MODE

  loop_debug();

  return;
#endif

  //wait until you are actually connected
  if (_firstLoop)
  {
    if (nh.connected())
     {
       _firstLoop=false;
        first_loop();
     }
     else
      nh.spinOnce();
    return;
  }
  
  // =====================
/*

*/

  //MOTOR

  motorController.tick();

  // PUB STATE

  motorController.publishSpace(motor_state_pub);
  
  // DIAGNOSTIC

   long now = millis();
   if ((now - lastDiagnosticTime) > diagnostic_rate_ms) {
      lastDiagnosticTime = now;
      float bat1 = battery_1.readValue();
      float bat2 = battery_2.readValue();

      sensor_msgs::BatteryState &batteryState_msg1 = battery_1.getMessage();
      battery_pub1.publish(&batteryState_msg1);

      sensor_msgs::BatteryState &batteryState_msg2 = battery_2.getMessage();
      battery_pub2.publish(&batteryState_msg2);
    }
    
  // ------------------

  /*
  adc_msg.adc0 = averageAnalog(0);
  
  sensor_pub.publish(&adc_msg);
  */
  
  nh.spinOnce();

 

 // delay(500);
}
