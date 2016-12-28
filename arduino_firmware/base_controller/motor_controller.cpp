
#include "motor_controller.h"

#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include <husky_base/RoverOdom.h>

#include "MDD10AMotorShieldV1.0.h"
#include "encoder_driver.h"
/* PID parameters and functions */
#include "diff_controller.h"

//#define WHEEL_DIAMETER 0.15
// modificabile dai parametri
//float WHEEL_DIAMETER;

//#define MS_TO_REV (1.0 / (WHEEL_DIAMETER * PI))

/* Convert the rate into an interval */
const int PID_INTERVAL_MS = 1000 / PID_RATE;
  
/* Track the next time we make a PID calculation */
//unsigned long nextPID = PID_INTERVAL_MS;
  
// TODO PARAMS
//#define base_controller_rate 10
//#define accel_limit 0.1


// ====================
// FROM MAIN CODE

void setMotorSpeeds(int leftSpeed, int rightSpeed);

// ====================

void MotorController:: init(ros::NodeHandle *nh)
{
  this->nh=nh;
  
  //lastMotorCommand = AUTO_STOP_INTERVAL;
  moving=0;
  v_left=v_right=v_des_left=v_des_right=0;
  t_last=t_pid=0;
  doPublishSpace=false;

  lastMotorCommand = millis();

  // read params

  WHEEL_DIAMETER = 0.20;
  MAX_ACCELERATION = 3;
  MAX_SPEED = 1;
  POLLING_TIMEOUT = 2;
  //MOTOR_CONTROLLER_RATE = 10;
  
  nh->getParam("~wheel_diameter", &WHEEL_DIAMETER, 1);
  nh->getParam("~max_acceleration", &MAX_ACCELERATION, 1);
  nh->getParam("~max_speed", &MAX_SPEED, 1);
  nh->getParam("~polling_timeout", &POLLING_TIMEOUT, 1);
 // nh->getParam("~motor_controller_rate", &MOTOR_CONTROLLER_RATE, 1);
  //WHEEL_DIAMETER = 0.15;

 
  // t_delta = 1.0 / base_controller_rate;

  //MS_TO_REV  = (1.0 / (WHEEL_DIAMETER * PI));
  
   // How many encoder ticks are there per meter?
  ticks_per_meter = ENCODER_RESOLUTION * GEAR_REDUCTION / (WHEEL_DIAMETER * PI);
  ticks_per_meter_div_RATE = ticks_per_meter / PID_RATE;
  ticks_per_meter_div_RATE_inv = 1.0f / ticks_per_meter_div_RATE;
  
  // What is the maximum acceleration we will tolerate when changing wheel speeds?
  max_acceleration_ticks = MAX_ACCELERATION * ticks_per_meter / PID_RATE;
  auto_stop_interval = POLLING_TIMEOUT * 1000 ;

  // dump
  dtostrf(WHEEL_DIAMETER,4,3,tmp_msg1);
  dtostrf(MAX_ACCELERATION,4,3,tmp_msg2);
  dtostrf(MAX_SPEED,4,3,tmp_msg3);
  sprintf(log_msg, "PARAMS: wheel_diameter:%s max_acceleration:%s max_speed:%s", tmp_msg1,tmp_msg2,tmp_msg3);
  (*nh).loginfo(log_msg);
  
  // INIT

  setPid();
  
  reset_cmd();
}

void MotorController::setPid()
{
  /* PID Parameters */
  Kp = 20;
  Kd = 12;
  Ki = 0;
  Ko = 50;
/*
  Kp = 10;
  Kd = 0;
  Ki = 0;
  Ko = 50;
  */
}

// conversion fun


void MotorController::stopAll()
{
    moving=0;
    setMotorSpeeds(0,0);
}

// in m/sec
void MotorController::setVelocity(float leftSpeed, float rightSpeed) {

    // cut speed
    leftSpeed = (leftSpeed >= 0 ) ?  min(leftSpeed,MAX_SPEED) :  max(leftSpeed,-MAX_SPEED);
    rightSpeed = (rightSpeed >= 0 ) ?  min(rightSpeed,MAX_SPEED) :  max(rightSpeed,-MAX_SPEED);
    
    moving = 1;
    lastMotorCommand = millis();
       
     // cerco il numero di ticks da fare nell'intervallo 
 
    v_des_left = leftSpeed * ticks_per_meter_div_RATE;
    v_des_right = rightSpeed * ticks_per_meter_div_RATE;

    #ifdef ROS_LOG_ENABLED   
    sprintf(log_msg, "setVelocity (%d , %d)", v_des_left,v_des_right);
    (*nh).loginfo(log_msg);
   #endif
}


void MotorController::tick()
{
    long now = millis();
    if ((now - lastMotorCommand) > auto_stop_interval) {;
      setMotorSpeeds(0, 0);
      moving = 0;
    }
   
    if (moving == 0) return;

    long t_delta = now - t_last;
    
    if (t_delta > PID_INTERVAL_MS) 
    {
      // aggiorno il PID
      t_pid = t_delta;
      t_last= now;
      doPublishSpace=true;
      
      if (v_left < v_des_left)
      {
                  v_left += max_acceleration_ticks;
                  if (v_left > v_des_left)
                      v_left = v_des_left;
      }
      else{
                  v_left -= max_acceleration_ticks;
                  if (v_left < v_des_left)
                      v_left = v_des_left;
      }
              
       
      if (v_right < v_des_right)
      {
                  v_right += max_acceleration_ticks;
                  if (v_right > v_des_right)
                      v_right = v_des_right;
      }
      else
      {
                  v_right -= max_acceleration_ticks;
                  if (v_right < v_des_right)
                      v_right = v_des_right;
      }
        
      // DEBUG
      // v_left = v_des_left;
      //  v_right = v_des_right;
    
      //#ifdef ROS_LOG_ENABLED   
      // sprintf(log_msg, "set motor (%d , %d)", v_left,v_right);
      // (*nh).loginfo(log_msg);
      // #endif

      // update

      if (v_left == 0 && v_right ==0)
      {
        // STOP
        setMotorSpeeds(0, 0);
        moving = 0;
      }
      else
      {
      
        leftPID.TargetTicksPerFrame = v_left;//leftTicksPerFrame;
        rightPID.TargetTicksPerFrame = v_right;//rightTicksPerFrame;
      }
      updatePID(t_pid);


      //  Set motor speeds in encoder ticks per PID loop
      //  move_cmd(v_left, v_right);
      leftPID.TargetTicksPerFrame = v_left;//leftTicksPerFrame;
      rightPID.TargetTicksPerFrame = v_right;//rightTicksPerFrame;
    }
}


// ========================


void MotorController::reset_cmd()
{
  left_pos_tick=0;
  right_pos_tick=0;
    resetEncoders();
    resetPID();
}

void MotorController::publishSpace(ros::Publisher &odom_pub)
{
  if (!doPublishSpace) return;
  doPublishSpace=false;
  
  husky_base::RoverOdom odom_msg;

  //odom_msg.header.stamp =  (*nh).now();
  //odom_msg.header.frame_id = "/petrorov/base/odom";

  long enc_left = leftPID.Delta ;
  long enc_right = rightPID.Delta;
  
  // dentro linear ci metto la velocità in metri al secondo

  odom_msg.left_speed = ticks_per_meter_div_RATE_inv * (enc_left) ;
  odom_msg.right_speed = ticks_per_meter_div_RATE_inv * (enc_right);
  
 
  //TODO ottimizzato
  /*
   * // dentro angular la posizione del frame in radianti 0 -2*PI
  left_pos_tick = readEncoder(MOTOR_LEFT) % ENCODER_RESOLUTION;
  right_pos_tick = readEncoder(MOTOR_RIGHT) % ENCODER_RESOLUTION;
  if (left_pos_tick < 0 ) left_pos_tick += ENCODER_RESOLUTION;
  if (right_pos_tick < 0 ) right_pos_tick += ENCODER_RESOLUTION;
  */

  //  dentro angular la posizione del motore in metri


  odom_msg.left_position = float(readEncoder(MOTOR_LEFT))  / ticks_per_meter ;
  odom_msg.right_position = float(readEncoder(MOTOR_RIGHT)) / ticks_per_meter;
  
  // dentro z il DT
  
  odom_msg.lastPidTime= t_pid;
  odom_pub.publish(&odom_msg);
}




