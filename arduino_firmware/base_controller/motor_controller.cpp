
#include "motor_controller.h"

#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include <ultron_kernel/RobotOdom.h>
#include <ultron_kernel/RobotSpeed.h>

#include "MDD10AMotorShieldV1.0.h"
#include "encoder_driver.h"
/* PID parameters and functions */
#include "diff_controller.h"

//#define WHEEL_DIAMETER 0.15
// modificabile dai parametri
//float WHEEL_DIAMETER;

//#define MS_TO_REV (1.0 / (WHEEL_DIAMETER * PI))

/* Convert the rate into an interval */
//const int PID_INTERVAL_MS = 1000 / PID_RATE;
  
/* Track the next time we make a PID calculation */
//unsigned long nextPID = PID_INTERVAL_MS;
  
// TODO PARAMS
//#define base_controller_rate 10
//#define accel_limit 0.1

MDD10AMotorShield drive_left(FRONT_LEFT_DIR,  FRONT_LEFT_PWM,   BACK_LEFT_DIR,  BACK_LEFT_PWM);

MDD10AMotorShield drive_right(FRONT_RIGHT_DIR,FRONT_RIGHT_PWM,  BACK_RIGHT_DIR, BACK_RIGHT_PWM);

/* PID Parameters */
extern int Kp ;
extern int Kd ;
extern int Ki ;
extern int Ko ;

// ====================
// FROM MAIN CODE
 ros::NodeHandle *gnh;
 
/* Wrap the drive motor set speed function */
void setMotorSpeed(int motorID, int spd) {

 /* #ifdef DEBUG_MOTOR
  Serial.print("SPEED: ");
  Serial.print(motorID);
  Serial.print(" ");
  Serial.println(spd);
      
  #endif
  */
  
  if (motorID == MOTOR_FRONT_LEFT) 
    drive_left.setM1Speed(spd * MOTOR_LEFT_SIGN);
  else  if (motorID == MOTOR_BACK_LEFT) 
    drive_left.setM2Speed(spd * MOTOR_LEFT_SIGN);
  else if (motorID == MOTOR_FRONT_RIGHT) 
    drive_right.setM1Speed(spd * MOTOR_RIGHT_SIGN);
  else if (motorID == MOTOR_BACK_RIGHT) 
    drive_right.setM2Speed(spd * MOTOR_RIGHT_SIGN);
}

// A convenience function for setting both motor speeds
// Set speed for motor 1, speed is a number between -MOTOR_INPUT_LIMIT and MOTOR_INPUT_LIMIT
void setMotorSpeeds(int leftFrontSpeed, int leftBackSpeed, int rightFrontSpeed,int rightBackSpeed)
{
  drive_left.setM1Speed(leftFrontSpeed * MOTOR_LEFT_SIGN);
  drive_left.setM2Speed(leftBackSpeed * MOTOR_LEFT_SIGN);
  drive_right.setM1Speed(rightFrontSpeed * MOTOR_LEFT_SIGN);
  drive_right.setM2Speed(rightBackSpeed * MOTOR_LEFT_SIGN);
  
#ifdef ROS_LOG_ENABLED  
        dtostrf(leftFrontSpeed * MOTOR_LEFT_SIGN,4,3,tmp_msg1);
        dtostrf(leftBackSpeed * MOTOR_LEFT_SIGN,4,3,tmp_msg2);
  
        sprintf(log_msg, "MOTOR LEFT (%s , %s)", tmp_msg1,tmp_msg2);
        gnh->loginfo(log_msg);
#endif

   // setMotorSpeed(MOTOR_LEFT, leftSpeed);
   // setMotorSpeed(MOTOR_RIGHT, rightSpeed);
}

void MotorController::setRawMotorSpeed(int motorID, int speed) {
  setMotorSpeed(motorID,speed);  
}


// ====================

float PID_INTERVAL_FLOAT;

// ====================

void MotorController:: init(ros::NodeHandle *nh)
{
  this->nh=nh;
  gnh = nh;
  //lastMotorCommand = AUTO_STOP_INTERVAL;
  moving=0;
  for(int i=0;i<4;i++){ v_des_speed[i]=0; v_speed[i]=0;}
  t_last_motor=t_last_odom=t_pid=0;
  //doPublishSpace=false;

  lastMotorCommand = millis();

  // read params

  WHEEL_DIAMETER = 0.20;
  MAX_ACCELERATION = 3;
  MAX_SPEED = 1;
  POLLING_TIMEOUT = 2;
  MOTOR_CONTROLLER_RATE = 10;
  ODOM_STATE_RATE = 10;

  if (nh != NULL)
  {
    nh->getParam("~wheel_diameter", &WHEEL_DIAMETER, 1);
    nh->getParam("~max_acceleration", &MAX_ACCELERATION, 1);
    nh->getParam("~max_speed", &MAX_SPEED, 1);
    nh->getParam("~polling_timeout", &POLLING_TIMEOUT, 1);
    nh->getParam("~motor_controller_rate", &MOTOR_CONTROLLER_RATE, 1);
    nh->getParam("~odom_state_rate", &ODOM_STATE_RATE, 1);

     nh->getParam("~Kp", &Kp, 1);
     nh->getParam("~Kd", &Kd, 1);
     nh->getParam("~Ki", &Ki, 1);

    //WHEEL_DIAMETER = 0.15;
  }
 
  // t_delta = 1.0 / base_controller_rate;

  //MS_TO_REV  = (1.0 / (WHEEL_DIAMETER * PI));
  motor_controller_rate_ms = 1000 / MOTOR_CONTROLLER_RATE;
  odom_rate_ms = 1000 / ODOM_STATE_RATE;
  
  PID_INTERVAL_FLOAT = 1000 / MOTOR_CONTROLLER_RATE;
  
   // How many encoder ticks are there per meter?
  ticks_per_meter = ENCODER_RESOLUTION * GEAR_REDUCTION / (WHEEL_DIAMETER * PI);
  ticks_per_meter_div_RATE = ticks_per_meter / MOTOR_CONTROLLER_RATE;
  ticks_per_meter_div_RATE_inv = 1.0f / ticks_per_meter_div_RATE;
  
  // What is the maximum acceleration we will tolerate when changing wheel speeds?
  max_acceleration_ticks = MAX_ACCELERATION * ticks_per_meter / MOTOR_CONTROLLER_RATE;
  auto_stop_interval = POLLING_TIMEOUT * 1000 ;

  // dump
  if (nh != NULL)
  {
    dtostrf(MOTOR_CONTROLLER_RATE,4,3,tmp_msg1);
    dtostrf(POLLING_TIMEOUT,4,3,tmp_msg2);
    sprintf(log_msg, "PARAMS: motor_rate:%s polling_timeout:%s ", tmp_msg1,tmp_msg2);
    (*nh).loginfo(log_msg);
    
    dtostrf(WHEEL_DIAMETER,4,3,tmp_msg1);
    dtostrf(MAX_ACCELERATION,4,3,tmp_msg2);
    dtostrf(MAX_SPEED,4,3,tmp_msg3);
    sprintf(log_msg, "PARAMS: wheel_diameter:%s max_acceleration:%s max_speed:%s", tmp_msg1,tmp_msg2,tmp_msg3);
    (*nh).loginfo(log_msg);

    sprintf(log_msg, "PARAMS: Kp:%d Kd:%d Ki:%d", Kp,Kd,Ki);
    (*nh).loginfo(log_msg);
  }
  
  // INIT

  //setPid();
  
  reset_cmd();

  // ports
  drive_left.init(nh);
  drive_right.init(nh);

  initEncoders();
}

/*
void MotorController::setPid()
{
  Kp = 20;
  Kd = 12;
  Ki = 0;
  Ko = 50;
}
*/
// conversion fun


void MotorController::stopAll()
{
    moving=0;
    setMotorSpeeds(0,0,0,0);
}

// in m/sec
void MotorController::setVelocity(const float *speed) {

/*
 #ifdef ROS_LOG_ENABLED
  dtostrf(speed[0],4,3,tmp_msg1);
  dtostrf(speed[1],4,3,tmp_msg2);
  
  sprintf(log_msg, "cmd_vel_callback LEFT (%s , %s)", tmp_msg1,tmp_msg2);
nh->loginfo(log_msg);

   dtostrf(speed[2],4,3,tmp_msg1);
  dtostrf(speed[3],4,3,tmp_msg2);
  
  sprintf(log_msg, "cmd_vel_callback RIGHT (%s , %s)", tmp_msg1,tmp_msg2);
  nh->loginfo(log_msg);
#endif
*/
    
    // cut speed
    for(int i=0;i<4;i++)
    {
      float _speed = (speed[i] >= 0 ) ?  min(speed[i],MAX_SPEED) :  max(speed[i],-MAX_SPEED);

      v_des_speed[i] = _speed * ticks_per_meter_div_RATE;
    }

    lastMotorCommand = millis();
       
     // cerco il numero di ticks da fare nell'intervallo 
 
    //v_des_left = leftSpeed * ticks_per_meter_div_RATE;
   // v_des_right = rightSpeed * ticks_per_meter_div_RATE;

    #ifdef ROS_LOG_ENABLED   
     dtostrf(v_des_speed[0],4,3,tmp_msg1);
    dtostrf(v_des_speed[1],4,3,tmp_msg2);
  
    sprintf(log_msg, "cmd_vel_callback LEFT (%s , %s)", tmp_msg1,tmp_msg2);
  
    (*nh).loginfo(log_msg);
   #endif
   
}


void MotorController::tick()
{
    long now = millis();

    int newMoving = (v_des_speed[0] != 0 || v_des_speed[1] != 0 || v_des_speed[2] != 0 || v_des_speed[3] != 0 );

    if (newMoving != moving)
    {
        moving = newMoving;

     //   sprintf(log_msg, "CHANGE (%d)", moving);
     //   nh->loginfo(log_msg);
 
        if (newMoving == 0)
        {
           // mi fermo
           setMotorSpeeds(0, 0,0,0);
           updatePID(0,nh);
        }
        
    }

   
    if (moving == 1)
    {
      // TIME OUT ??
      if ((now - lastMotorCommand) > auto_stop_interval) {;
        setMotorSpeeds(0, 0,0,0);
        moving = 0;
        updatePID(0,nh);
      }
     }
   
    if (moving == 0) return;

   // nh->loginfo("d");
   
    long t_delta = now - t_last_motor;
    
    if (t_delta > motor_controller_rate_ms) 
    {
      // aggiorno il PID
      t_pid = t_delta;
      t_last_motor= now;

        for(int i=0;i<4;i++)
        {
          if (v_speed[i] < v_des_speed[i])
          {
                      v_speed[i] += max_acceleration_ticks;
                      if (v_speed[i] > v_des_speed[i])
                          v_speed[i] = v_des_speed[i];
          }
          else{
                      v_speed[i] -= max_acceleration_ticks;
                      if (v_speed[i] < v_des_speed[i])
                          v_speed[i] = v_des_speed[i];
          }
  
        }
      
#if 0
        dtostrf(v_speed[0],4,3,tmp_msg1);
        dtostrf(v_speed[1],4,3,tmp_msg2);
  
        sprintf(log_msg, "SET LEFT (%s , %s)", tmp_msg1,tmp_msg2);
        nh->loginfo(log_msg);
#endif

        motorPID[0].TargetTicksPerFrame = v_speed[0];//leftTicksPerFrame;
        motorPID[1].TargetTicksPerFrame = v_speed[1];//leftTicksPerFrame;
        motorPID[2].TargetTicksPerFrame = v_speed[2];//rightTicksPerFrame;
        motorPID[3].TargetTicksPerFrame = v_speed[3];//rightTicksPerFrame;

        updatePID(t_pid,nh);
        
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
  //if (!doPublishSpace) return;
  //doPublishSpace=false;
   long now = millis();
    
   if ((now - t_last_odom) > odom_rate_ms) {
    
    t_last_odom = now;

     ultron_kernel::RobotOdom odom_msg;
  /*
    //odom_msg.header.stamp =  (*nh).now();
    //odom_msg.header.frame_id = "/petrorov/base/odom";
  
    long enc_left = motorPID[0].Delta ;
    long enc_right = rightPID.Delta;
    
    // dentro linear ci metto la velocità in metri al secondo
  
    odom_msg.left_speed = ticks_per_meter_div_RATE_inv * (enc_left) ;
    odom_msg.right_speed = ticks_per_meter_div_RATE_inv * (enc_right);
    
   */
    //TODO ottimizzato
    /*
     * // dentro angular la posizione del frame in radianti 0 -2*PI
    left_pos_tick = readEncoder(MOTOR_LEFT) % ENCODER_RESOLUTION;
    right_pos_tick = readEncoder(MOTOR_RIGHT) % ENCODER_RESOLUTION;
    if (left_pos_tick < 0 ) left_pos_tick += ENCODER_RESOLUTION;
    if (right_pos_tick < 0 ) right_pos_tick += ENCODER_RESOLUTION;
    */
  
    //  dentro angular la posizione del motore in metri
  
  //TODO
   // odom_msg.left_position = float(readEncoder(MOTOR_LEFT))  / ticks_per_meter ;
    //odom_msg.right_position = float(readEncoder(MOTOR_RIGHT)) / ticks_per_meter;
    
    // dentro z il DT
    for(int i=0;i<4;i++)
    {
      odom_msg.speed[i] = ticks_per_meter_div_RATE_inv * (motorPID[i].Delta ) ;
      odom_msg.position[i] =  float(motorPID[i].Encoder)  / ticks_per_meter ;
    }
    
    odom_msg.lastPidTime= t_pid;
    
    odom_pub.publish(&odom_msg);
   }
}

void MotorController::getDataEncoders( ultron_kernel::GetDataEncoders::Response &res){
  //TODO
  res.left_encoder_pos = (readEncoder(MOTOR_FRONT_LEFT))  ;
  res.right_encoder_pos = (readEncoder(MOTOR_FRONT_RIGHT));
  
}




