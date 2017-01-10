#ifndef MotorController_h
#define MotorController_h

#include "config.h"
#include <ros.h>
#include <ultron_kernel/GetDataEncoders.h>

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class MotorController
{
  public:

  // PARAMS
    float WHEEL_DIAMETER;
    float MAX_SPEED;
    float MAX_ACCELERATION;
    // seconds
    float MOTOR_RATE;
    float POLLING_TIMEOUT;
    
    //float MOTOR_CONTROLLER_RATE;
    //float MS_TO_REV;

private:
    
    long lastMotorCommand;
    float ticks_per_meter;

    long motor_rate_ms;
    float ticks_per_meter_div_RATE;
    float ticks_per_meter_div_RATE_inv;
    
    long t_pid,t_last;
    bool doPublishSpace;


    float max_acceleration_ticks;
    float auto_stop_interval;

   // bool stopped;
  
    int v_des_left;
    int v_des_right;

    int v_left;
    int v_right;

   // posizione dei motori modulo ENCODER_RESOLUTION
    long left_pos_tick;
    long right_pos_tick;

    ros::NodeHandle *nh;
    
  public:  

    MotorController(){
    }

    void init(ros::NodeHandle *nh);
    
    void setPid();

    // commands

    void stopAll();
    
    // in m/sec
    void setVelocity(float leftSpeedMS, float rightSpeedMS);

  
    void tick();
    
    // action 

    void reset_cmd();

    // publish
    
    void publishSpace(ros::Publisher &odom_pub);
    
    // service

    void getDataEncoders( ultron_kernel::GetDataEncoders::Response &res);
    
};

#endif
