

#include "ultron_kernel/horizon_legacy/clearpath.h"
#include <ros/ros.h>

namespace
{
  std::string port_;
}

namespace horizon_legacy
{

  void reconnect()
  {
    if (port_.empty())
    {
      throw std::logic_error("Can't reconnect when port is not configured");
    }
    ROS_INFO_STREAM("Connecting to Ultron on port " << port_ << "...");
    clearpath::Transport::instance().configure(port_.c_str(), 3);
    ROS_INFO("Connected");
  }

  void connect(std::string port)
  {
    port_ = port;
    reconnect();
  }

  void configureLimits(double max_speed, double max_accel)
  {

    bool success = false;
    while (!success)
    {
      try
      {
        clearpath::SetMaxAccel(max_accel, max_accel).send();
        clearpath::SetMaxSpeed(max_speed, max_speed).send();
        success = true;
      }
      catch (clearpath::Exception *ex)
      {
        ROS_ERROR_STREAM("Error configuring velocity and accel limits: " << ex->message);
        reconnect();
      }
    }
  }

  void controlSpeed(double speed_left, double speed_right, double accel_left, double accel_right)
  {
    bool success = false;
    while (!success)
    {
      try
      {
        clearpath::SetDifferentialSpeed(speed_left, speed_right, accel_left, accel_right).send();
        success = true;
      }
      catch (clearpath::Exception *ex)
      {
        ROS_ERROR_STREAM("Error sending speed and accel command: " << ex->message);
        reconnect();
      }
    }
  }

}
