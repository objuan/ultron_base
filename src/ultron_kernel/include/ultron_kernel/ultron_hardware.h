
#ifndef ultron_kernel_ultron_HARDWARE_H
#define ultron_kernel_ultron_HARDWARE_H

#include "ultron_kernel/ultron_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ultron_kernel/RobotStatus.h"
#include <string>

#include <geometry_msgs/Twist.h>
#include <ultron_kernel/GetRobotInfo.h>
#include <ultron_kernel/RobotInfo.h>
#include <ultron_kernel/RobotOdom.h>
#include <ultron_kernel/RobotSpeed.h>
#include <ultron_kernel/RobotCommand.h>
#include <ultron_kernel/GetDataEncoders.h>

namespace ultron_kernel
{

  /**
  * Class representing Ultron hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class UltronHardware :
    public hardware_interface::RobotHW
  {
  public:
    UltronHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

    void updateDiagnostics();

    void reportLoopDuration(const ros::Duration &duration);

    void onMotorState(const ultron_kernel::RobotOdom::ConstPtr &msg);

    // quando in robot e' pronto
     bool onRobotConnectedSrv( ultron_kernel::RobotCommand::Request & req, ultron_kernel::RobotCommand::Response & res);

  private:

    void initializeDiagnostics();

    void resetTravelOffset();

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    // services

    void GetRobotInfoSrv();
    void ResetPositionSrv();
    void StopSrv();
    void GetDataEncodersSrv();

  private:

    ros::NodeHandle nh_, private_nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // Diagnostics
    ros::Publisher diagnostic_publisher_;
    ultron_kernel::RobotStatus ultron_status_msg_;
    diagnostic_updater::Updater diagnostic_updater_;
  //  UltronHardwareDiagnosticTask<clearpath::DataSystemStatus> system_status_task_;
  //  UltronHardwareDiagnosticTask<clearpath::DataPowerSystem> power_status_task_;
  //  UltronHardwareDiagnosticTask<clearpath::DataSafetySystemStatus> safety_status_task_;
    UltronSoftwareDiagnosticTask software_status_task_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    // ARDUINO

    //! We will be publishing to the "/base_controller/command" topic to issue commands
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber motor_state_sub_;

    ros::ServiceServer connect_srv;

    ultron_kernel::RobotOdom lastMotorStateMsg;

    std::string robotName;
    bool isConnected;

    bool hasMotorState;


    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[4];
  };

}  // namespace ultron_kernel
#endif  // ultron_kernel_ultron_HARDWARE_H
