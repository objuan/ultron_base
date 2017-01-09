

#include "ultron_kernel/ultron_hardware.h"
#include <boost/assign/list_of.hpp>



namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace ultron_kernel
{

  /**
  * Initialize Ultron hardware
  */
  UltronHardware::UltronHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh),isConnected(false),
   // system_status_task_(ultron_status_msg_),
   // power_status_task_(ultron_status_msg_),
  //  safety_status_task_(ultron_status_msg_),
    software_status_task_(ultron_status_msg_, target_control_freq)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.15);
   // private_nh_.param<double>("max_accel", max_accel_, 5.0);
  //  private_nh_.param<double>("max_speed", max_speed_, 1.0);
  //  private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    //std::string port;
    //private_nh_.param<std::string>("port", port, "/dev/prolific");

   // horizon_legacy::connect(port);
   // horizon_legacy::configureLimits(max_speed_, max_accel_);

    // ARDUINO INTERFACE


    cmd_vel_pub_ = nh_.advertise<ultron_kernel::RobotOdom>("/petrorov/cmd_vel", 1);

    motor_state_sub_ = nh_.subscribe<ultron_kernel::RobotOdom>("/petrorov/base/motor_state", 1000,&UltronHardware::onMotorState,this );

    connect_srv= nh_.advertiseService("/petrorov/base/connect",&UltronHardware::onRobotConnectedSrv,this);

    hasMotorState=false;

     ROS_INFO("CONTROL NODE INITIALIZED");

    resetTravelOffset();
    initializeDiagnostics();
    registerControlInterfaces();
  }


  bool UltronHardware::onRobotConnectedSrv( ultron_kernel::RobotCommand::Request & req, ultron_kernel::RobotCommand::Response & res){

      ROS_INFO("onRobotConnectedSrv");

  /*
      isConnected=true;
     // onConnected();

    GetRobotInfoSrv();

    ResetPositionSrv();

    StopSrv();
    */

    return true;
  }


  void UltronHardware::onMotorState(const ultron_kernel::RobotOdom::ConstPtr &msg){
      lastMotorStateMsg = *msg.get();
      hasMotorState=true;
  }



  void UltronHardware::GetRobotInfoSrv()
  {
        ros::ServiceClient client = private_nh_.serviceClient<ultron_kernel::GetRobotInfo>("/petrorov/base/srv/getInfo");
        ultron_kernel::GetRobotInfo srv;

        if (client.call(srv))
        {
            ROS_INFO("ROBOT NAME: %s",srv.response.info.name.c_str());
            robotName = srv.response.info.name.c_str();

            // messo qui , il service onRobotConnectedSrv non va
            isConnected=true;

            // aspetto un attimo

            ros::Duration(0.5).sleep(); // sleep for half a second

            ResetPositionSrv();

            GetDataEncodersSrv();

            StopSrv();
        }
        else
        {
            ROS_DEBUG("Failed to connect GetRobotInfoSrv .. ");
        }
  }


  void UltronHardware::ResetPositionSrv()
  {
	if (!isConnected) return;
        ros::ServiceClient client = private_nh_.serviceClient<ultron_kernel::RobotCommand>("/petrorov/base/srv/resetPos");
        ultron_kernel::RobotCommand srv;

        if (client.call(srv))
        {
            ROS_INFO("Reset Position DONE");

        }
        else
        {
            ROS_ERROR("Failed to connect ResetPositionSrv .. ");
        }
  }

  void UltronHardware::GetDataEncodersSrv()
  {
	if (!isConnected) return;
	ROS_INFO("GetDataEncodersSrv CALL");

        ros::ServiceClient client = private_nh_.serviceClient<ultron_kernel::GetDataEncoders>("/petrorov/base/srv/getDataEncoders");
        ultron_kernel::GetDataEncoders srv;

        if (client.call(srv))
        {
            ROS_INFO("GetDataEncodersSrv DONE");

 	    for (int i = 0; i < 4; i++)
	    {
	        joints_[i%2].position_offset = linearToAngular((i % 2 == 0) ? srv.response.left_encoder_pos : srv.response.right_encoder_pos);
	    }
        }
        else
        {
            ROS_ERROR("Failed to connect GetDataEncodersSrv .. ");
        }
  }

  void UltronHardware::StopSrv()
  {
	if (!isConnected) return;
        ros::ServiceClient client = private_nh_.serviceClient<ultron_kernel::RobotCommand>("/petrorov/base/srv/stop");
        ultron_kernel::RobotCommand srv;

        if (client.call(srv))
        {
          ROS_INFO("Stop DONE");
        }
        else
        {
            ROS_ERROR("Failed to connect StopSrv .. ");
        }
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void UltronHardware::resetTravelOffset()
  {
	GetDataEncodersSrv();
	//TODO
   /* horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
      polling_timeout_);
    if (enc)
    {
      for (int i = 0; i < 4; i++)
      {
        joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
      }
    }
    else
    {
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
    */
  }

  /**
  * Register diagnostic tasks with updater class
  */
  void UltronHardware::initializeDiagnostics()
  {
 /*   horizon_legacy::Channel<clearpath::DataPlatformInfo>::Ptr info =
      horizon_legacy::Channel<clearpath::DataPlatformInfo>::requestData(polling_timeout_);
    std::ostringstream hardware_id_stream;
    hardware_id_stream << "Ultron " << info->getModel() << "-" << info->getSerial();

    diagnostic_updater_.setHardwareID(hardware_id_stream.str());
    diagnostic_updater_.add(system_status_task_);
    diagnostic_updater_.add(power_status_task_);
    diagnostic_updater_.add(safety_status_task_);
    diagnostic_updater_.add(software_status_task_);
    diagnostic_publisher_ = nh_.advertise<ultron_msgs::UltronStatus>("status", 10);
    */
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void UltronHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * External hook to trigger diagnostic update
  */
  void UltronHardware::updateDiagnostics()
  {
      //TODO
    //diagnostic_updater_.force_update();
    //ultron_status_msg_.header.stamp = ros::Time::now();
    //diagnostic_publisher_.publish(ultron_status_msg_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void UltronHardware::updateJointsFromHardware()
  {
      if (!isConnected)
        GetRobotInfoSrv();

      if (!hasMotorState) return;
      hasMotorState=false; // consumo



      ROS_DEBUG_STREAM("Received travel information (L:" << lastMotorStateMsg.left_position  << " R:" << lastMotorStateMsg.right_position << ")");

      for (int i = 0; i < 4; i++)
      {
        //  ROS_INFO("aa");

        double delta = linearToAngular(( i %2 == 0) ? lastMotorStateMsg.left_position : lastMotorStateMsg.right_position) - joints_[i].position - joints_[i].position_offset;

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0)
        {
          joints_[i].position += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
          ROS_DEBUG("Dropping overflow measurement from encoder");
        }
      }

      ROS_DEBUG_STREAM("Received linear speed information (L:" << lastMotorStateMsg.left_speed << " R:" << lastMotorStateMsg.right_speed << ")");

      for (int i = 0; i < 4; i++)
      {
           double speed = linearToAngular(( i %2 == 0) ? lastMotorStateMsg.left_speed : lastMotorStateMsg.right_speed);
           joints_[i].velocity = linearToAngular(speed);

       /* if (i % 2 == LEFT)
        {
          joints_[i].velocity = linearToAngular(20);
        }
        else
        { // assume RIGHT
          joints_[i].velocity = linearToAngular(30);
        }
        */
      }


      /*

    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
      polling_timeout_);
    if (enc)
    {
      ROS_DEBUG_STREAM("Received travel information (L:" << enc->getTravel(LEFT) << " R:" << enc->getTravel(RIGHT) << ")");
      for (int i = 0; i < 4; i++)
      {
        double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0)
        {
          joints_[i].position += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
          ROS_DEBUG("Dropping overflow measurement from encoder");
        }
      }
    }

    horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed = horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(
      polling_timeout_);
    if (speed)
    {
      ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed() << " R:" << speed->getRightSpeed() << ")");
      for (int i = 0; i < 4; i++)
      {
        if (i % 2 == LEFT)
        {
          joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
        }
        else
        { // assume RIGHT
          joints_[i].velocity = linearToAngular(speed->getRightSpeed());
        }
      }
    }
    */
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  *  NOTE: SPEED ARE IN RAD/SEC
  */
  void UltronHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);


    //ROS_INFO_STREAM("Received speed (L:" << diff_speed_left << " R:" << diff_speed_right << ")");

    ultron_kernel::RobotOdom base_cmd;

    // metri al secondo
    base_cmd.left_speed = diff_speed_left;
    base_cmd.right_speed = diff_speed_right;

    cmd_vel_pub_.publish(base_cmd);

  //  limitDifferentialSpeed(diff_speed_left, diff_speed_right);

  //  horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
  }

  /**
  * Update diagnostics with control loop timing information
  */
  void UltronHardware::reportLoopDuration(const ros::Duration &duration)
  {
    software_status_task_.updateControlFrequency(1 / duration.toSec());
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void UltronHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Ultron reports travel in metres, need radians for ros_control RobotHW
  */
  double UltronHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Ultron needs m/s,
  */
  double UltronHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace ultron_kernel
