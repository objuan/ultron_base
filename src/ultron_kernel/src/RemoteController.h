#pragma once

//#include "ultron_kernel/ultron_hardware.h"
//#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <fcntl.h>
#include <diagnostic_updater/diagnostic_updater.h>

typedef boost::chrono::steady_clock time_source;

class RemoteController
{
 	ros::NodeHandle nh_;
  	int event_count_;
  	int pub_count_;
  	ros::Publisher pub_;
	double lastDiagTime_;

	std::vector<double> speed;
	std::vector<double> turn;
	bool normalize;
	double dead_zone;
	std::vector<double> speed_trx;
	std::vector<double> turn_trx;


// 	js_event event;
    	struct timeval tv;
    	fd_set set;
    	int joy_fd;
    	sensor_msgs::Joy joy_msg; // Here because we want to reset it on device close.
	bool tv_set ;
       	bool publication_pending ;
	bool publish_soon;
	double coalesce_interval_; // Defaults to 100 Hz rate limit.

  diagnostic_updater::Updater diagnostic_;
  
public:

  ///\brief Publishes diagnostics and status
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    double now = ros::Time::now().toSec();
    double interval = now - lastDiagTime_;
   // if (open_)
      stat.summary(0, "OK");
   // else
   //   stat.summary(2, "Joystick not open.");
    
    stat.add("topic", pub_.getTopic());
  //  stat.add("device", joy_dev_);
  //  stat.add("dead zone", deadzone_);
  //  stat.add("autorepeat rate (Hz)", autorepeat_rate_);
   // stat.add("coalesce interval (s)", coalesce_interval_);
    stat.add("recent joystick event rate (Hz)", event_count_ / interval);
    stat.add("recent publication rate (Hz)", pub_count_ / interval);
    stat.add("subscribers", pub_.getNumSubscribers());
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = now;
}

public:
	void calibrate(double &ch1,double &ch2);

	void init(ros::NodeHandle private_nh,ros::NodeHandle nh);

	// ritorna il nuovo bufffer
	void onReceive(int &data_packet_start, std::string &input);
	
};

