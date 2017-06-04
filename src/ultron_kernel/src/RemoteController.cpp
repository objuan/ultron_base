#include "RemoteController.h"


void RemoteController::init(ros::NodeHandle private_nh,ros::NodeHandle nh)
{
	this->nh_=nh;

   	diagnostic_.add("R/C Driver Status", this, &RemoteController::diagnostics);
    	diagnostic_.setHardwareID("none");

	event_count_ = 0;
    	pub_count_ = 0;
	lastDiagTime_ = ros::Time::now().toSec();

  	double remote_control_frequency;
  	//private_nh.param<double>("remote_control_frequency", remote_control_frequency, 50.0);
 	//private_nh.param<double>("coalesce_interval", coalesce_interval_, 0.001);
	private_nh.getParam("/petrorov/ultron/remote_control/speed", speed);
	private_nh.getParam("/petrorov/ultron/remote_control/turn", turn);
	private_nh.param<bool>("/petrorov/ultron/remote_control/normalize", normalize, false);
	private_nh.param<double>("/petrorov/ultron/remote_control/dead_zone", dead_zone, 2048);
	private_nh.getParam("/petrorov/ultron/remote_control/speed_trx", speed_trx);
	private_nh.getParam("/petrorov/ultron/remote_control/turn_trx", turn_trx);


	//ROS_INFO("Calibrate INFOS speed: %f-%f turn:%f-%f"  ,speed[0],speed[1],turn[0],turn[1]);

  	pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);

	diagnostic_.force_update();

    	tv_set = false;
    	publication_pending = false;
	publish_soon=false;
      	tv.tv_sec = 1;
      	tv.tv_usec = 0;

	joy_msg.buttons.resize(2);
	joy_msg.axes.resize(4);
	joy_msg.buttons[0]= 0;
	joy_msg.buttons[1]= 0;
}

// =====================================================
//
// type , 0 = mezzo, 1 = inizio
double Normalize(double in_val,std::vector<double> &range,double dead_zone,int type)
{
	double val = in_val;	
	//if (std::abs(val - range[2]) < dead_zone) val= range[2];

	if (val < range[2] && type == 0)
	{
		if (val > range[2] - dead_zone) val = range[2]- dead_zone;
		val = std::max(val,	range[0]);	
		val =  0.5 *((val - range[0] ) / (range[2]-range[0] - dead_zone));
	}
	else //>=
	{
		if (val < range[2]+dead_zone) val = range[2]+dead_zone;
		val = std::max(val,	range[0]);	
		val = std::min(val,	range[1]);	
		if (type == 0)
			val = 0.5 + 0.5 * ((val - (range[2] + dead_zone) ) / (range[1]-range[2]-dead_zone));
		else
			val =  ((val - (range[2] + dead_zone) ) / (range[1]-range[2]-dead_zone));
	}
	//else
	//	val=0.5;

	return val;
}

void RemoteController::calibrate(double &ch1,double &ch2){
	if (normalize)
	{
		/*if (std::abs(ch1 - speed[2]) < dead_zone)
			ch1= speed[2];
	
		if (std::abs(ch2 - turn[2]) < dead_zone)
			ch2= turn[2];

		// assign ch1 -> speed
		// ch2 -> turn
		// normalize 0 ,1
		ch1 = std::max(ch1,	speed[0]);	
		ch1 = std::min(ch1,	speed[1]);	
		ch2 = std::max(ch2,	turn[0]);	
		ch2 = std::min(ch2,	turn[1]);	

		ch1 = (ch1 - speed[0]) / (speed[1]-speed[0]);
		ch2 = (ch2 - turn[0]) / (turn[1]-turn[0]);
*/
		ch1 = Normalize(ch1,speed,dead_zone,1);
		ch2 = Normalize(ch2,turn,dead_zone,0);

		// final TRX
		ch1 = (ch1 + speed_trx[0]) * speed_trx[1];
		ch2 = (ch2 + turn_trx[0]) * turn_trx[1];

	}
}


// =====================================================
//
void RemoteController::onReceive(int &data_packet_start, std::string &input)
{
	//ROS_INFO("Calibrate INFOS speed: %f-%f turn:%f-%f"  ,speed[0],speed[1],turn[0],turn[1]);
  	ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
        if ((input.length() >= data_packet_start + 8) && (input.compare(data_packet_start + 6, 2, "\n\r")))  //check if positions 6,7 exist, then test values
        {
		bool publish_now=true;

		joy_msg.header.stamp = ros::Time().now();
		
		// raw data , from 0 to 1024
		int16_t _ch1 = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff &(char)input[data_packet_start + 3]);
		int16_t _ch2 = (((0xff &(char)input[data_packet_start + 4]) << 8) | 0xff &(char)input[data_packet_start + 5]);

		double ch1,ch2;
		ch1=_ch1;
		ch2=_ch2;

		joy_msg.axes[2] = ch1;
		joy_msg.axes[3] = ch2;

		calibrate(ch1,ch2);

		joy_msg.axes[0] = ch1;
		joy_msg.axes[1] = ch2;

	 	if (publish_now)
		{
		  // Assume that all the JS_EVENT_INIT messages have arrived already.
		  // This should be the case as the kernel sends them along as soon as
		  // the device opens.
		  //ROS_INFO("Publish...");
		  pub_.publish(joy_msg);
		  publish_now = false;
		  tv_set = false;
		  publication_pending = false;
		  publish_soon = false;
		  pub_count_++;
		}
		
		// If an axis event occurred, start a timer to combine with other
		// events.
		if (!publication_pending && publish_soon)
		{
		  tv.tv_sec = trunc(coalesce_interval_);
		  tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
		  publication_pending = true;
		  tv_set = true;
		  //ROS_INFO("Pub pending...");
		}
		
		// If nothing is going on, start a timer to do autorepeat.
		/*if (!tv_set && autorepeat_rate_ > 0)
		{
		  tv.tv_sec = trunc(autorepeat_interval);
		  tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6; 
		  tv_set = true;
		  //ROS_INFO("Autorepeat pending... %i %i", tv.tv_sec, tv.tv_usec);
		}
*/
		
		if (!tv_set)
		{
		  tv.tv_sec = 1;
		  tv.tv_usec = 0;
		}
	
		diagnostic_.update();

		//
		input.erase(0, data_packet_start + 8);
	}
	else
	{
		if (input.length() >= data_packet_start + 8)
                {
                  	input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                }
                else
                {
                  	// do not delete start character, maybe complete package has not arrived yet
                  	input.erase(0, data_packet_start);
                }

	}

}
