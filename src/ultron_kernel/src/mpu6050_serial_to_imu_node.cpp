#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
//#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <ultron_kernel/RobotIMU.h>

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

class IMUManager
{
  	std::string tf_parent_frame_id;
	std::string tf_frame_id;
	std::string frame_id;
	double time_offset_in_seconds;
  	bool broadcast_tf;
  	double linear_acceleration_stddev;
  	double angular_velocity_stddev;
  	double orientation_stddev;
  	uint8_t last_received_message_number;
  	bool received_message ;
  	int data_packet_start;

  	tf::Quaternion orientation;
  	tf::Quaternion zero_orientation;

	sensor_msgs::Imu imu;
	sensor_msgs::Temperature temperature_msg;

	ros::Publisher imu_pub;
	ros::Publisher imu_temperature_pub;
	ros::ServiceServer service;
//	ros::Subscriber hw_subscriver;

	tf::Transform transform;
	tf::TransformBroadcaster tf_br;
public:
	IMUManager(ros::NodeHandle &nh){
		received_message = false;

	  ros::NodeHandle private_node_handle("~");
	  //private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
	  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "base_link");
	  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
	  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
	  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
	  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
	  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
	  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
	  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

//		  ros::NodeHandle nh("imu");
	  imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
	  imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
	  service = nh.advertiseService("set_zero_orientation", set_zero_orientation);
	
  	
	  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
	  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
	  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

	  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
	  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
	  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

	  imu.orientation_covariance[0] = orientation_stddev;
	  imu.orientation_covariance[4] = orientation_stddev;
	  imu.orientation_covariance[8] = orientation_stddev;

	  temperature_msg.variance = 0;
	 
	  transform.setOrigin(tf::Vector3(0,0,0));
	}

	void onIMU(const ultron_kernel::RobotIMU::ConstPtr &_msg){
    
		const ultron_kernel::RobotIMU &msg = *_msg.get();
		const unsigned char *input = &msg.teapotData[0];

		int data_packet_start=0;

  		ROS_INFO("onIMU");
        // get quaternion values
        int16_t w = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff &(char)input[data_packet_start + 3]);
        int16_t x = (((0xff &(char)input[data_packet_start + 4]) << 8) | 0xff &(char)input[data_packet_start + 5]);
       	int16_t y = (((0xff &(char)input[data_packet_start + 6]) << 8) | 0xff &(char)input[data_packet_start + 7]);
    	int16_t z = (((0xff &(char)input[data_packet_start + 8]) << 8) | 0xff &(char)input[data_packet_start + 9]);

		double wf = w/16384.0;
         double xf = x/16384.0;
         double yf = y/16384.0;
         double zf = z/16384.0;

	        tf::Quaternion orientation(xf, yf, zf, wf);

         	if (!zero_orientation_set)
            {
               	zero_orientation = orientation;
            	zero_orientation_set = true;
          	}

          	//http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
          	tf::Quaternion differential_rotation;
          	differential_rotation = zero_orientation.inverse() * orientation;

          	// get gyro values
            int16_t gx = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);
            int16_t gy = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);
             int16_t gz = (((0xff &(char)input[data_packet_start + 14]) << 8) | 0xff &(char)input[data_packet_start + 15]);
                // calculate rotational velocities in rad/s
                // without the last factor the velocities were too small
                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                // FIFO frequency 100 Hz -> factor 10 ?
                // seems 25 is the right factor
                //TODO: check / test if rotational velocities are correct
                double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

                // get acelerometer values
                int16_t ax = (((0xff &(char)input[data_packet_start + 16]) << 8) | 0xff &(char)input[data_packet_start + 17]);
                int16_t ay = (((0xff &(char)input[data_packet_start + 18]) << 8) | 0xff &(char)input[data_packet_start + 19]);
                int16_t az = (((0xff &(char)input[data_packet_start + 20]) << 8) | 0xff &(char)input[data_packet_start + 21]);
                // calculate accelerations in m/s²
                double axf = ax * (8.0 / 65536.0) * 9.81;
                double ayf = ay * (8.0 / 65536.0) * 9.81;
                double azf = az * (8.0 / 65536.0) * 9.81;

                // get temperature
                int16_t temperature = (((0xff &(char)input[data_packet_start + 22]) << 8) | 0xff &(char)input[data_packet_start + 23]);
                double temperature_in_C = (temperature / 340.0 ) + 36.53;
                ROS_DEBUG_STREAM("Temperature [in C] " << temperature_in_C);

                uint8_t received_message_number = input[data_packet_start + 25];
                ROS_DEBUG("received message number: %i", received_message_number);

                if (received_message) // can only check for continuous numbers if already received at least one packet
                {
                  uint8_t message_distance = received_message_number - last_received_message_number;
                  if ( message_distance > 1 )
                  {
                    ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU6050 data packets from arduino.");
                  }
                }
                else
                {
                  received_message = true;
                }
                last_received_message_number = received_message_number;

                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                // publish imu message
                imu.header.stamp = measurement_time;
                imu.header.frame_id = frame_id;

                quaternionTFToMsg(differential_rotation, imu.orientation);

                imu.angular_velocity.x = gxf;
                imu.angular_velocity.y = gyf;
                imu.angular_velocity.z = gzf;

                imu.linear_acceleration.x = axf;
                imu.linear_acceleration.y = ayf;
                imu.linear_acceleration.z = azf;

                imu_pub.publish(imu);

                // publish temperature message
                temperature_msg.header.stamp = measurement_time;
                temperature_msg.header.frame_id = frame_id;
                temperature_msg.temperature = temperature_in_C;

                imu_temperature_pub.publish(temperature_msg);

                // publish tf transform
                if (broadcast_tf)
                {
                  transform.setRotation(differential_rotation);
                  tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                }
               
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mpu6050_imu_node");

    ros::NodeHandle nh("imu");
    
	IMUManager manager(nh);

	ros::NodeHandle nhh;

    ros::Subscriber hw_subscriver = nhh.subscribe<ultron_kernel::RobotIMU>("/petrorov/base/imu",1000,
		&IMUManager::onIMU,&manager );

	
	ros::spin();  

   /* ros::Rate r(200); // 200 hz
  	while(ros::ok())
  	{
    	ros::spinOnce();
    	r.sleep();
  	}
*/
}

