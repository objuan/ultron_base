#include "mpu9250_serial_to_imu_node.h"
#include "fusion/RTFusionRTQF.h"

#define FUSION
#define PACK_SIZE 32

static const double G_TO_MPSS = 9.80665;
#define uT_2_T 1000000
#define DEG_TO_RAD (M_PI / 180.0)

// ==========================================================================================

tf::Quaternion fromEuler(const tf::Vector3& vec)
{
    float cosX2 = cos(vec.x() / 2.0f);
    float sinX2 = sin(vec.x() / 2.0f);
    float cosY2 = cos(vec.y() / 2.0f);
    float sinY2 = sin(vec.y() / 2.0f);
    float cosZ2 = cos(vec.z() / 2.0f);
    float sinZ2 = sin(vec.z() / 2.0f);

    float x = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    float y = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    float z = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    float w = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;

    tf::Quaternion q(x,y,z,w);
    q.normalize();
    return q;
}

void toAngleAxis(const tf::Quaternion &q, float &angle, tf::Vector3& vec)
{
    float sqrLength = q.x() * q.x() + q.y() * q.y() + q.z() * q.z() ;
    if (sqrLength > 0)
    {
        angle = 2.0f * acos(q.w());
        float invLen = 1.0f / sqrt(sqrLength);
        vec= tf::Vector3(q.x() * invLen,q.y() * invLen,q.z() * invLen);
    }
    else{
        angle=0;
        vec= tf::Vector3(1.0f, 0.0, 0.0);
    }
}

tf::Quaternion fromAngleAxis(float &angle, const tf::Vector3& axis)
{
    float halfAnge = angle*0.5f;
    float s = sin(halfAnge);
    return tf::Quaternion(s * axis.x(),s * axis.y(),s * axis.z(),cos(halfAnge));
}

int main(int argc, char** argv)
{
    serial::Serial ser;
    std::string port;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    double time_offset_in_seconds;
    bool broadcast_tf;
    bool broadcast_odom;
    double linear_acceleration_stdev_;
    double angular_velocity_stdev_;
    double mag_stdev_,mag_covariance;
    double orientation_stddev_,pitch_roll_stdev_,yaw_stdev_;
    double linear_acceleration_covariance,angular_velocity_covariance,pitch_roll_covariance,yaw_covariance,orientation_covariance;
    uint8_t last_received_message_number;
    bool received_message = false;
    int data_packet_start;
    double imu_rate;

    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;

    ros::init(argc, argv, "mpu9250_rc_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    // private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "base_link");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
    private_node_handle.param<bool>("broadcast_odom", broadcast_odom, true);
    private_node_handle.param<double>("imu_rate", imu_rate, 200.0);
    //private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
    //private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    //private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 50);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 50);
    ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 50);
    ros::ServiceServer service = nh.advertiseService("imu/set_zero_orientation", set_zero_orientation);

    // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
    private_node_handle.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807 );

    // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    private_node_handle.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));

    private_node_handle.param("mag_stdev", mag_stdev_, 0.05 / (400));

    // 1 degree for pitch and roll
    // private_node_handle.param("orientation_stddev", orientation_stddev_, 1.0 * (M_PI / 180.0));

    // 1 degree for pitch and roll
    private_node_handle.param("pitch_roll_stdev", pitch_roll_stdev_, 1.0 * (M_PI / 180.0));

    // 5 degrees for yaw
    private_node_handle.param("yaw_stdev", yaw_stdev_, 5.0 * (M_PI / 180.0));

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    mag_covariance = mag_stdev_ * mag_stdev_;

    // orientation_covariance = orientation_stddev_ * orientation_stddev_;
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
    yaw_covariance = yaw_stdev_ * yaw_stdev_;


    //ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Rate rate(100); // 100 HZ
    // ros::Rate rate(5); // 200 hz
    //ros::Rate rate(1000.0 / imu_rate); // 200 hz

    ROS_INFO_STREAM("PARAM: rate " << imu_rate);

    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField mag;

    /*
     * orientation_covariance: [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
angular_velocity:
  x: -0.05
  y: -0.05
  z: 0.02
angular_velocity_covariance: [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
linear_acceleration:
  x: 1.92212921875
  y: -2.39251078125
  z: 8.78632921875
linear_acceleration_covariance: [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
*/
   /* <param name="cov_orientation"  type="double" value="0.0005"/>
        <param name="cov_velocity"     type="double" value="0.00025"/>
        <param name="cov_acceleration" type="double" value="0.1"/>
     */

    imu.linear_acceleration_covariance[0] = 0.04;
    imu.linear_acceleration_covariance[4] = 0.04;
    imu.linear_acceleration_covariance[8] = 0.04;

    imu.angular_velocity_covariance[0] = 0.02;
    imu.angular_velocity_covariance[4] = 0.02;
    imu.angular_velocity_covariance[8] = 0.02;

    mag.magnetic_field_covariance[0] = mag_covariance;
    mag.magnetic_field_covariance[4] = mag_covariance;
    mag.magnetic_field_covariance[8] = mag_covariance;

    // imu.orientation_covariance[0] = orientation_covariance;
    // imu.orientation_covariance[4] = orientation_covariance;
    // imu.orientation_covariance[8] = orientation_covariance;
    imu.orientation_covariance[0] = 0.0025;
    imu.orientation_covariance[4] = 0.0025;
    imu.orientation_covariance[8] = 0.0025;

    /*
    imu.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imu.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imu.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imu.angular_velocity_covariance[0] = angular_velocity_covariance;
    imu.angular_velocity_covariance[4] = angular_velocity_covariance;
    imu.angular_velocity_covariance[8] = angular_velocity_covariance;

    mag.magnetic_field_covariance[0] = mag_covariance;
    mag.magnetic_field_covariance[4] = mag_covariance;
    mag.magnetic_field_covariance[8] = mag_covariance;

    // imu.orientation_covariance[0] = orientation_covariance;
    // imu.orientation_covariance[4] = orientation_covariance;
    // imu.orientation_covariance[8] = orientation_covariance;
    imu.orientation_covariance[0] = pitch_roll_covariance;
    imu.orientation_covariance[4] = pitch_roll_covariance;
    imu.orientation_covariance[8] = yaw_covariance;
*/

    sensor_msgs::Temperature temperature_msg;
    temperature_msg.variance = 0;

    static tf::TransformBroadcaster tf_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));

    std::string input;
    std::string read;


    RemoteController rc;

    rc.init(private_node_handle,nh);
    tf::Quaternion last_orientation(0,0,0,1);

    int convergentCount=0;
    int convergentTotal=100; // i seconds
  //  float margin =  0.005;
    float margin =  1;

    //RTFusionRTQF fusion;

    while(ros::ok())
    {
        try
        {
            if (ser.isOpen())
            {
                // read string from serial device
                if(ser.available())
                {               
                    read = ser.read(ser.available());
                    ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
                    input += read;
                    while (input.length() >= 8)//28) // while there might be a complete package in input
                    {
                        //parse for data packets
                        data_packet_start = input.find("$\x09");
                        if (data_packet_start != std::string::npos)
                        {
                            //ROS_INFO("A");
                            rc.onReceive(data_packet_start,input);
                        }
                        //data_packet_start = input.find("$\x04");
                        data_packet_start = input.find("$\x05");
                        if (data_packet_start != std::string::npos)
                        {

                            if (input.length() < PACK_SIZE) continue;

                            ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
                            if ((input.length() >= data_packet_start + PACK_SIZE) && (input.compare(data_packet_start + (PACK_SIZE-2), 2, "\n\r")))  //check if positions 26,27 exist, then test values
                            {
                                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                                // get quaternion values

                                double decimals = 1000;

                                int16_t _ax = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff &(char)input[data_packet_start + 3]);
                                int16_t _ay = (((0xff &(char)input[data_packet_start + 4]) << 8) | 0xff &(char)input[data_packet_start + 5]);
                                int16_t _az = (((0xff &(char)input[data_packet_start + 6]) << 8) | 0xff &(char)input[data_packet_start + 7]);

                                int16_t _gx = (((0xff &(char)input[data_packet_start + 8]) << 8) | 0xff &(char)input[data_packet_start + 9]);
                                int16_t _gy = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);
                                int16_t _gz = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);

                                int16_t _mx = (((0xff &(char)input[data_packet_start + 14]) << 8) | 0xff &(char)input[data_packet_start + 15]);
                                int16_t _my = (((0xff &(char)input[data_packet_start + 16]) << 8) | 0xff &(char)input[data_packet_start + 17]);
                                int16_t _mz = (((0xff &(char)input[data_packet_start + 18]) << 8) | 0xff &(char)input[data_packet_start + 19]);

#ifdef FUSION
                                int16_t _fx = (((0xff &(char)input[data_packet_start + 20]) << 8) | 0xff &(char)input[data_packet_start + 21]);
                                int16_t _fy = (((0xff &(char)input[data_packet_start + 22]) << 8) | 0xff &(char)input[data_packet_start + 23]);
                                int16_t _fz = (((0xff &(char)input[data_packet_start + 24]) << 8) | 0xff &(char)input[data_packet_start + 25]);
                                int16_t _fw = (((0xff &(char)input[data_packet_start + 26]) << 8) | 0xff &(char)input[data_packet_start + 27]);
#endif
                                //float aRes = getAres();

                                // calculate accelerations in m/s²
                                double aRes = 1;//getAres() * 9.81;
                                double ax = (double)_ax * aRes; //  (8.0 / 65536.0) * 9.81;
                                double ay = (double)_ay * aRes;
                                double az = (double)_az * aRes;

                                //float ax = (float)_ax*aRes; // - accelBias[0];
                                //float ay = (float)_ay*aRes; // - accelBias[1];
                                //float az = (float)_az*aRes; // - accelBias[2];

                                const double _d2r = 3.14159265359f/180.0f;
                                double gRes = 1;//getGres() * _d2r;

                                double gx = (double)_gx*gRes;
                                double gy = (double)_gy*gRes;
                                double gz = (double)_gz*gRes ;

                                //TODO: check / test if rotational velocities are correct
                                //double gx = (double)_gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                                //double gy = (double)_gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                                //double gz = (double)_gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;


                                double mRes = 1;//getMres();

                                double mx = (double)_mx*mRes;
                                double my = (double)_my*mRes;
                                double mz = (double)_mz*mRes ;

                                ax/= decimals;
                                ay/= decimals;
                                az/= decimals;
                                gx/= decimals;
                                gy/= decimals;
                                gz/= decimals;
                                mx/= decimals;
                                my/= decimals;
                                mz/= decimals;

#ifdef FUSION

                                double fx = (double)_fx;
                                double fy = (double)_fy;
                                double fz = (double)_fz;
                                double fw = (double)_fw;


                                fx/= decimals;
                                fy/= decimals;
                                fz/= decimals;
                                fw/= decimals;

                               //  ROS_INFO_STREAM("f " << fx <<","<< fy <<","
                              //                  << fz );//<<","<< fw );

                              //  ROS_INFO_STREAM("m " << mx <<","<< my <<","<< mz);
#endif
                                /*
 * Gyro: x:-0.00 y:0.00 z:0.00Accel: x:0.15 y:0.13 z:0.99Mag: x:27.94 y:-6.98 z:26.56Pose: roll:7.21 pitch:-8.45 yaw:22.51
Gyro: x:0.00 y:-0.00 z:-0.00Accel: x:0.15 y:0.13 z:0.99Mag: x:28.52 y:-6.59 z:26.14Pose: roll:7.21 pitch:-8.45 yaw:22.38
Gyro: x:-0.00 y:-0.00 z:-0.00Accel: x:0.15 y:0.13 z:0.99Mag: x:27.96 y:-6.79 z:26.18Pose: roll:7.21 pitch:-8.45 yaw:22.41
Gyro: x:-0.00 y:-0.00 z:-0.00Accel: x:0.15 y:0.13 z:0.99Mag: x:27.63 y:-6.80 z:26.34Pose: roll:7.21 pitch:-8.45 yaw:22.32
*/
                                //mz=0;

                                // time

                                ros::Time time =  ros::Time::now() + ros::Duration(time_offset_in_seconds);
                                ros::Duration diff=time-lastTime;

                                deltat = diff.toSec();
                                lastTime = time;

                                // ----- allineamento iniziale-----------

#ifdef FUSION

                              // tf::Quaternion orientation;
                              //  orientation.setEuler(fx,fy,fz);
                              // orientation = fromEuler(tf::Vector3(fz,fx,fx));

                               tf::Quaternion orientation(fx,fy,fz,fw);
                             
                               // orientation.getAngle()
                                orientation.normalize();

                                // --- CONVERTO SUL FRAME ROS ---------
                                // il FRAME di origine e' uello di RTIMULIB, uguale al compass di 9250
                                // Z giu , Y a destra
                                // per convertire devo ruotare di 180 lungo la X


                                // acc

                                ay = -ay;
                                az=-az;

                                // giro

                                gy=-gy;
                                gz=-gz;

                                // quaternion

                                tf::Matrix3x3 rot;
                                rot.setRotation(orientation);

                                tfScalar roll,pitch,yaw;
                               // roll = -roll;
                                rot.getRPY( roll, pitch, yaw);

                                rot.setIdentity();

                                pitch=-pitch;
                                yaw = -yaw;
                                rot.setRPY(roll, pitch, yaw);

                              //  ROS_INFO_STREAM("m " << roll <<","<< pitch <<","<< yaw );

                                rot.getRotation(orientation);

                                if (!zero_orientation_set)
                                {
                                    zero_orientation = orientation;
                                    zero_orientation_set = true;
                                }
#else
                                MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, my,mx, mz);
                                tf::Quaternion orientation(q[1], q[2], q[3], q[0]);

                                //ROS_INFO_STREAM("Prev  " << last_orientation.x() << " " << last_orientation.y() << " " << last_orientation.z() << " " << last_orientation.w() );
                                //ROS_INFO_STREAM("Read  " << orientation.x() << " " << orientation.y() << " " << orientation.z() << " " << orientation.w() );

                                // sono alineato quando è fermo
                                if (!zero_orientation_set)
                                {
                                    float dist=fabs(orientation.x() - last_orientation.x() )+
                                            fabs(orientation.y() - last_orientation.y() )+
                                            fabs(orientation.z() - last_orientation.z() )+
                                            fabs(orientation.w() - last_orientation.w() );

                                    convergentCount++;

                                    ROS_INFO_STREAM("Distance " << dist << " c:" << convergentCount);


                                    if (dist < margin && convergentCount >= convergentTotal)
                                    {
                                        zero_orientation = orientation;
                                        zero_orientation_set = true;

                                        ROS_INFO_STREAM("Started" << dist);
                                    }
                                    else
                                    {
                                         last_orientation = orientation;
                                         if ( dist >= margin)
                                             convergentCount=0;
                                    }
                                }
#endif
                                if (zero_orientation_set)
                                {

                                    //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                                    tf::Quaternion differential_rotation;
                                    differential_rotation = zero_orientation.inverse() * orientation;

                                    // ABSOLUTE
                                    //differential_rotation = orientation;
                                    // -------------------
/*
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
*/
                                    received_message = true;

                                    // calculate measurement time
                                    ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                                    // publish imu message
                                    imu.header.stamp = measurement_time;
                                    imu.header.frame_id = frame_id;

                                    //differential_rotation = tf::Quaternion(0,0,0,1);
                                    quaternionTFToMsg(differential_rotation, imu.orientation);
                                    // quaternionTFToMsg(orientation, imu.orientation);

                                    // unity
                                    //Accelerations should be in m/s^2 (not in g's),
                                    // and rotational velocity should be in rad/sec
                                    // magnetic field vector in Tesla

                                    imu.linear_acceleration.x = ax;
                                    imu.linear_acceleration.y = ay ;
                                    imu.linear_acceleration.z = az ;

                                    imu.angular_velocity.x = gx * DEG_TO_RAD;
                                    imu.angular_velocity.y = gy* DEG_TO_RAD;
                                    imu.angular_velocity.z = gz* DEG_TO_RAD;


                                    imu_pub.publish(imu);

                                    // publish mag messAGE

                                    mag.header.stamp = measurement_time;
                                    mag.header.frame_id = frame_id;

                                    mag.magnetic_field.x = mx/uT_2_T;
                                    mag.magnetic_field.y = my/uT_2_T;
                                    mag.magnetic_field.z = mz/uT_2_T;

                                    mag_pub.publish(mag);

                                    // publish temperature message
                                    // temperature_msg.header.stamp = measurement_time;
                                    //  temperature_msg.header.frame_id = frame_id;
                                    // temperature_msg.temperature = temperature_in_C;

                                    //  imu_temperature_pub.publish(temperature_msg);

                                    // publish tf transform
                                    if (broadcast_tf)
                                    {
                                        transform.setRotation(differential_rotation);

                                        tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                                    }

                                    // publish ODOM
                                    if (broadcast_odom)
                                    {
                                        nav_msgs::Odometry odom;
                                        odom.header.stamp = measurement_time;
                                        odom.header.frame_id ="odom";
                                        odom.pose.pose.position.x = 0;
                                        odom.pose.pose.position.y = 0;
                                        odom.pose.pose.position.z = 0.0;
                                        odom.pose.pose.orientation = imu.orientation;

                                        //set the velocity
                                        odom.child_frame_id = tf_frame_id;
                                        odom.twist.twist.linear.x = 0;
                                        odom.twist.twist.linear.y = 0;
                                        odom.twist.twist.angular.z = 0;

                                        //odom_pub.publish(odom);
                                    }

                                }

                                input.erase(0, data_packet_start + PACK_SIZE); // delete everything up to and including the processed packet
                            }
                            else
                            {
                                if (input.length() >= data_packet_start + PACK_SIZE)
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
                        else
                        {
                            // no start character found in input, so delete everything
                            input.clear();
                        }
                    }
                }
            }
            else
            {
                // try and open the serial port
                try
                {
                    ROS_INFO_STREAM("Open serial port: " << port);

                    ser.setPort(port);
                    ser.setBaudrate(115200);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }

                if(ser.isOpen())
                {
                    ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
                }
            }
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        /*catch (...)
        {
            ROS_ERROR_STREAM("Error !!");
        }*/
        ros::spinOnce();
        rate.sleep();
    }
}
