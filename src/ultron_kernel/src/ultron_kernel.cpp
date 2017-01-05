
#include "ultron_kernel/ultron_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Ultron, not realtime safe
*/
void controlLoop(ultron_kernel::UltronHardware &ultron,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
// ROS_INFO("cc");
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  ultron.reportLoopDuration(elapsed);
  ultron.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  ultron.writeCommandsToHardware();
}

/**
* Diagnostics loop for Ultron, not realtime safe
*/
void diagnosticLoop(ultron_kernel::UltronHardware &ultron)
{
  ultron.updateDiagnostics();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ultron_kernel");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  ultron_kernel::UltronHardware ultron(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&ultron, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Ultron hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue ultron_queue;
  ros::AsyncSpinner ultron_spinner(1, &ultron_queue);

  // ROS_INFO("bb");

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(ultron), boost::ref(cm), boost::ref(last_time)),
    &ultron_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);


  ros::TimerOptions diagnostic_timer(
    ros::Duration(1 / diagnostic_frequency),
    boost::bind(diagnosticLoop, boost::ref(ultron)),
    &ultron_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  //ROS_INFO("11");

  ultron_spinner.start();

   //ROS_INFO("zz");
  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
