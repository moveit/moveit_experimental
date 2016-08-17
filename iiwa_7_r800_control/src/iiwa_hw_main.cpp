/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   ros_control main() entry point for controlling robots in ROS
*/

#include <iiwa_7_r800_control/iiwa_hw_interface.h>
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_hw_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<iiwa_7_r800_control::IIWAHWInterface> iiwa_hw_interface(
      new iiwa_7_r800_control::IIWAHWInterface(nh));
  iiwa_hw_interface->init();

  // Repeatedly call the step routine to receive and process FRI packets
  // Stop if ROS is ready for shutdown
  bool success = true;
  while (success && ros::ok())
  {
    // Blocking
    success = iiwa_hw_interface->step();
  }

  return 0;
}
