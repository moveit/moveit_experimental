/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   Main node for teleoperation of MoveIt! robots
*/

#include <string>
#include <iostream>

// Command line arguments
#include <gflags/gflags.h>

// This class
#include <moveit_teleop/teleoperation.h>

// ROS
#include <ros/ros.h>

int main(int argc, char **argv)
{
  google::SetVersionString("0.1.0");
  google::SetUsageMessage("Teleoperation capabilities for controlling a robot in soft realtime with MoveIt!");
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "moveit_teleop");

  // Message
  ROS_INFO_STREAM_NAMED("main", "Starting MoveIt! Teleoperation");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Main program
  moveit_teleop::Teleoperation manager;

  ros::spin();

  // Shutdown
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");

  ros::shutdown();

  return 0;
}
