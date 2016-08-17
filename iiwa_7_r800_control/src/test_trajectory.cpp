/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   Send simple trajectory to IIWA
*/

#ifndef IIWA_7_R800_CONTROL_TEST_TRAJECTORY_H
#define IIWA_7_R800_CONTROL_TEST_TRAJECTORY_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace iiwa_7_r800_control
{
class TestTrajectory
{
public:
  /**
   * \brief Constructor
   */
  TestTrajectory() : name_("test_trajectory")
  {
    const std::string joint_trajectory_topic = "/iiwa_7_r800/reflexxes_trajectory_controller/command";
    ROS_DEBUG_STREAM_NAMED(name_, "Publishing to " << joint_trajectory_topic);
    const std::size_t queue_size = 1;
    joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(joint_trajectory_topic, queue_size);
    ros::spinOnce();
    ros::Duration(1).sleep();

    ROS_INFO_STREAM_NAMED(name_, "TestTrajectory Ready.");
  }

  void test()
  {
    trajectory_msgs::JointTrajectory trajectory;
    std::size_t num_joints = 7;

    // Add joint names
    for (std::size_t i = 0; i < num_joints; ++i)
    {
      trajectory.joint_names.push_back("joint_a" + std::to_string(i));
    }
    // Add points
    trajectory_msgs::JointTrajectoryPoint point1;
    point1.positions.resize(num_joints);
    point1.velocities.resize(num_joints);

    // Point 1
    trajectory.points.push_back(point1);

    // Point 2
    trajectory_msgs::JointTrajectoryPoint point2 = point1;
    point2.positions[5] = 1.57;
    trajectory.points.push_back(point2);

    // Point 3
    trajectory_msgs::JointTrajectoryPoint point3 = point2;
    point3.positions[4] = 1.57;
    trajectory.points.push_back(point3);

    // Point 4
    trajectory.points.push_back(point1);

    // Publish trajectory
    // std::cout << "Trajectory: \n" << trajectory << std::endl;
    joint_trajectory_pub_.publish(trajectory);
    ros::spinOnce();
  }

private:
  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Send a trajectory
  ros::Publisher joint_trajectory_pub_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestTrajectory> TestTrajectoryPtr;
typedef boost::shared_ptr<const TestTrajectory> TestTrajectoryConstPtr;

}  // namespace iiwa_7_r800_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ROS_INFO_STREAM_NAMED("main", "Starting TestTrajectory...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  iiwa_7_r800_control::TestTrajectory server;
  server.test();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif
