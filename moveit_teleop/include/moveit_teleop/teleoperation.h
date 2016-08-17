/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   High level teleoperation interfaces for realtime control
*/

#ifndef MOVEIT_TELEOP_TELEOPERATION_H
#define MOVEIT_TELEOP_TELEOPERATION_H

// C++
#include <string>
#include <vector>

// boost
#include <boost/thread/locks.hpp>

// moveit_boilerplate
#include <moveit_boilerplate/boilerplate.h>
#include <moveit_boilerplate/namespaces.h>

// moveit_teleop
#include <moveit_teleop/remote_control.h>

namespace moveit_boilerplate
{
MOVEIT_CLASS_FORWARD(TrajectoryIO);
}

namespace moveit_teleop
{
MOVEIT_CLASS_FORWARD(Teleoperation);

static const std::string PACKAGE_PATH = "moveit_boilerplate";

class Teleoperation : public moveit_boilerplate::Boilerplate
{
public:
  /** \brief Constructor */
  Teleoperation();

  /** \brief Destructor */
  virtual ~Teleoperation();

private:
  /** \brief Publish the robot state to Rviz in a separate thread */
  void visualizationThread(const ros::TimerEvent &e);

  /** \brief Move to the specified waypoint in the trajectory */
  void playTrajectoryWaypoint(std::size_t point_id);

  /**
   * \brief Callback from interactive markers
   * \param pose - pose recieved from marker
   * \param mode - 1 is regular - recieve new pose,
   *               2 is reset imarker
   */
  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                          const Eigen::Affine3d &feedback_pose);

  /** \brief Stop playback of trajectory */
  void stopTrajectory();

  /** \brief Start the playback of saved from file waypoint trajectories */
  void playbackTrajectory();

  /** \brief Recieve dashboard callback */
  void dashboardCallback(const dashboard_msgs::DashboardControl::ConstPtr &msg);

  /** \brief Recieve input from a game controller */
  void teleopCallback(int button_id);

  /** \brief Add objects to planning scene for robot to avoid */
  void addCollisionObjects(const std::vector<double> &object_dimensions_6, const rvt::colors &color,
                           const std::string &name);

  /** \brief Callback to allow visualization thread to see IK solution from planner */
  void visualizeGoalStateCallback(const moveit::core::RobotState &state);

  /** \brief Callback to allow visualization thread to see IK start state */
  void visualizeStartStateCallback(const moveit::core::RobotState &state);

  // --------------------------------------------------------
  // Name of this class
  std::string name_ = "teleoperation";

  // Child classes
  RemoteControlPtr remote_control_;

  // Visualization -------------------------------------
  bool has_state_to_visualize_ = false;
  double visualization_rate_;  // hz
  ros::Timer non_realtime_loop_;

  // Debug
  bool debug_output_start_pose_;
  bool debug_show_joint_limits_;

  // State to visualize in Rviz, also the current goal state
  moveit::core::RobotStatePtr visualize_goal_state_;

  // Trajectory Playback -------------------------------------------
  bool save_imarker_movements_ = false;
  ros::Time save_imarker_elapsed_time_;
  std::string waypoints_file_;
  std::size_t current_test_pose_;
  ros::Time start_next_test_pose_;
  bool play_test_pose_ = false;
  bool waypoints_loop_;

  // Allow loading and saving trajectories to file
  moveit_boilerplate::TrajectoryIOPtr trajectory_io_;

  // Secondary visual tools
  mvt::MoveItVisualToolsPtr visual_start_state_;

  // Last link of robot arm at the end effector
  const moveit::core::LinkModel *ik_tip_link_;
};  // end class

}  // namespace moveit_teleop

#endif  // MOVEIT_TELEOP_TELEOPERATION_H
