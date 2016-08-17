/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   Encapsulate all external control inputs from user
*/

#ifndef MOVEIT_TELEOP_REMOTE_CONTROL_H
#define MOVEIT_TELEOP_REMOTE_CONTROL_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <dashboard_msgs/DashboardControl.h>
#include <sensor_msgs/Joy.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>

// MoveIt
#include <moveit_boilerplate/debug_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_boilerplate/namespaces.h>

// C++
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>

namespace moveit_teleop
{
MOVEIT_CLASS_FORWARD(RemoteControl);

typedef std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &, const Eigen::Affine3d &)>
    IMarkerCallback;
typedef std::function<void(const dashboard_msgs::DashboardControl::ConstPtr &)> DashboardCallback;
typedef std::function<void()> MotionStopCallback;
typedef std::function<void(const std::size_t &)> TeleopCallback;

class RemoteControl
{
public:
  /**
   * \brief Constructor
   * \param debug_interface - Tool for creating break points and user verification points
   */
  RemoteControl(moveit_boilerplate::DebugInterfacePtr debug_interface, mvt::MoveItVisualToolsPtr visual_tools,
                psm::PlanningSceneMonitorPtr planning_scene_monitor, JointModelGroup *arm_jmg);

  /** \brief Destructor */
  virtual ~RemoteControl()
  {
  }

  /** \brief Load the Rviz interaction tool */
  void initializeInteractiveMarkers(const Eigen::Affine3d &pose);

  /** \brief Set where in the parent class the feedback should be sent */
  void setIMarkerCallback(IMarkerCallback callback)
  {
    imarker_callback_ = callback;
  }

  /** \brief Set where in the parent class the rviz dashboard callback calls */
  void setDashboardCallback(DashboardCallback callback)
  {
    dashboard_callback_ = callback;
  }

  /** \brief Set where in the parent class the game controller callback calls */
  void setTeleopCallback(TeleopCallback callback)
  {
    teleop_callback_ = callback;
  }

  /** \brief Tell the imarker server to move the marker in Rviz */
  void sendUpdatedIMarkerPose();

  /** \brief Programmatically move the interactive marker */
  void updateIMarkerPose(const Eigen::Affine3d &pose);

  /** \brief Programmatically control imarker location and execute robot */
  void moveIMarkerAndExecute(const Eigen::Affine3d &pose);

  /** \brief Stop motion of robot (soft stop) */
  void softMotionStop();

  /** \brief Set a callback event to fire when stop is called */
  void setMotionStopCallback(MotionStopCallback motion_stop_callback)
  {
    motion_stop_callback_ = motion_stop_callback;
  }

  /** \brief Get the pose relative to the interactive marker */
  Eigen::Affine3d getIMPose();

private:
  /** \brief Receive inputs from a GUI panel on Rviz */
  void rvizDashboardCallback(const dashboard_msgs::DashboardControl::ConstPtr &msg);

  /** \brief Recieves inputs from joystick */
  void gameControllerCallback(const sensor_msgs::Joy::ConstPtr &msg);

  /** \brief Recieves inputs from rosmsg */
  void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /** \brief Process motion change */
  void poseMsgControl(const Eigen::Affine3d &input_pose);

  /** \brief Helper for creating a imarker */
  void make6DofMarker(const geometry_msgs::Pose &pose);

  /** \brief Helper for geometric shape */
  visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg);

  /** \brief Use the mesh of the robot's end effector as the IMarker control */
  visualization_msgs::InteractiveMarkerControl &makeEEControl(visualization_msgs::InteractiveMarker &msg);

  /** \brief Callback from interactive marker server */
  void iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /** \brief Get the pose relative to the ROBOT, not the interactive marker */
  Eigen::Affine3d getEEPose();

  /** \brief Continously update the imarker location using a game controller */
  void gameControllerThread(const ros::TimerEvent &e);

  /** \brief Use the planning scene to get the robot's current state */
  moveit::core::RobotStatePtr getCurrentState();

  /** \brief Get pose of the end effector */
  const Eigen::Affine3d &getCurrentPose();

  // --------------------------------------------------------

  // Short name of this class
  std::string name_ = "remote_control";

  // A shared node handle
  ros::NodeHandle nh_;

  // State machine for debugging other aspects of system
  moveit_boilerplate::DebugInterfacePtr debug_interface_;

  // Visualization
  mvt::MoveItVisualToolsPtr visual_tools_;

  // Access to planning scene
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Input sources
  ros::Subscriber rviz_dashboard_sub_;
  ros::Subscriber game_controller_sub_;
  ros::Subscriber pose_msg_sub_;

  // Game controller control
  double game_controller_speed_;
  double game_controller_x_ = 0.0;
  double game_controller_y_ = 0.0;
  double game_controller_z_ = 0.0;
  double game_controller_rol_ = 0.0;
  double game_controller_pit_ = 0.0;
  double game_controller_yaw_ = 0.0;
  double game_controller_rotation_speed_;
  ros::Timer non_realtime_loop_;

  // Interactive markers
  std::string imarker_topic_;
  std::string imarker_robot_link_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::InteractiveMarker int_marker_;
  bool teleoperation_ready_ = true;
  boost::mutex imarker_mutex_;
  Eigen::Affine3d imarker_pose_;

  // Head pose topic
  bool deadman_prev_enabled_ = false;  // when to set the starting state of the incoming poses
  Eigen::Affine3d pose_msg_previous_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Amount to move interactive marker from tip link of kinematic chain
  Eigen::Affine3d imarker_offset_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Desired planning group to work with
  JointModelGroup *arm_jmg_;

  // Callbacks
  IMarkerCallback imarker_callback_;         // hook to parent class
  DashboardCallback dashboard_callback_;     // hook to parent class
  MotionStopCallback motion_stop_callback_;  // hook to parent class
  TeleopCallback teleop_callback_;           // hook to parent class
};                                           // end class

}  // namespace moveit_teleop

#endif  // MOVEIT_TELEOP_REMOTE_CONTROL_H
