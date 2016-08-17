/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   High level teleoperation interfaces for realtime control
*/

// C++
#include <string>
#include <algorithm>
#include <vector>

// moveit_boilerplate
#include <moveit_boilerplate/trajectory_io.h>

// Boost
#include <boost/lexical_cast.hpp>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Command line arguments
#include <gflags/gflags.h>

// moveit_teleop
#include <moveit_teleop/teleoperation.h>
#include <moveit_teleop/collision_workspace.h>

DEFINE_bool(play_traj, false, "Playback saved trajectory");
DEFINE_string(pose, "", "Requested robot pose");

namespace moveit_teleop
{
Teleoperation::Teleoperation() : moveit_boilerplate::Boilerplate()
{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "visualization_rate", visualization_rate_);
  error += !rosparam_shortcuts::get(name_, rpnh, "waypoints_file", waypoints_file_);
  error += !rosparam_shortcuts::get(name_, rpnh, "waypoints_loop", waypoints_loop_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/output_start_pose", debug_output_start_pose_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/show_joint_limits", debug_show_joint_limits_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Load trajectory IO class
  trajectory_io_.reset(new moveit_boilerplate::TrajectoryIO(planning_scene_monitor_, visual_tools_));

  // Robot Start State Visualization
  visual_start_state_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(), "/moveit_boilerplate/markers2",
                                                       planning_scene_monitor_));
  visual_start_state_->loadRobotStatePub("/moveit_teleop/robot_start_state");
  visual_start_state_->hideRobot();

  // Create collision environment
  moveit_teleop::CollisionWorkspace collision_workspace(nh_, visual_tools_);

  // End effector parent link (arm tip for ik solving)
  ik_tip_link_ = arm_jmg_->getOnlyOneEndEffectorTip();
  if (!ik_tip_link_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Did not find ik_tip_link");
  }

  // Load robot states
  visualize_goal_state_.reset(new moveit::core::RobotState(*getCurrentState()));

  // Create thread for publishing to rviz
  non_realtime_loop_ =
      nh_.createTimer(ros::Duration(1.0 / visualization_rate_), &Teleoperation::visualizationThread, this);

  // Does user want to move to pose?
  std::string pose = FLAGS_pose;
  if (!pose.empty())
  {
    ROS_INFO_STREAM_NAMED(name_, "Moving to SRDF pose '" << FLAGS_pose << "'");
    const double velocity_scaling_factor = 1.0;
    planning_interface_->moveToSRDFPoseNoPlan(arm_jmg_, FLAGS_pose, velocity_scaling_factor);  // this is blocking
  }

  // Create remote control class that includes interactive marker and game controller
  remote_control_.reset(new RemoteControl(debug_interface_, visual_tools_, planning_scene_monitor_, arm_jmg_));
  remote_control_->initializeInteractiveMarkers(getCurrentPose());

  // Set remote_control callback functions
  remote_control_->setIMarkerCallback(
      std::bind(&Teleoperation::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));
  remote_control_->setDashboardCallback(std::bind(&Teleoperation::dashboardCallback, this, std::placeholders::_1));
  remote_control_->setMotionStopCallback(
      std::bind(&moveit_boilerplate::ExecutionInterface::stopExecution, execution_interface_));
  remote_control_->setTeleopCallback(std::bind(&Teleoperation::teleopCallback, this, std::placeholders::_1));

  // Debug: output current EE pose
  if (debug_output_start_pose_)
  {
    ROS_INFO_STREAM_NAMED(name_, "Current EE pose:");
    std::vector<double> xyzrpy;
    visual_tools_->convertToXYZRPY(getCurrentPose(), xyzrpy);
    std::copy(xyzrpy.begin(), xyzrpy.end(), std::ostream_iterator<double>(std::cout, ", "));
  }

  // Does user want to run recorded trajectory?
  if (FLAGS_play_traj)
  {
    playbackTrajectory();
  }
}

Teleoperation::~Teleoperation()
{
}

void Teleoperation::visualizationThread(const ros::TimerEvent &e)
{
  // Check if there is anything to visualize
  // TODO(davetcoleman): this feature is currently not used since restructuring, but may be useful soon so
  // I think we should keep it
  if (has_state_to_visualize_)
  {
    has_state_to_visualize_ = false;

    visual_tools_->publishRobotState(visualize_goal_state_, rvt::BLUE);
  }

  // Test pose functionality
  if (play_test_pose_ && start_next_test_pose_ < ros::Time::now())
  {
    // Check that we have test poses
    if (trajectory_io_->getCartWaypoints().empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Failed to play back test trajectory because no "
                                    "poses available");
      play_test_pose_ = false;
      return;
    }

    // Choose next pose ID
    if (++current_test_pose_ % trajectory_io_->getCartWaypoints().size() == 0)
    {
      if (waypoints_loop_)
      {
        current_test_pose_ = 0;
      }
      else
      {
        play_test_pose_ = false;
        ROS_INFO_STREAM_NAMED(name_, "Finished playing back trajectory");
        return;
      }
    }

    playTrajectoryWaypoint(current_test_pose_);

    // Choose duration for this pose
    const double &waypoint_duration = trajectory_io_->getCartWaypoints()[current_test_pose_].time_;
    start_next_test_pose_ = ros::Time::now() + ros::Duration(waypoint_duration);
  }
}

void Teleoperation::playTrajectoryWaypoint(std::size_t point_id)
{
  if (point_id > trajectory_io_->getCartWaypoints().size() - 1)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Invalid waypoint id " << point_id);
    play_test_pose_ = false;
    return;
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Current waypoint: " << point_id);

  // Move imarker
  remote_control_->updateIMarkerPose(trajectory_io_->getCartWaypoints()[point_id].pose_);

  // Send next pose
  execution_interface_->executePose(trajectory_io_->getCartWaypoints()[point_id].pose_, arm_jmg_);

  // Debug: optionally show joint limits
  if (debug_show_joint_limits_)
    showJointLimits(arm_jmg_);
}

void Teleoperation::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                       const Eigen::Affine3d &feedback_pose)
{
  using visualization_msgs::InteractiveMarkerFeedback;

  double elapsed_time;  // for recording poses

  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      break;
    // --------------------------------------------------------------------------

    case InteractiveMarkerFeedback::MENU_SELECT:
      switch (feedback->menu_entry_id)
      {
        case 1:  // Reset
          ROS_INFO_STREAM_NAMED(name_, "Reset imaker");
          // User has requested to reset location of interactive marker
          remote_control_->updateIMarkerPose(getCurrentPose());
          break;

        case 3:
          ROS_INFO_STREAM_NAMED(name_, "Adding Pose");

          // Add a new point then save the entire trajectory to file
          trajectory_io_->addCartWaypoint(feedback_pose);
          trajectory_io_->saveCartTrajectoryToFile(waypoints_file_);
          break;

        case 4:
          ROS_INFO_STREAM_NAMED(name_, "Recording IMarker Movements");
          visual_tools_->deleteAllMarkers();
          save_imarker_movements_ = true;
          std::cout << "save_imarker_elapsed_time_ " << save_imarker_elapsed_time_ << std::endl;
          break;

        case 5:
          ROS_INFO_STREAM_NAMED(name_, "Stopping Recording IMarker");
          save_imarker_movements_ = false;
          trajectory_io_->saveCartTrajectoryToFile(waypoints_file_);
          break;

        case 7:  // Playing back trajectory

          playbackTrajectory();
          break;

        case 8:
          ROS_INFO_STREAM_NAMED(name_, "Stopping trajectory");

          stopTrajectory();
          break;

        case 9:
          ROS_INFO_STREAM_NAMED(name_, "Clearing trajectory");

          // Remove all points then save empty file
          trajectory_io_->clearCartWaypoints();
          trajectory_io_->saveCartTrajectoryToFile(waypoints_file_);
          break;

        default:
          ROS_WARN_STREAM_NAMED(name_, "Unknown menu id");
      }
      break;
    // --------------------------------------------------------------------------

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      break;

    // --------------------------------------------------------------------------
    case InteractiveMarkerFeedback::POSE_UPDATE:

      execution_interface_->executePose(feedback_pose, arm_jmg_);

      // Optionally save this pose to file
      if (save_imarker_movements_)
      {
        if (save_imarker_elapsed_time_.sec == 0)
          elapsed_time = 0;
        else
          elapsed_time = (ros::Time::now() - save_imarker_elapsed_time_).toSec();
        save_imarker_elapsed_time_ = ros::Time::now();
        trajectory_io_->addCartWaypoint(feedback_pose, elapsed_time);
      }

      break;

    case InteractiveMarkerFeedback::MOUSE_UP:
      break;
  }
}

void Teleoperation::stopTrajectory()
{
  play_test_pose_ = false;
}

void Teleoperation::playbackTrajectory()
{
  ROS_INFO_STREAM_NAMED(name_, "Playing back trajectory from file");

  // Load waypoints from file if none exist
  if (trajectory_io_->getCartWaypoints().empty())
  {
    trajectory_io_->loadCartTrajectoryFromFile(waypoints_file_);

    // Debug show all poses
    visual_tools_->enableBatchPublishing();
    for (std::size_t i = 0; i < trajectory_io_->getCartWaypoints().size(); ++i)
    {
      visual_tools_->publishZArrow(trajectory_io_->getCartWaypoints()[i].pose_, rvt::RED);
    }
    visual_tools_->triggerBatchPublishAndDisable();
  }

  // Check that we have test poses
  if (trajectory_io_->getCartWaypoints().empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "No waypoints loaded, unable to playback trajectory");
    return;
  }

  // Initialize robot arm with first waypoint
  current_test_pose_ = 0;
  ROS_INFO_STREAM_NAMED(name_, "Playing back trajectory");

  // Start randomly playing point
  play_test_pose_ = true;
  start_next_test_pose_ = ros::Time::now();
}

void Teleoperation::dashboardCallback(const dashboard_msgs::DashboardControl::ConstPtr &msg)
{
  if (msg->robot_home)
  {
    ROS_INFO_STREAM_NAMED(name_, "Moving to home pose");
    const double velocity_scaling_factor = 0.75;
    planning_interface_->moveToSRDFPoseNoPlan(arm_jmg_, "home", velocity_scaling_factor);
    remote_control_->updateIMarkerPose(getCurrentPose());
  }
  else if (msg->robot_reset)
  {
    ROS_INFO_STREAM_NAMED(name_, "Moving to zero pose");
    const double velocity_scaling_factor = 0.75;
    planning_interface_->moveToSRDFPoseNoPlan(arm_jmg_, "zero", velocity_scaling_factor);
    remote_control_->updateIMarkerPose(getCurrentPose());
  }
  else if (msg->robot_bringup)  // Actually play
  {
    playbackTrajectory();
  }
  else if (msg->toggle_gripper)  // Actually stop
  {
    std::cout << "stopping trajecotry " << std::endl;
    execution_interface_->stopExecution();
    stopTrajectory();
  }
}

void Teleoperation::teleopCallback(int button_id)
{
  const double velocity_scaling_factor = 0.75;
  const bool wait_for_execution = false;

  // TODO(davetcoleman): remove this callback feature
  switch (button_id)
  {
    case 1:  // Go home
      ROS_INFO_STREAM_NAMED(name_, "Moving to home pose");
      planning_interface_->moveToSRDFPoseNoPlan(arm_jmg_, "home", velocity_scaling_factor, wait_for_execution);
      remote_control_->updateIMarkerPose(getCurrentPose());
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Unknown button controller id: " << button_id);
  }
}

}  // namespace moveit_teleop
