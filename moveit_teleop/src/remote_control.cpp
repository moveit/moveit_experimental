/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   Encapsulate all external control inputs from user
*/

// C++
#include <string>

// moveit_teleop
#include <moveit_teleop/remote_control.h>
#include <moveit_teleop/teleoperation.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace moveit_teleop
{
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerControl;

RemoteControl::RemoteControl(moveit_boilerplate::DebugInterfacePtr debug_interface,
                             mvt::MoveItVisualToolsPtr visual_tools,
                             psm::PlanningSceneMonitorPtr planning_scene_monitor, JointModelGroup *arm_jmg)
  : nh_("~")
  , debug_interface_(debug_interface)
  , visual_tools_(visual_tools)
  , planning_scene_monitor_(planning_scene_monitor)
  , arm_jmg_(arm_jmg)
{
  // Settings
  std::string rviz_dashboard_topic;
  std::string game_controller_topic;
  std::string pose_msg_topic;

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  int error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "rviz_dashboard_topic", rviz_dashboard_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "game_controller_topic", game_controller_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "pose_msg_topic", pose_msg_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_topic", imarker_topic_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_robot_link", imarker_robot_link_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_offset", imarker_offset_);
  error += !rosparam_shortcuts::get(name_, rpnh, "game_controller_speed", game_controller_speed_);
  error += !rosparam_shortcuts::get(name_, rpnh, "game_controller_rotation_speed", game_controller_rotation_speed_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Subscribe to Rviz Dashboard
  const std::size_t button_queue_size = 10;
  rviz_dashboard_sub_ = nh_.subscribe<dashboard_msgs::DashboardControl>(rviz_dashboard_topic, button_queue_size,
                                                                        &RemoteControl::rvizDashboardCallback, this);
  // Subscribe to Game Controller
  game_controller_sub_ = nh_.subscribe<sensor_msgs::Joy>(game_controller_topic, button_queue_size,
                                                         &RemoteControl::gameControllerCallback, this);
  // Subscribe to Pose Msg
  const std::size_t queue_size = 1;  // do not allow buffering
  pose_msg_sub_ =
      nh_.subscribe<geometry_msgs::PoseStamped>(pose_msg_topic, queue_size, &RemoteControl::poseMsgCallback, this);

  // Create thread for game controller
  const double game_controller_rate = 10;
  non_realtime_loop_ =
      nh_.createTimer(ros::Duration(1.0 / game_controller_rate), &RemoteControl::gameControllerThread, this);

  ROS_INFO_STREAM_NAMED("remote_control", "RemoteControl Ready.");
}

void RemoteControl::initializeInteractiveMarkers(const Eigen::Affine3d &pose)
{
  // Move marker to tip of fingers
  imarker_pose_ = pose * imarker_offset_.inverse();

  // Convert
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(imarker_pose_, pose_msg);

  // Server
  imarker_server_.reset(new interactive_markers::InteractiveMarkerServer(imarker_topic_, "", false));

  // Menu
  menu_handler_.insert("Reset", boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle1 = menu_handler_.insert("Recording");
  menu_handler_.insert(sub_menu_handle1, "Add Current Pose", boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  menu_handler_.insert(sub_menu_handle1, "Record IMarker Movements",
                       boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  menu_handler_.insert(sub_menu_handle1, "Stop Recording IMarker",
                       boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle2 = menu_handler_.insert("Playback");
  menu_handler_.insert(sub_menu_handle2, "Play Trajectory", boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  menu_handler_.insert(sub_menu_handle2, "Stop Trajectory", boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  menu_handler_.insert(sub_menu_handle2, "Clear Trajectory", boost::bind(&RemoteControl::iMarkerCallback, this, _1));

  // marker
  make6DofMarker(pose_msg);
  imarker_server_->applyChanges();
}

void RemoteControl::updateIMarkerPose(const Eigen::Affine3d &pose)
{
  // Move marker to tip of fingers
  imarker_pose_ = pose * imarker_offset_.inverse();
  sendUpdatedIMarkerPose();
}

void RemoteControl::sendUpdatedIMarkerPose()
{
  // Convert
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(imarker_pose_, pose_msg);

  imarker_server_->setPose(int_marker_.name, pose_msg);
  imarker_server_->applyChanges();
}

void RemoteControl::rvizDashboardCallback(const dashboard_msgs::DashboardControl::ConstPtr &msg)
{
  if (msg->next_step)
    debug_interface_->setReadyForNextStep();
  else if (msg->auto_step)
    debug_interface_->setAutonomous();
  else if (msg->full_auto)
    debug_interface_->setFullAutonomous();
  else if (msg->stop)
  {
    softMotionStop();
    debug_interface_->setStop();
  }
  else
    dashboard_callback_(msg);
}

void RemoteControl::gameControllerCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  // Table of index number of /joy.buttons: ------------------------------------
  // 0 - A
  if (msg->buttons[0])
    debug_interface_->setReadyForNextStep();
  // 1 - B
  if (msg->buttons[1])
    softMotionStop();
  // 2 - X
  // 3 - Y
  // 4 - LB
  // 5 - RB
  // 6 - back
  if (msg->buttons[6])  // Reset imarker
  {
    updateIMarkerPose(getCurrentPose());
  }
  // if (msg->buttons[6])
  // 7 - start
  // 8 - power
  if (msg->buttons[8])
    debug_interface_->setFullAutonomous();
  // 9 - Button stick left - does not exist on Logitech Wireless Gamepad
  // 10 - Button stick right - does not exist on Logitech Wireless Gamepad

  // Table of index number of /joy.axes: ------------------------------------
  // 4 - Left/Right Axis stick right
  // 0 - Left/Right Axis stick left
  // 1 - Up/Down Axis stick left
  // ? - Up/Down Axis stick right
  // 6 - cross key left/right
  // 7 - cross key up/down

  // Depends on controller:
  game_controller_x_ = msg->axes[4];
  game_controller_y_ = msg->axes[3];
  game_controller_z_ = msg->axes[7];
  game_controller_rol_ = msg->axes[6];
  game_controller_pit_ = msg->axes[1];
  game_controller_yaw_ = msg->axes[0];
}

void RemoteControl::poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_DEBUG_STREAM_THROTTLE_NAMED(5, name_, "Recieved pose message callback");

  // Convert to eigen
  Eigen::Affine3d input_pose;
  tf::poseMsgToEigen(msg->pose, input_pose);

  // Publish raw input as TF
  tf::Transform pose_transform;
  tf::poseEigenToTF(input_pose, pose_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(pose_transform, ros::Time::now(), "world", "pose_input"));

  bool deadman_enabled = false;  // motion allowed when enabled

  // Button press modes
  if (msg->header.frame_id == "home")
  {
    teleop_callback_(1);  // hook to parent class
    return;
  }
  else if (msg->header.frame_id == "control")
  {
    deadman_enabled = true;
  }
  else if (msg->header.frame_id == "")
  {
    // Motion disabled
    // Unless you sent a home command already, which will keep going
    deadman_enabled = false;
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unknown vive controller command mode: " << msg->header.frame_id);
  }

  // Motion modes
  if (deadman_enabled)  // button pressed - control enabled
  {
    if (!deadman_prev_enabled_)  // deadman was just enabled since last callback
    {
      deadman_prev_enabled_ = true;

      // reset requested
      pose_msg_previous_ = input_pose;
    }
  }
  else  // button not pressed, motion not allowed
  {
    if (deadman_prev_enabled_)
    {
      deadman_prev_enabled_ = false;

      // Reset iMarker
      updateIMarkerPose(getCurrentPose());

      // Stop motion
      if (motion_stop_callback_)
        motion_stop_callback_();
      else
        ROS_ERROR_STREAM_NAMED(name_, "No stop motion callback provided");
    }
  }

  // Show visual icon if button pressed
  const std::size_t static_id = 1;  // overwrite the spheres as we publish them. do not use zero
  visual_tools_->publishSphere(input_pose, rvt::RED, deadman_enabled ? rvt::XXLARGE : rvt::LARGE, "Sphere", static_id);

  // Do not command if deadman is off
  if (!deadman_enabled)
    return;

  // If motion allowed, calculate change
  poseMsgControl(input_pose);
}

void RemoteControl::poseMsgControl(const Eigen::Affine3d &input_pose)
{
  // Set desired pose to current imarker location
  Eigen::Affine3d desired_pose = imarker_pose_;

  // METHOD 1 ------------------------------------------------------------------
  /*
  // Calculate offset
  Eigen::Vector3d trans_diff = pose_msg_previous_.translation() - input_pose.translation();

  // Change frames of reference
  Eigen::Vector3d trans_diff_converted;
  trans_diff_converted[0] = -1.0 * trans_diff[0];  // world Z
  trans_diff_converted[1] = -1.0 * trans_diff[1];  // wolrd y
  trans_diff_converted[2] = -1.0 * trans_diff[2];  // world x

  // Apply to desired pose
  desired_pose.translation() += trans_diff_converted;

  // Save last location
  pose_msg_previous_ = input_pose;
  */

  // METHOD 2 ------------------------------------------------------------------

  // Calculate offset
  Eigen::Affine3d trans_diff = pose_msg_previous_.inverse() * input_pose;
  desired_pose = desired_pose * trans_diff;  // Apply to desired pose

  // Save last location
  pose_msg_previous_ = input_pose;

  // Send
  moveIMarkerAndExecute(desired_pose);
}

void RemoteControl::make6DofMarker(const geometry_msgs::Pose &pose)
{
  int_marker_.header.frame_id = "world";
  int_marker_.pose = pose;
  int_marker_.scale = 0.3;

  int_marker_.name = "6dof_teleoperation";
  int_marker_.description = "moveit_teleop";

  // insert a box
  //makeBoxControl(int_marker_);

  // insert mesh of robot's end effector
  makeEEControl(int_marker_);
  int_marker_.controls[0].interaction_mode = InteractiveMarkerControl::MENU;

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  imarker_server_->insert(int_marker_);
  imarker_server_->setCallback(int_marker_.name, boost::bind(&RemoteControl::iMarkerCallback, this, _1));
  menu_handler_.apply(*imarker_server_, int_marker_.name);
}

visualization_msgs::InteractiveMarkerControl &RemoteControl::makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = msg.scale * 0.15;  // x direction
  marker.scale.y = msg.scale * 0.25;  // y direction
  marker.scale.z = msg.scale * 0.5;   // height
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl &RemoteControl::makeEEControl(visualization_msgs::InteractiveMarker &msg)
{
  // Use the mesh of the robot's end effector as the IMarker control
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_use_embedded_materials = true;

  // Set filename of mesh
  const moveit::core::LinkModel* ee_link = visual_tools_->getRobotModel()->getLinkModel(imarker_robot_link_);
  marker.mesh_resource = ee_link->getVisualMeshFilename();
  ROS_DEBUG_STREAM_NAMED(name_, "Loading interactive marker mesh from file '" << marker.mesh_resource << "'");
  //"file:///home/dave/ros/current/ws_iiwa/src/iiwa_7_r800/iiwa_7_r800_description/meshes/iiwa_7_r800/visual/link_7.dae";

  // Set scale
  marker.scale.x = ee_link->getVisualMeshScale()[0];
  marker.scale.y = ee_link->getVisualMeshScale()[0];
  marker.scale.z = ee_link->getVisualMeshScale()[0];

  // Set color
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

void RemoteControl::iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // Only allow one feedback to be processed at a time
  {
    boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
    if (!teleoperation_ready_)
    {
      return;
    }
    teleoperation_ready_ = false;
  }

  // Convert
  Eigen::Affine3d robot_ee_pose;
  tf::poseMsgToEigen(feedback->pose, robot_ee_pose);

  // Offset ee pose forward, because interactive marker is a special thing in front of hand
  robot_ee_pose = robot_ee_pose * imarker_offset_;

  // Redirect to base class
  imarker_callback_(feedback, robot_ee_pose);

  // Allow next feedback to be processed
  {
    boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
    teleoperation_ready_ = true;
  }
}

Eigen::Affine3d RemoteControl::getEEPose()
{
  return imarker_pose_ * imarker_offset_.inverse();
}
Eigen::Affine3d RemoteControl::getIMPose()
{
  return imarker_pose_;
}

void RemoteControl::gameControllerThread(const ros::TimerEvent &e)
{
  bool marker_changed = false;
  Eigen::Affine3d new_pose = imarker_pose_;
  if (game_controller_y_ != 0.0)
  {
    marker_changed = true;
    new_pose.translation().y() += game_controller_y_ * game_controller_speed_;
  }

  if (game_controller_z_ != 0.0)
  {
    new_pose.translation().z() += game_controller_z_ * game_controller_speed_;
    marker_changed = true;
  }

  if (game_controller_x_ != 0.0)
  {
    new_pose.translation().x() += game_controller_x_ * game_controller_speed_;
    marker_changed = true;
  }

  // Roll
  if (game_controller_rol_ != 0.0)
  {
    ROS_DEBUG_STREAM_NAMED("remote_control",
                           "Changing roll by " << game_controller_rol_ * game_controller_rotation_speed_ << " radians");
    new_pose =
        new_pose * Eigen::AngleAxisd(game_controller_rol_ * game_controller_rotation_speed_, Eigen::Vector3d::UnitX());
    marker_changed = true;
  }

  // Pitch
  if (game_controller_pit_ != 0.0)
  {
    ROS_DEBUG_STREAM_NAMED("remote_control", "Changing pitch by " << game_controller_pit_ *
                                                                         game_controller_rotation_speed_ << " radians");
    new_pose =
        new_pose * Eigen::AngleAxisd(game_controller_pit_ * game_controller_rotation_speed_, Eigen::Vector3d::UnitY());
    marker_changed = true;
  }

  // Yaw
  if (game_controller_yaw_ != 0.0)
  {
    ROS_DEBUG_STREAM_NAMED("remote_control",
                           "Changing yaw by " << game_controller_yaw_ * game_controller_rotation_speed_ << " radians");
    new_pose =
        new_pose * Eigen::AngleAxisd(game_controller_yaw_ * game_controller_rotation_speed_, Eigen::Vector3d::UnitZ());
    marker_changed = true;
  }

  // Process command
  if (marker_changed)
  {
    moveIMarkerAndExecute(new_pose);
  }
}

void RemoteControl::moveIMarkerAndExecute(const Eigen::Affine3d &pose)
{
  // Copy new pose to desired
  imarker_pose_ = pose;

  // Update iMarker in rviz
  sendUpdatedIMarkerPose();

  // Convert
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(imarker_pose_, pose_msg);

  // Fake a iMarker callback to be executed on hardware
  visualization_msgs::InteractiveMarkerFeedbackPtr feedback(new InteractiveMarkerFeedback);
  feedback->pose = pose_msg;
  feedback->event_type = InteractiveMarkerFeedback::POSE_UPDATE;
  iMarkerCallback(feedback);
}

void RemoteControl::softMotionStop()
{
  // Call event if necessary
  if (motion_stop_callback_)
  {
    motion_stop_callback_();
  }
}

moveit::core::RobotStatePtr RemoteControl::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

const Eigen::Affine3d &RemoteControl::getCurrentPose()
{
  return getCurrentState()->getGlobalLinkTransform(arm_jmg_->getOnlyOneEndEffectorTip());
}

}  // namespace moveit_teleop
