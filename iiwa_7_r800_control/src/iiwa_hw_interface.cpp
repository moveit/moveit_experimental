/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   ros_control hardware interface that wraps the Fast Robotics Interface for
   the Kuka IIWA 7 R800
*/

// Compile options
// #define USE_JOINT_STATUS_DEBUG // Show joint states as read in
// #define USE_JOINT_COMMAND_DEBUG // Show joint commands as written

#include <iiwa_7_r800_control/iiwa_hw_interface.h>
#include <functional>  // std::bind

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace iiwa_7_r800_control
{
// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

IIWAHWInterface::IIWAHWInterface(ros::NodeHandle &nh) : GenericHWInterface(nh)
{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "monitor_control_freq", monitor_control_freq_);
  rosparam_shortcuts::shutdownIfError(name_, error);
}

void IIWAHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  // Alert user if connection is hung in this thread
  const double CONNECTIVITY_THREAD_HZ = 1.0;
  feedback_connectivity_thread_ =
      nh_.createTimer(ros::Duration(1 / CONNECTIVITY_THREAD_HZ), &IIWAHWInterface::checkConnectivity, this);

  // Connect
  ROS_INFO_STREAM_NAMED(name_, "Connecting to Fast Robot Interface at " << monitor_control_freq_ << " hz");
  lbr_client_.init();

  // Set a control cycle callback to preserve Kuka's SDK paradigm
  lbr_client_.setControlCycleCallback(std::bind(&IIWAHWInterface::command, this, std::placeholders::_1));

  // Set a state change callback to preserve Kuka's SDK paradigm
  lbr_client_.setStateChangeCallback(
      std::bind(&IIWAHWInterface::stateChangeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Resize vectors
  joint_position_prev_.resize(num_joints_, 0.0);

  // Load controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  ROS_INFO_NAMED(name_, "Loaded iiwa_hw_interface.");
}

IIWAHWInterface::~IIWAHWInterface()
{
  shutdown();
}

void IIWAHWInterface::checkConnectivity(const ros::TimerEvent &e)
{
  if (!robot_connected_)
  {
    ROS_WARN_STREAM_NAMED(name_, "Waiting for robot connection from FRI client");
  }
}

void IIWAHWInterface::stateChangeCallback(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state)
{
  // This is the best indicator we've gotten a response from the FRI client
  robot_connected_ = true;

  // Cache the state
  current_state_ = new_state;

  // React on state change events
  switch (new_state)
  {
    case KUKA::FRI::MONITORING_WAIT:
    {
      ROS_INFO_STREAM_NAMED(name_, "Robot mode: MONITORING_WAIT  ");
      break;
    }
    case KUKA::FRI::MONITORING_READY:
    {
      ROS_INFO_STREAM_NAMED(name_, "Robot mode: MONITORING_READY  ");
      setHoldPosition();
      break;
    }
    case KUKA::FRI::COMMANDING_WAIT:
    {
      ROS_INFO_STREAM_NAMED(name_, "Robot mode: COMMANDING_WAIT  ");
      if (!initial_startup_)
        resetCommand();
      break;
    }
    case KUKA::FRI::COMMANDING_ACTIVE:
    {
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      ROS_INFO_STREAM_NAMED(name_, "Robot connected and ready. Beep boop beep.");
      initial_startup_ = false;
      break;
    }
    default:
    {
      ROS_ERROR_STREAM_NAMED(name_, "Disconnected from robot");
      robot_connected_ = false;
      break;
    }
  }
}

void IIWAHWInterface::setHoldPosition()
{
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    joint_position_command_[i] = lbr_client_.robotState().getMeasuredJointPosition()[i];  // radians
    joint_velocity_command_[i] = 0.0;
    joint_effort_command_[i] = 0.0;
  }
}

void IIWAHWInterface::resetCommand()
{
  ROS_INFO_STREAM_NAMED(name_, "Resetting joint commands, joint limits, and controllers");

  // Reset commanded position
  setHoldPosition();

  // Reset joint limits state, in case of mode switch or e-stop
  reset();

  // Stop and start all running controllers before updating
  const bool reset_controllers = true;
  controller_manager_->update(ros::Time::now(), elapsed_time_, reset_controllers);
}

void IIWAHWInterface::command(bool write_output)
{
  // Check that node isn't asking for a shutdown
  if (!ros::ok())
  {
    std::cout << "ROS shutting down, control cycle disabled " << std::endl;
    return;
  }

  // Copy current time data
  current_time_.tv_sec = lbr_client_.robotState().getTimestampSec();
  current_time_.tv_nsec = lbr_client_.robotState().getTimestampNanoSec();

  // Calculate elapsed time
  if (first_loop_)
  {
    elapsed_time_ = ros::Duration(0);
    first_loop_ = false;
  }
  else
  {
    elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec +
                                  (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);

    // Check for bad elapsed time
    const double target_seconds = 1.0 / static_cast<double>(monitor_control_freq_);
    static const double TIME_EQUAL_THRESHOLD = 0.001;
    if (fabs(elapsed_time_.toSec() - target_seconds) > TIME_EQUAL_THRESHOLD)
      if (current_state_ == KUKA::FRI::COMMANDING_ACTIVE)
        ROS_ERROR_STREAM_THROTTLE_NAMED(0.1, name_, "Elapsed time is too large: " << elapsed_time_.toSec()
                                                                                  << ", expecting " << target_seconds);
  }
  last_time_ = current_time_;

  // Input
  read(elapsed_time_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  if (write_output)
    write(elapsed_time_);
}

bool IIWAHWInterface::step()
{
  return lbr_client_.step();
}

void IIWAHWInterface::read(ros::Duration &elapsed_time)
{
  // Copy robot state to ROS-based memory
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    joint_position_[i] = lbr_client_.robotState().getMeasuredJointPosition()[i];  // radians
    joint_effort_[i] = lbr_client_.robotState().getMeasuredTorque()[i];           // Nm

    if (elapsed_time.toSec() > 0)
    {
      joint_velocity_[i] = (joint_position_[i] - joint_position_prev_[i]) / elapsed_time.toSec();
    }
    else
      joint_velocity_[i] = 0;

    joint_position_prev_[i] = joint_position_[i];
  }

// Debug only - NOT REALTIME
#ifdef USE_JOINT_STATUS_DEBUG
  std::cout << "State: " << std::endl;
  std::cout << printStateHelper() << std::endl;
#endif
}

void IIWAHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Write
  lbr_client_.robotCommand().setJointPosition(&joint_position_command_[0]);

// Debug only - NOT REALTIME
#ifdef USE_JOINT_COMMAND_DEBUG
  std::cout << "Command:" << std::endl;
  std::cout << printCommandHelper() << std::endl;
#endif
}

void IIWAHWInterface::enforceLimits(ros::Duration &period)
{
  // Enforces SATURATION position and velocity limits
  pos_jnt_sat_interface_.enforceLimits(period);

  // Enforces SOFT position and velocity limits
  pos_jnt_soft_limits_.enforceLimits(period);
}

/** \brief End communication with robot */
void IIWAHWInterface::shutdown()
{
}

}  // namespace iiwa_7_r800_control
