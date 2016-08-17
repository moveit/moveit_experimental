/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   ros_control hardware interface that wraps the Fast Robotics Interface for
           the Kuka LBR IIWA 7 R800
*/

#ifndef IIWA_7_R800_CONTROL_IIWA_HW_INTERFACE_H
#define IIWA_7_R800_CONTROL_IIWA_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <time.h>

// Interface for Kuka LBR IIWA 7 R800
#include <iiwa_7_r800_control/fri_client.h>

namespace iiwa_7_r800_control
{
// \brief Hardware interface for a robot
class IIWAHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /** \brief Currently supported command modes */
  enum CommandMode
  {
    POSITION = 0,
    JOINT_IMPEDANCE
  };

  /**
   * \brief Constructor.
   * \param nh - Node handle for topics.
   */
  IIWAHWInterface(ros::NodeHandle &nh);

  /** \brief Destructor. */
  virtual ~IIWAHWInterface();

  /** \brief Initialize the robot hardware interface */
  void init();

  /** \brief Thread to alert user if connection is having issues */
  void checkConnectivity(const ros::TimerEvent &e);

  /**
   * \brief Handles when the Kuka controller changes from active to monitoring, resetting command
   * \param old_state - the old mode that KUKA controller is in
   * \param new_state - the new mode that KUKA controller is in
   */
  void stateChangeCallback(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state);

  /**
   * \brief Reset the commanded position
   */
  void setHoldPosition();

  /** \brief Reset robot for estops, mode changes, etc */
  void resetCommand();

  /** \brief Callback function for the realtime loop
   *         Based on the speed on the robot controller.
   * \param write - whether or not to send commands back to robot
   */
  void command(bool write_output);

  /** \brief Pass through for cycling the loop. */
  bool step();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

  /** \brief Realtime control loop.
      The loop's time is based of the robot's control computer clock, not ours.
  */
  void read_write();

  /** \brief End communication with robot. */
  void shutdown();

protected:
  // Name of this class
  std::string name_ = "iiwa_hw_interface";

  // FRI library that communicates to robot over UDP.
  fri_client lbr_client_;

  // Not for actual control loop, only for warning if we aren't meeting correct times
  int monitor_control_freq_ = 0;

  // State variables
  bool initial_startup_ = true;
  bool robot_connected_ = false;
  KUKA::FRI::ESessionState current_state_;

  // Current method for sending commands.
  CommandMode mode_;

  // Speciality states.
  std::vector<double> joint_position_prev_;

  // Elapsed time variables.
  ros::Duration elapsed_time_;
  struct timespec last_time_;
  struct timespec current_time_;
  bool first_loop_ = true;  // Compensate for the lack of previous data.

  // Where various controllers are loaded.
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Alert user if connection is hung
  ros::Timer feedback_connectivity_thread_;

};  // end class

}  // namespace iiwa_7_r800_control

#endif  // IIWA_7_R800_CONTROL_IIWA_HW_INTERFACE_H
