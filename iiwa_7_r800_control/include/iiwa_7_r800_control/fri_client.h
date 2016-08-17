/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   Fast Robot Interace client from the Kuka SDK that adapts FRI to ROS
*/

#ifndef IIWA_7_R800_CONTROL_FRI_CLIENT_H
#define IIWA_7_R800_CONTROL_FRI_CLIENT_H

#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <memory>

namespace iiwa_7_r800_control
{
typedef std::function<void(bool)> ControlCycleCallback;
typedef std::function<void(KUKA::FRI::ESessionState, KUKA::FRI::ESessionState)> StateChangeCallback;

/**
 * \brief Template client implementation.
 */
class fri_client : public KUKA::FRI::LBRClient
{
public:
  /**
   * \brief Constructor.
   */
  fri_client();

  /**
   * \brief Destructor.
   */
  virtual ~fri_client();

  /** \brief Initialize. */
  void init();

  /**
   * \brief Callback for FRI state changes.
   * @param oldState
   * @param newState
   */
  virtual void onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state);

  /**
   * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
   * If you do not want to change the default-behavior, you do not have to implement this method.
   */
  virtual void monitor();

  /**
   * \brief Callback for the FRI session state 'Commanding Wait'.
   * If you do not want to change the default-behavior, you do not have to implement this method.
   */
  virtual void waitForCommand();

  /**
   * \brief Callback for the FRI state 'Commanding Active'.
   * If you do not want to change the default-behavior, you do not have to implement this method.
   */
  virtual void command();

  /** \brief Tell the FRI application to move to the next step */
  bool step();

  /** \brief Set a function to call every control cycle */
  void setControlCycleCallback(ControlCycleCallback hw_interface_callback)
  {
    hw_interface_callback_ = hw_interface_callback;
  }

  /** \brief Set a function to call when the state changes */
  void setStateChangeCallback(StateChangeCallback state_change_callback)
  {
    state_change_callback_ = state_change_callback;
  }

private:
  // create new udp connection
  KUKA::FRI::UdpConnection connection_;

  // pass connection and client to a new FRI client application
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  // Callback to communicate with parent hardware interface class in realtime loop
  ControlCycleCallback hw_interface_callback_;

  // Callback to communicate with parent hardware interface class when state changes
  StateChangeCallback state_change_callback_;
};

}  // namespace iiwa_7_r800_control

#endif  // IIWA_7_R800_CONTROL_FRI_CLIENT_H
