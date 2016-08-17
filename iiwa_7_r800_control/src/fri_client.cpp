/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   Fast Robot Interace client from the Kuka SDK that adapts FRI to ROS
*/
#include <stdio.h>
#include <string.h>  // memcpy
#include <iiwa_7_r800_control/fri_client.h>
#include <iostream>
#include <math.h>

// ROS
#include <ros/ros.h>

namespace iiwa_7_r800_control
{
fri_client::fri_client()
{
}

fri_client::~fri_client()
{
}

void fri_client::init()
{
  // pass connection and client to a new FRI client application
  app_.reset(new KUKA::FRI::ClientApplication(connection_, *this));

  // Connect client application to KUKA Sunrise controller.
  // Parameter NULL means: repeat to the address, which sends the data
  const int PORT_ID = 30200;  // default set by KUKA
  app_->connect(PORT_ID, NULL);
}

void fri_client::onStateChange(KUKA::FRI::ESessionState old_state, KUKA::FRI::ESessionState new_state)
{
  LBRClient::onStateChange(old_state, new_state);

  state_change_callback_(old_state, new_state);
}

bool fri_client::step()
{
  // Pass through the next tick
  return app_->step();
}

void fri_client::monitor()
{
  LBRClient::monitor();

  const bool write = true;
  hw_interface_callback_(false);
}

void fri_client::waitForCommand()
{
  const bool write = true;
  hw_interface_callback_(false);

  // In waitForCommand(), the joint values have to be mirrored. Which is done,
  // by calling the base method.
  LBRClient::waitForCommand();
}

void fri_client::command()
{
  // Call ROS to read/compute/write via this callback
  const bool write = true;
  hw_interface_callback_(write);
}

}  // namespace iiwa_7_r800_control
