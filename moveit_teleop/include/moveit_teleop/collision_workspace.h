/* Copyright 2015 Google Inc.
   Author: Dave Coleman
   Desc:   Collision workspace at for IIWA
*/

#ifndef MOVEIT_TELEOP_COLLISION_WORKSPACE_H
#define MOVEIT_TELEOP_COLLISION_WORKSPACE_H

// C++
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_teleop
{
class CollisionWorkspace
{
public:
  /**
   * \brief Constructor
   */
  CollisionWorkspace(ros::NodeHandle nh, moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
    : name_("collision_workspace"), nh_(nh), visual_tools_(visual_tools)
  {
    std::vector<double> object1_dimensions_6;
    std::vector<double> object2_dimensions_6;
    std::vector<double> object3_dimensions_6;

    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "object1_dimensions_6", object1_dimensions_6);
    error += !rosparam_shortcuts::get(name_, rpnh, "object2_dimensions_6", object2_dimensions_6);
    error += !rosparam_shortcuts::get(name_, rpnh, "object3_dimensions_6", object3_dimensions_6);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Add objects to planning scene for robot to avoid
    addCollisionObjects(object1_dimensions_6, rviz_visual_tools::BLACK, "table");
    addCollisionObjects(object2_dimensions_6, rviz_visual_tools::BLUE, "screen");
    addCollisionObjects(object3_dimensions_6, rviz_visual_tools::GREY, "table2");

    // Trigger update
    ros::spinOnce();
    visual_tools_->triggerPlanningSceneUpdate();
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    ROS_INFO_STREAM_NAMED(name_, "CollisionWorkspace Ready.");
  }

  void addCollisionObjects(const std::vector<double> &object_dimensions_6, const rviz_visual_tools::colors &color,
                           const std::string &name)
  {
    ROS_INFO_STREAM_NAMED(name_, "Adding collision object '" << name << "'");
    Eigen::Vector3d point1(object_dimensions_6[0], object_dimensions_6[1], object_dimensions_6[2]);
    Eigen::Vector3d point2(object_dimensions_6[3], object_dimensions_6[4], object_dimensions_6[5]);
    visual_tools_->publishCollisionCuboid(point1, point2, name, color);
  }

private:
  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CollisionWorkspace> CollisionWorkspacePtr;
typedef boost::shared_ptr<const CollisionWorkspace> CollisionWorkspaceConstPtr;

}  // namespace moveit_teleop

#endif  // MOVEIT_TELEOP_COLLISION_WORKSPACE_H
