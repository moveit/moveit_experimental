# MoveIt! Teleop

This package allows a robot's end effector pose to be controlled with [interactive markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers:%20Getting%20Started) and other intputs in realtime, while still using ROS topics to publish *joint* trajectory commands to the robot's controllers. It can send new joint trajectories at 100 hz while maintaining a smooth end effector pose.

Features Include:

 - Ability to save and playback cartesian trajectories to/from file
 - Proper use of ROS topics for separating higher level logic from ros_control, etc low level controllers
 - Use of [KDL IK Solver](http://people.mech.kuleuven.be/~rsmits/kdl/api/html/index.html) combined with MoveIt!'s cartesian path planner
 - Robot agnostic
 - Use of spiffy configuration yaml files

## Usage

IIWA-specific example:

### Simulation

Start IIWA's ros_control non-physics-based simulated hardware interface:

    roslaunch iiwa_7_r800_control iiwa_7_r800_simulation.launch

### Hardware

Connect to KUKA's robot hardware. Be sure to select the correct _ROSFRI program on the pendant and press play:

    roslaunch iiwa_7_r800_control iiwa_7_r800_hardware.launch

### Visualize

Start Rviz:

    roslaunch moveit_teleop iiwa_7_r800_visualize.launch

### Applicaiton

### Applicaiton

Then the main program:

    roslaunch moveit_teleop iiwa_7_r800_teleop.launch

### Interactive Marker Teleop

To enable the interactive markers, press the "Interact" button on the top tool bar.

You should now see red, green, and blue arrows on the top of the robot. You can now drag the arrows around to control.

You might notice multiple arms moving at the same time, they represent the following:
 - Semi-transparent regular color: this is the simulated arm moving via ros_control, based on the ``/joint_states`` topic
 - Blue: this is the IK-solved goal state based on the interactive marker
 - Other: these are debugging robots used in development you can disable by unchecking State Display Plugins on the left.

To reset the interactive marker, right click on the interactive marker to see a popup menu.

### Joystick Teleop

Control end effector position with a Xbox / Logitech / etc game controller. If you go beyond the limits with the interactive marker, press the "BACK" button to reset the imaker location. To use a game controller, also launch this node:

    rosrun joy joy_node

Or enable the line in ``moveit_teleop/launch/iiwa_7_r800_teleop.launch``.

### Cartesian Trajectory Recording & Playback

Right click on the interactive marker to see a popup menu with recording and playback options. There are two methods to record a trajectory:

- Record single waypoints. This will cause each point to be played back in 2 second intervals (you can adjust in the csv file).
- Record live trajectory as moved by imarker. This will play back the trajectory exactly as you moved the arm around

To automatically play:

    roslaunch moveit_teleop iiwa_7_r800_teleop.launch play_traj:=1

To record multiple trajectories and save/load them from different files, edit the yaml file:

    moveit_teleop/config/config_iiwa_7_r800.yaml

And find the configuration labled ``waypoints_file``.

## Sending to a specific pose in SRDF

You can define poses using the MoveIt setup assistant, such as "home", then move to them using a command line argument:

    roslaunch moveit_teleop iiwa_7_r800_teleop.launch pose:=home

## Configuring

You can find lots of configuration options in ``moveit_teleop/config/config_ROBOT.yaml``. Of particular interest is you'll need to change the trajectory file path ``waypoints_file`` to a location on your computer.

## Benchmarking

To benchmark, there are two CSV data collection locations.

To record the controller commands and actual hardware state, simply run:

    roslaunch iiwa_7_r800_control iiwa_7_r800_log_controller.launch

Edit that file to change what CSV file to write to and where.

To record the output from MoveIt! before it is post-processed by the quintic spline joint trajectory controller, enable a setting in ``config_iiwa_7_r800.yaml``:

    execution_interface:
      save_traj_to_file: true
      save_traj_to_file_path: /home/dave/ros/current/ws_iiwa/src/iiwa_trajectory_data/

It will save each trajectory it streams to the ros_control node into a new csv file named ``arm_moveit_trajectory_1.csv`` with a new number a the end, but it overwrites between node restarts.

To anaylize in Matlab, use the script located in ``iiwa_7_r800/iiwa_7_r800_control/scripts/analyze_controller_state.m``.
