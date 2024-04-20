# ME5415 Design Project
This repo is the source code to simulate a UR5 with a soft gripper in Gazebo, as to fulfill the requirement of the ME5415 Adv. Soft Robotics module.

## Dependencies
### 1. Install the Universal Robots and moveit packages

    sudo apt install ros-noetic-universal-robots
    sudo apt install ros-noetic-moveit-commander
### 2. clone and catkin make this package

## Usage
### 1. Launch the Gazebo world
    roslaunch me5415_world world.launch
:warning: The Gazebo is set to pause during start up to solve a problem that initial joint config not loaded fast enough, unpause the Gazebo before continuing

### 2. Launch the moveit commander
    roslaunch me5415_world moveit.launch

### 3. Launch the robot controller script
    rosrun me5415_moveit_config robot_controller.py
This is the actual script that controls the robot. There are mainly 4 ways to move the robot:
| Motion Type                 | Descriptipn                                                                         | Remarks                                      |
|-----------------------------|-------------------------------------------------------------------------------------|----------------------------------------------|
| Move to a saved joint       | Supply a saved joint name, currently supported: "source_table" and "assemble_table" |                                              |
| Move via joint              | Supply a list of joint_goal (list of 6 elements)                                    |                                              |
| Move via cartesian          | Supply a geometry_msgs/Pose of a Cartesian coordinate, wrt to the "world" frame     | Uses IK to calculate, might have singularity |
| Move in cartesian direction | Fine tuning the cartesian pose, eg: move along z+ by 10cm                           | Uses IK to calculate, might have singularity |

This is the main part to add in coordinate to complete the simulation

    # to get the current joint angle of the robot
    rostopic echo /eff_joint_traj_controller/state