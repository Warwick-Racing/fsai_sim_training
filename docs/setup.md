# Setting up

Using the terminal.

1. Create the folders.
    - `mkdir -p training_ws/src`
2. Move to src/ directory.
   - `cd training_ws/src`
3. Clone the Git repos containing the packages.
   - `git clone git@github.com:Warwick-Racing/fsai_common.git`
   - `git clone git@github.com:Warwick-Racing/fsai_messages.git`
   - `git clone git@github.com:Warwick-Racing/fsai_sim_training.git`
   - `git clone git@github.com:Warwick-Racing/ros2_differential_drive.git`
   - `git clone git@github.com:Warwick-Racing/webots_fsai.git` 
4. Return to root dir of workspace.
   - `cd ..`


# Building the workspace

1. Ensure that ROS humble is sourced.
   - `source /opt/ros/humble/setup.bash`
2. If you have not done so yet, install other dependencies for ROS Humble.
   - `sudo apt-get install ros-humble-ackermann-msgs`
   - `sudo apt-get install ros-humble-webots-ros2-driver`
   - `sudo apt-get install ros-humble-tf-transformations`
3. Build workspace.
   - `colcon build`
