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
2. Build workspace.
   - `colon build`
