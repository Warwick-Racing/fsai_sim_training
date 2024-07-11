# fsai_sim_training

The aim of this repo is to guide you through setting up an initial ROS2 workspace to the point where you can run the Warwick fsai simulator and start understanding how ROS2 works at the level of actually running systems.

## Setting up

### Aim

A ROS2 workspace typically consits of multiple dependent ROS2 packages. 
For the examples in this guide we are going to be making use of the following packages.

- fsai_common
- fsai_messages
- fsai_sim_training
- ros2_differential_drive
- webots_fsai

As well as multiple other default packages that we will install to the machine and do not need to worry about copying into this workspace.

The folder structure that we are aiming to achieve is shown below, as you can see below each package is a seperate folder under the src/ directory.

For an explaination of the purpose of each package see [HERE](docs/packages.md).

```
training_ws/
|- install/
|- src/
  |- fsai_common/
  |- fsai_messages/
  |- fsai_sim_training/
  |- ros2_differential_drive/
  |- webots_fsai/
```

### Steps

These and all other steps are assuming that you are using a Linux system, probably Ubuntu. 
Or, at the very least, Windows with WSL and Ubuntu configured.

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


## Building the workspace

5. Ensure that ROS humble is sourced.
   - `source /opt/ros/humble/setup.bash`
5. Build workspace.
   - `colon build`


## Running ROS systems

Before you can use the contents of your workspace you need to source it.
You can have multiple different workspaces in various stages of development or use at any one time. ROS needs to know which one you are mean when you run its commands.

Using the terminal from the workspace directory.

1. Source the workspace.
   - `source install/setup.bash`
2. Run a ROS node.
   - `ros2 pkg executables fsai_sim_training demo`

If everything is successful this will run the *demo* node from the *fsai_sim_training* package and something like the following will appear in the terminal:

```
[INFO] [1720708128.676359008] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1720708129.138447369] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1720708129.637026928] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1720708130.138686990] [minimal_publisher]: Publishing: "Hello World: 3"

```

Press Ctrl+C to kill the node.


### Launch files

Whilst it is possible to run ROS nodes individually, it quickly becomes unmanagable for anything but the smallest of projects. 
Instead we run launch files that contain the instructions to run and correctly configure multiple nodes.

Launch files are the main way that you will interact with ROS. 
For example, a simple launch file that only uses functionality from the webots_fsai package.

From the terminal in the workspace directory.

1. Run the launch file.
    - `ros2 launch webots_fsai demo_full.py`

If everything is successful the webots simulator should launch and the simulated vehicle should start moving.


##




This launch file consists of 3 parts.



1. webots_launch
    - This calls the webots_fsai start_simulator launch file which in turns starts the webots simulator and runs the webots/ROS driver node.
2. vehicle_launch.
    - This calls the webots_fsai start_vehicle launch file and emulates vehicle start up process from the real vehicle.
3. ctrl
    - Publishes a simple control commmand to the vehicle.





Launch files can include other launch files so it makes sense to break functionality into seperate launch files and to them combine as needed.

