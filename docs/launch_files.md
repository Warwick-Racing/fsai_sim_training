# Launch files

Whilst it is possible to run ROS nodes individually, it quickly becomes unmanageable for anything but the smallest of projects. 
Instead we run launch files that contain the instructions to run and correctly configure multiple nodes.

Launch files are the main way that you will interact with ROS. 
For example, a simple launch file that only uses functionality from the `webots_fsai` package.

From the terminal in the workspace directory.
1. Run the launch file. This will launch multiple ROS nodes.
    - `ros2 launch webots_fsai demo_simple.py`

If everything is successful the webots simulator should launch and the simulated vehicle should start moving.

The vehicle is going to swerve off the track almost instantly but that's because the control command is a constant go forwards while steering left. The point is that the vehicle has moved.

##


This launch file consists of 3 parts.

1. webots_launch
    - This calls the `webots_fsai` `start_simulator` launch file which in turns starts the webots simulator and runs the webots/ROS driver node.
2. vehicle_launch.
    - This calls the `webots_fsai` `start_vehicle` launch file and emulates vehicle start up process from the real vehicle.
3. ctrl
    - Publishes a simple control commmand to the vehicle.

Launch files can include other launch files so it makes sense to break functionality into separate launch files and to them combine as needed. This is especially useful when certain functionality is going to be reused, e.g. starting the simulator.

The `start_simulator` launch file is a good example of this. It is used in all the other launch files that start the simulator, it:

1. Loads webots.
2. Runs the driver node that translates between webots and ROS.
3. Tidies up when the simulation is closed.

The `start_vehicle` launch file is another example. The webots vehicle emulates the real vehicle start up process so that we can test how the rest of the software stack responds. But most of the time we just want to get the vehicle moving. The `start_vehicle` launch file does just that by automatically:

1. Powering on the vehicle.
2. Selecting a mission.
3. Making the vehicle active. 

