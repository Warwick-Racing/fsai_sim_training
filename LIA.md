# fsai\_sim\_training

Visualise the materials [HERE](https://liascript.github.io/course/?https://warwick-racing.github.io/fsai_sim_training/LIA.md).

The aim of this repo is to guide you through setting up an initial ROS2 workspace to the point where you can run the Warwick fsai simulator and start understanding how ROS2 works at the level of actually running systems.

These instructions assume that you have a basic understanding on the Linux terminal.


## ROS workspace structure

A ROS2 workspace is simple a directory containing the resources needed to run a project.
It will typically consists of multiple ROS2 packages, where each package may be dependent on one or more of the other packages.

A ROS2 package is simple a directory that contains the resources needed to run a specific part of a project.

For the examples in this guide we are going to be making use of the following custom packages.

- fsai_common
- fsai_messages
- fsai_sim_training
- ros2_differential_drive
- webots_fsai

As well as multiple default packages that we will install to the machine and do not need to worry about copying into this workspace.

The folder structure that we are aiming to achieve is shown below, as you can see below each package is a separate folder under the src/ directory.

For an explanation of the purpose of each package see [HERE](docs/packages.md).


``` ascii
+- training_ws/
+-+- install/
  +- src/
  +-+- fsai_common/
    +- fsai_messages/
    +- fsai_sim_training/
    +- ros2_differential_drive/                                      
    +- webots_fsai/
```

### Packages in this example workspace.

Listed in alphabetical order, not priority.

**fsai\_common**

Containing common functionality that the vehicle requires, both simulated and real.
For example common configuration files, or control code for missions such as STATIC_INSPECTION_A which functions identically in both simulation and reality.

**fsai\_messages**

Containing message and service definitions. 
These describe the formats of the messages that the various ROS nodes will use to communicate with each other.

These are kept in their own package because multiple packages need to know the message formats.

**ros2\_differential\_drive**

An example of a ROS package handling a specific piece of functionality. 
In this case the package performs odometry calculations based on wheel rotations.

Importantly, this was a originally a pre-existing node produced by the ROS community. We were able to take an modify the node to meet our specific needs rather than having to create everyting from scratch.

**webots\_fsai**

Containing the files needed to simulate the vehicle in the Webots simulator. 
As much as possible the simulated vehicle attempts to duplicate the behavior of the real vehicle, e.g. producing the same messages (as defined in fsai_messages) that the real vehicle would.


## Setting up the workspace

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


### Building the workspace

1. Ensure that ROS humble is sourced.
  - `source /opt/ros/humble/setup.bash`
2. If you have not done so yet, install other dependencies for ROS Humble.
  - `sudo apt-get install ros-humble-ackermann-msgs`
  - `sudo apt-get install ros-humble-webots-ros2-driver`
  - `sudo apt-get install ros-humble-tf-transformations`
3. Build workspace.
  - `colcon build`


## Running ROS nodes

Before you can use the contents of your workspace you need to source it.
You can have multiple different workspaces in various stages of development or use at any one time. ROS needs to know which one you are mean when you run its commands.

Using the terminal from the workspace directory.

1. Source the workspace.
  - `source install/setup.bash`
2. List the executable nodes within the fsai_sim_training package.
  - `ros2 pkg executables fsai_sim_training`
3. Run the demo ROS node.
  - `ros2 run fsai_sim_training demo`

If everything is successful this will run the *demo* node from the *fsai\_sim\_training* package and something like the following will appear in the terminal:

```
[INFO] [1720708128.676359008] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1720708129.138447369] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1720708129.637026928] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1720708130.138686990] [minimal_publisher]: Publishing: "Hello World: 3"

```

Press Ctrl+C to kill the node.


## Launch files

Whilst it is possible to run ROS nodes individually, it quickly becomes unmanageable for anything but the smallest of projects. 
Instead we run launch files that contain the instructions to run and correctly configure multiple nodes.

Launch files are the main way that you will interact with ROS. 
For example, a simple launch file that only uses functionality from the `webots_fsai` package.

From the terminal in the workspace directory.

1. Run the launch file. This will launch multiple ROS nodes.

    - `ros2 launch webots_fsai demo_simple.py`

If everything is successful the webots simulator should launch and the simulated vehicle should start moving.

The vehicle is going to swerve off the track almost instantly but that's all we are telling the vehicle is do it go forwards while steering left. The point is that the vehicle has moved.

Press Ctrl+C to stop the simulation.

![](docs/images/demo_simple_running.png)


### demo_simple.py

The `demo_simple.py` launch file consists of 3 parts.

1. webots_launch

    - This calls the `webots_fsai` `start_simulator` launch file which in turns starts the webots simulator and runs the webots/ROS driver node.

2. vehicle_launch.

    - This calls the `webots_fsai` `start_vehicle` launch file and emulates vehicle start up process from the real vehicle.

3. ctrl

    - Publishes a simple control commmand to the vehicle, in this case go forwards while steering left.

### start_simulator.py

Launch files can include other launch files so it makes sense to break functionality into separate launch files and to them combine as needed. This is especially useful when certain functionality is going to be reused, e.g. starting the simulator.

The `start_simulator` launch file is a good example of this. It is used in all the other launch files that start the simulator, it:

1. Loads webots.
2. Runs the driver node that translates between webots and ROS.
3. Tidies up when the simulation is closed.

### start_vehicle.py

The `start_vehicle` launch file is another example. The webots vehicle emulates the real vehicle start up process so that we can test how the rest of the software stack responds. But most of the time we just want to get the vehicle moving. The `start_vehicle` launch file does just that by automatically:

1. Powering on the vehicle.
2. Selecting a mission.
3. Making the vehicle active (toggling the grossfunk).



### ROS graph

So our `demo_simple.py` launch file ends up running multiple seperate nodes that are all working together to make the vehicle move.

The diagram below shows how the nodes are connected to each other.

- Webots is the simulator we are using.
- webots_driver is the node that translates between webots and ROS.
  - If we changed the simulation software we would need to change this node.
  - But nothing else would need to change.
- power, select_mission and toggle_grossfunc send one off commands (ROS service calls) to the vehicle to step through the start up process.
- ctrl sends a continuous stream of data (ROS topic) with desired speed, steering and braking information.
  - For safety reasons on the real vehicle, the vehicle automatically stops if it does not receive a control command for a certain amount of time.
  - The simulated vehicle has no safety concerns but works the same way because otherwise it wouldn't be an accurate simulation.

This is the simplest possible setup we can use to get the vehicle moving in the simulator.

``` ascii                             
+-----------+   .-- ROS -----------------------------------------------.
|           |   |   +---------------+              +------+            |
|  Webots   *------>|               |              |      |            |       
| Simulator |<------* webots_driver |<--- /ctrl ---* ctrl |            |
|           |   |   |               |              |      |            |      
+-----------+   |   +---------------+              +------+            |
                |       ^       ^ ^                                    |
                |       |       |  \                                   |
                |       |       |   .------------.                     |
                |       |       |                 \                    |
                |   +---o---+ +-o--------------+ +-o----------------+  |
                |   |       | |                | |                  |  |
                |   | power | | select_mission | | toggle_grossfunc |  |
                |   |       | |                | |                  |  |
                |   +-------+ +----------------+ +------------------+  |
                .------------------------------------------------------.
```

## Next steps

Try running an example that is a bit more complex and closer to what we would be running on the real vehicle.

One possible architecture is to have a single launch file that runs the nodes for all possible missions. 
However only one mission is actually active at a given time. 
In this example, each mission has a seperate node that outputs the control commands for that mission.
These are all passed to the control_multiplexer node which forwards the commands from the mission that is actually selected at that time.

The static_a, static_b, autonomous_demo, acceleration and control_multiplexer nodes are all listening (subscribing) to the /status topic.

The /status topic contains the information that the car generates about its current state. 
I.e. current mission, current speed/steering/brakes, wheel ticks etc.
The mission nodes need this information to complete successfully.
For example autonomous_demo needs to know when it has travelled the required distance.

In this example 

`ros2 launch fsai_sim_training example_manual.py`

!?[](docs/videos/manual_control.mp4)

example_manual

``` ascii                             
                .-- ROS ---------------------------------------------------------.
                |   +----------------+                                           |
                |   |                |                  +----------+             |
                |   | manual_control |                  |          |             |
                |   |                |            .---->| static_a *----------.  |
                |   +-------o--------+            |     |          |          |  |
                |           |                     |     +----------+          |  |
+-----------+   |           V                     |                           |  |
|           |   |   +---------------+             |     +----------+          |  |
|  Webots   *------>|               |             |     |          |          |  |   
| Simulator |<------* webots_driver *-- /status --+---->| static_b *--------. |  |
|           |   |   |               |             |     |          |        | |  |      
+-----------+   |   +---------------+             |     +----------+        | |  |
                |           ^                     |                         | |  |
                |           |                     |  +-----------------+    | |  |
                |         /ctrl                   |  |                 |    | |  |
                |           |                     +->| autonomous_demo *--. | |  |
                |   +---------------------+       |  |                 |  | | |  |
                |   |                     |       |  +-----------------+  | | |  |
                |   | control_multiplexer |<------+                       | | |  |
                |   |                     |       |   +--------------+    | | |  |
                |   +---------------------+       |   |              |    | | |  |
                |             ^ ^ ^ ^             .-->| acceleration *--. | | |  |
                |             | | | |                 |              |  | | | |  |
                |             | | | |                 +--------------+  | | | |  |
                |             | | | |                                   | | | |  |
                |             | | | .-- /accel_ctrl --------------------. | | |  |
                |             | | .---- /demo_ctrl -----------------------. | |  |
                |             | .------ /static_b_ctrl ---------------------. |  |
                |             .-------- /static_a_ctrl -----------------------.  |
                .----------------------------------------------------------------.
```



## Advanced

``` ascii                             
                .-- ROS -----------------------------------------------.
                |   +-------+ +----------------+ +------------------+  |
                |   |       | |                | |                  |  |
                |   | power | | select_mission | | toggle_grossfunc |  |
                |   |       | |                | |                  |  |
                |   +---o---+ +-o--------------+ +-o----------------+  |
                |       |       |                 /                    |
                |       |       |   .------------.                     |
                |       |       |  /                                   |
+-----------+   |       V       V V                                    |
|           |   |   +---------------+              +----------+        |
|  Webots   *------>|               |              |          |        |       
| Simulator |<------* webots_driver *-- /cones --->| delaunay |        |
|           |   |   |               |              |          |        |      
+-----------+   |   +---------------+              +----*-----+        |
                |      ^                                |              |
                |      |                             /centers          |
                |    \ctrl                              |              |
                |      |                                V              |
                |   +--*---+                      +-------------+      |
                |   |      |                      |             |      |
                |   | ctrl |<---- /steer_angle ---* breadcrumbs |      |
                |   |      |                      |             |      |
                |   +------+                      +-------------+      |
                |                                                      |
                .------------------------------------------------------.
```



