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

## Packages in this example workspace.

Listed in alphabetical order, not priority.

**fsai_common**

Containing common functionality that the vehicle requires, both simulated and real.
For example common configuration files, or control code for missions such as STATIC_INSPECTION_A which functions identically in both simulation and reality.

**fsai_messages**

Containing message and service definitions. 
These describe the formats of the messages that the various ROS nodes will use to communicate with each other.

These are kept in their own package because multiple packages need to know the message formats.

**ros2_differential_drive**

An example of a ROS package handling a specific piece of functionality. 
In this case the package performs odometry calculations based on wheel rotations.

Importantly, this was a originally a pre-existing node produced by the ROS community. We were able to take an modify the node to meet our specific needs rather than having to create everyting from scratch.

**webots_fsai**

Containing the files needed to simulate the vehicle in the Webots simulator. 
As much as possible the simulated vehicle attempts to duplicate the behavior of the real vehicle, e.g. producing the same messages (as defined in fsai_messages) that the real vehicle would.
