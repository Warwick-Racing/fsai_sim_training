
### Packages in this example workspace.

Listed in alphabetical order, not priority.

**fsai_common**

Containing common functionality that the vehicle requires, both simulated and real.
For example common configuration files, or control code for missions such as STATIC_INSPECTION_A which functions identically in both simulation and reality.

**fsai_messages**

Containing message and service definitions, these are kept in their own package because multiple other packages depend on them.

**ros2_differential_drive**

An example of a ROS package handling a specific piece of functionality. 
In this case the package performs odometry calculations based on wheel rotations.
Importantly, this was a pre-existing node produced by the ROS community. We were able to simply use the existing node as opposed to creating our own.

*In reality we have made some minor tweaks but the point stands*.

**webots_fsai**

Containing the files needed to simulate the vehicle in the Webots simulator. 
As much as possible the simulated vehicle attempts to duplicate the behaviour of the real vehicle, e.g. producing the same messages (as definied in fsai_messages) that the real vehicle would.
