# Running ROS nodes

Before you can use the contents of your workspace you need to source it.
You can have multiple different workspaces in various stages of development or use at any one time. ROS needs to know which one you are mean when you run its commands.

Using the terminal from the workspace directory.

1. Source the workspace.
   - `source install/setup.bash`
2. List the executable nodes within the fsai_sim_training package.
   - `ros2 pkg executables fsai_sim_training`
3. Run the demo ROS node.
   - `ros2 run fsai_sim_training demo`

If everything is successful this will run the *demo* node from the *fsai_sim_training* package and something like the following will appear in the terminal:

```
[INFO] [1720708128.676359008] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1720708129.138447369] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1720708129.637026928] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1720708130.138686990] [minimal_publisher]: Publishing: "Hello World: 3"

```

Press Ctrl+C to kill the node.
