# Assignment 3.0 Alica turtlesim

Go through [the tutorial on alica](https://github.com/rapyuta-robotics/alica/tree/devel/supplementary/alica_ros1/alica_ros_turtlesim).

<b>NOTE</b>: Alica is already part of this repository. Please use this repository as catkin workspace instead of `catkin_ws`.

Try out different master plans, attach screnshots/gifs of results.
Hint: lookup launch files to find out how to set certain master plan.

# Assignment 3.1: Alica turtle in Flatland

In this assignment you are supposed to make alica turtlesim tutorial in [Flatland simulation](https://flatland-simulator.readthedocs.io/en/latest/overview.html).

Go into [alica_ros_flatland_turtle](https://github.com/Gamezar/ros-training/tree/main/src/alica_ros_flatland_turtle) package and remove CATKIN_IGNORE file.

This package contains draft of the setup - you need to make it complete.

## Guidance

To complete it let's understand how alica in assignment 3.0 works with turtlesim.

Please keep alica designer open and navigate through it when some instance of turtlesim state machine is mentioned.

### Playing with Flatland

Try out running flatland simulation using following command after building flatland:
1. Build `flatland_models`
2. Execute:
```bash
roslaunch flatland_models sim.launch
```

Spawn a model using `Spawn model` button. Use turtlebot model in flatland_models directory.

Try playing with `rostopic` and `rosservice`. See what you can do in simulation using them alone.

### Moving turtle to it's place in a formation

Let's look inside `Simulation` plan.

Inside this plan we can see first state with plan `TeleportToRandomPosition`.

As name suggests this plan will move turtle into random position.

After turtle was teleported `MakeFormation` plan is executed.

Let's look inside this plan. You can see 2 states: one is for going to the center, the other one is for going to the position on the circle. Both of them use the behaviour `GoToCalculatedResult`.

To see the implementation of this behaviour refer to [the code](https://github.com/rapyuta-robotics/alica/blob/2ea37c705ee3ba43575c5b7636c41a19fed74af5/supplementary/alica_turtlesim/libalica-turtlesim/src/GoToCalculatedResult.cpp#L24).

You can see that behaviour commands turtle with following method: `_turtle->moveTowardPosition`. Looking up [what is _turtle](https://github.com/rapyuta-robotics/alica/blob/2ea37c705ee3ba43575c5b7636c41a19fed74af5/supplementary/alica_turtlesim/libalica-turtlesim/include/GoToCalculatedResult.h#L23), we find out that the type is [`turtlesim::TurtleInterfaces`](https://github.com/rapyuta-robotics/alica/blob/2ea37c705ee3ba43575c5b7636c41a19fed74af5/supplementary/alica_turtlesim/include/alica_turtlesim/turtle_interfaces.hpp#L20), which is an interface that allows usage of different implementations of turtle using basic polymorphism.

So to be able to use existing functionality in Flatlad, we need to simply implement this interface. For that modify [the draft file](https://github.com/Gamezar/ros-training/blob/main/src/alica_ros_flatland_turtle/src/turtle_ros1_interfaces.cpp).

After implementing it you will be still getting compilation errors. Fix them.

Find out how to launch simulation and turtles using `roslaunch` (without using `flatland_models`'s launch file) and observe turtles spawning and at least attempting to form a circle.
