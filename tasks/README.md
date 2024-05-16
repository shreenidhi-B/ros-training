# Assignment 0. ROS tutorials

Before starting assignments study [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials). Create a pull request with all tutorials walked through.

# Assignment 1. Turtle sim movement

Learn how to move turtle in turtlesim using ROS.

Explore tutorials listed below. Write your own node but in C++ which will run turtlesim and move turtle on different commands:
1. [Moving in straight line](http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line)
- Add ros service `/move_straight` with parameters `distance` and `speed`
- Add launch file

3. [Moving to a goal](http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal)
- Add ros service `/move_to` with parameters `x`, `y`, `tolerance`
- Add launch file

# Assignment 2. Autocharging

Explore [battery indicator node](https://github.com/Gamezar/ros-training/tree/main/src/battery_indicator).

You can launch it by using `roslaunch battery_indicator battery_indicator.launch` after building it with catkin.

This node publishes a rostopic to show [battery status](https://github.com/Gamezar/ros-training/blob/main/src/battery_indicator/msg/BatteryStatus.msg). And has a rosservice, for which we can start charging a battery.

In this assignment you will have to implement autocharging node:

1. Monitor battery status, and if it goes below `critical_percent` (define it using ros parameters) - send a service request to start charging.
2. When battery status goes above or becomes equal to `full_battery` (also using ros parameters) - send a service request to stop charging.
3. Create an `ErrorStatus` message that will have `error` of type `bool` and `description` of type `string` as it's fields.
4. Publish on this topic with a rate of 2Hz.
5. Whenever we detect that battery percentage goes below `warning_percentage` (define it using ros parameters) - publish error on topic `/error_status` with following descritpion: `Robot is about to deplete it's battery, don't assign new job`. 
6. When battery percentage goes above `warning_percentage` - clear error from the topic.
7. Create launch file that will also run battery_indicator node. Create a README with instructions on how to run your node
