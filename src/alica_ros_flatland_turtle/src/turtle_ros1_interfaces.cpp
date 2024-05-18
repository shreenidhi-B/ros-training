#include "alica_ros_flatland/turtle_ros1_interfaces.hpp"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#include <engine/blackboard/Blackboard.h>

#include <flatland_msgs/MoveModel.h>
#include <flatland_msgs/SpawnModel.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

namespace turtle_flatland {

TurtleRos1Interfaces::TurtleRos1Interfaces(const std::string &name)
    : TurtleInterfaces(name) {
  // initialize publisher, subscriber and service client.
  ros::NodeHandle nh("~");
  _velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  _poseSub = nh.subscribe("***", 1, &TurtleRos1Interfaces::poseSubCallback,
                          this); // Find the correct topic name
  _teleportClient = ros::NodeHandle().serviceClient<flatland_msgs::MoveModel>(
      "***"); // Find the correct service name
  _spawnClient = ros::NodeHandle().serviceClient<flatland_msgs::SpawnModel>(
      "***"); // Find the correct service name
  // wait for the service to be available here
}

bool TurtleRos1Interfaces::teleport(float x, float y) {
  flatland_msgs::MoveModel srv;
  // compose the request here
  if (/* call the service here */) {
    ROS_INFO_STREAM("Teleported to (" << x << ", " << y << ")");
    return true;
  } else {
    ROS_ERROR_STREAM("Failed to teleport to (" << x << ", " << y << ")");
    return false;
  }
}

bool TurtleRos1Interfaces::spawn() {
  flatland_msgs::SpawnModel srv;
  srv.request.name;
  srv.request.ns; // You need to specify name and namespace of the model, such
                  // that names and topics will not overlap
  srv.request.yaml_path; // here you should specify location of the file which
                         // has information about the robot model
  // specify the initial pose of the robot
  if (/* call the service here */) {
    return true;
  } else {
    return false;
  }
}

void TurtleRos1Interfaces::poseSubCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  _currentPose = msg;
}

void TurtleRos1Interfaces::rotate(const float dYaw) {
  geometry_msgs::Twist msg;
  msg.angular.z = dYaw;
  _velPub.publish(msg);
}

bool TurtleRos1Interfaces::moveTowardPosition(float x, float y) const {
  if (!_currentPose) {
    ROS_WARN_THROTTLE(5, "Waiting for valid pose");
    // Wait until we have a valid pose
    return false;
  }
  // Transform goal position into coordinates of turtle body frame
  float cosTheta = std::cos(tf::getYaw(_currentPose->pose.pose.orientation));
  float sinTheta = std::sin(tf::getYaw(_currentPose->pose.pose.orientation));
  float dx = x - _currentPose->pose.pose.position.x;
  float dy = y - _currentPose->pose.pose.position.y;

  // Calculate goal distance and return true if reach goal
  constexpr float goalTolerance = 0.01;
  bool isReachGoal = false;
  if (dx * dx + dy * dy <= goalTolerance * goalTolerance) {
    isReachGoal = true;
    return isReachGoal;
  }

  // Transform to turtle body frame
  float txG = dx * cosTheta + dy * sinTheta;
  float tyG = -dx * sinTheta + dy * cosTheta;

  // Simple proposional control
  constexpr float kLin = 2.0;
  constexpr float kAng = 2.0;

  float linearX = kLin * txG;
  float linearY = kLin * tyG;
  float headingError = std::atan2(linearY, linearX);
  float angular = kAng * headingError;

  geometry_msgs::Twist msg;
  msg.linear.x = linearX;
  msg.angular.z = angular;
  _velPub.publish(msg);

  return isReachGoal;
}

} // namespace turtle_flatland
