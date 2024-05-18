#pragma once

#include <nav_msgs/Odometry.h>
#include <alica_turtlesim/turtle_interfaces.hpp>
#include <ros/ros.h>
#include <string>

namespace alica
{
class Blackboard;
}

namespace turtle_flatland
{

class TurtleRos1Interfaces : public turtlesim::TurtleInterfaces
{
public:
    TurtleRos1Interfaces(const std::string& name);
    bool teleport(const float x, const float y) override;                 // teleport turtle to (x,y)
    bool spawn() override;                                                // Spawn the turtle in the middle of the map
    bool moveTowardPosition(const float x, const float y) const override; // publish cmd_vel based on input(x,y) and current pose
    void rotate(const float dYaw) override;                               // publish rotating speed of turtle based on (dYaw)

private:
    void poseSubCallback(const nav_msgs::OdometryConstPtr& msg); // callback of /pose from the robot
    ros::Publisher _velPub;                                      // publish cmd_vel to the robot
    ros::Subscriber _poseSub;                                    // subscribe to the odeometry of the robot
    ros::ServiceClient _teleportClient;                          // client of move service
    ros::ServiceClient _spawnClient;
    nav_msgs::OdometryConstPtr _currentPose; // current position
};

} // namespace turtlesim

