#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>
#include <iostream>

class TurtleBot
{
public:
    TurtleBot() : rate(10) // Correct initialization of ros::Rate with 10 Hz frequency
    {
        velocity_publisher = node_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        pose_subscriber = node_handle.subscribe("/turtle1/pose", 10, &TurtleBot::updatePose, this);
    }

    void move2goal()
    {
        turtlesim::Pose goal_pose;
        float distance_tolerance;

        std::cout << "Set your x goal: ";
        std::cin >> goal_pose.x;
        std::cout << "Set your y goal: ";
        std::cin >> goal_pose.y;
        std::cout << "Set your tolerance: ";
        std::cin >> distance_tolerance;

        geometry_msgs::Twist vel_msg;

        do
        {
            vel_msg.linear.x = linearVel(goal_pose);
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;
            vel_msg.angular.z = angularVel(goal_pose);

            velocity_publisher.publish(vel_msg);
            rate.sleep();
        } while (euclideanDistance(goal_pose) >= distance_tolerance);

        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
    }

private:
    ros::NodeHandle node_handle;
    ros::Publisher velocity_publisher;
    ros::Subscriber pose_subscriber;
    turtlesim::Pose pose;
    ros::Rate rate; // Use ros::Rate directly

    void updatePose(const turtlesim::Pose::ConstPtr &msg)
    {
        pose = *msg;
        pose.x = std::round(pose.x * 10000.0) / 10000.0;
        pose.y = std::round(pose.y * 10000.0) / 10000.0;
    }

    float euclideanDistance(const turtlesim::Pose &goal_pose)
    {
        return std::sqrt(std::pow((goal_pose.x - pose.x), 2) + std::pow((goal_pose.y - pose.y), 2));
    }

    float linearVel(const turtlesim::Pose &goal_pose, float constant = 1.5)
    {
        return constant * euclideanDistance(goal_pose);
    }

    float steeringAngle(const turtlesim::Pose &goal_pose)
    {
        return std::atan2(goal_pose.y - pose.y, goal_pose.x - pose.x);
    }

    float angularVel(const turtlesim::Pose &goal_pose, float constant = 6)
    {
        return constant * (steeringAngle(goal_pose) - pose.theta);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_controller");
    TurtleBot turtlebot;
    turtlebot.move2goal();
    ros::spin();
    return 0;
}