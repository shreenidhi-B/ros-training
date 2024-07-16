#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void move(double speed, double distance, bool isForward)
{
    ros::NodeHandle n;
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;

    // Set the speed
    vel_msg.linear.x = isForward ? abs(speed) : -abs(speed);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    // Setting the current time for distance calculation
    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok() && current_distance < distance)
    {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop the robot
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_cleaner");

    double speed, distance;
    bool isForward;

    std::cout << "Let's move your robot" << std::endl;
    std::cout << "Input your speed: ";
    std::cin >> speed;
    std::cout << "Type your distance: ";
    std::cin >> distance;
    std::cout << "Forward? (1 for yes, 0 for no): ";
    std::cin >> isForward;

    move(speed, distance, isForward);

    return 0;
}