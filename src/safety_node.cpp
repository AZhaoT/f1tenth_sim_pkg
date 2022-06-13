// Automatic emergency brakig with iTTC (immediate Time To Collision)

#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;
/**
 * @brief
 *  One publisher should publish to the /brake topic with an
 *  ackermann_msgs/AckermannDriveStamped brake message.
 *
 *  One publisher should publish to the /brake_bool topic with a
 *  std_msgs/Bool message.
 *
 *  You should also subscribe to the /scan topic to get the
 *  sensor_msgs/LaserScan messages and the /odom topic to get
 *  the nav_msgs/Odometry messages
 *
 *  The subscribers should use the provided odom_callback and
 *  scan_callback as callback methods
 *
 *  NOTE that the x component of the linear velocity in odom is the speed
 *
 */

/**
 * @brief The class that handles emergency braking
 *
 */

class SafetyNode : public rclcpp::Node
{
public:
    // create node auto_brake
    SafetyNode()
        : Node("auto_brake")
    {
        
        // subscriber for odom /ego_racecar/odom
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 10, std::bind(&SafetyNode::odom_callback, this, _1));
        // subscriber subscribes topic /scan
        subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&SafetyNode::scan_callback, this, _1));

        // publisher publishes topic brake_bool
        publisher_bool = this->create_publisher<std_msgs::msg::Bool>("brake_bool", 10);
        // publisher publishes topic /drive
        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    }
    
    double odom_vel;   
    // obtains forward velocity

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        odom_vel = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const
    {
        const int arr_size = 1080;
        const double ttcmin = 1.5; // minimum ttc needed to break in front of an obstacle, change as needed
        double laser_angle{};
        double projected_vel{};
        double ittc[arr_size] = {};
        double ittc_min{100}; // minimum in ittc[]
        // Loop to store scan_msg->range in the range array

        for (int i = 0; i < arr_size; ++i)
        {
            if (scan_msg->ranges[i] > scan_msg->range_max || scan_msg->ranges[i] < scan_msg->range_min)
                continue;

            laser_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // odom_vel * cos(laser_angle) is the forward/backward velocity (+/-) projected onto the laserscan beam
            projected_vel = odom_vel * cos(laser_angle); // project_vel > 0 means vehicle is approaching an obstacle
            ittc[i] = (projected_vel > 0) ? (scan_msg->ranges[i] / projected_vel)
                                          : (std::numeric_limits<double>::infinity());
            if (ittc_min > ittc[i])
                ittc_min = ittc[i];
        }

        if (ittc_min < ttcmin) // if minimum is below threshold. Maybe: k * odom_vel/deceleration
        {
            // publish brake_bool
            auto brake_bool_msg = std_msgs::msg::Bool();
            brake_bool_msg.data = true;
            // publish
            publisher_bool->publish(brake_bool_msg);
            // Publisher print:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "TTC is:" << ittc_min);

            auto drive_brake_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_brake_msg.drive.speed = 0;
            drive_brake_msg.drive.acceleration = 0;
            drive_brake_msg.drive.jerk = 0;
            // publish
            publisher_drive->publish(drive_brake_msg);
            // publisher print:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Braking...");
        }
    }


private:
    double deceleration{8.26}; // f1-10th approx deceleration

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laserscan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
