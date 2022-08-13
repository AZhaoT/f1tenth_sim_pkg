#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_sim/msg/gap_dir.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node control
class GapDrive : public rclcpp::Node
{
public:
  GapDrive()
      : Node("drive_to_gap")
  {

    // subscriber subscibes topic gapfollow
    subscription_gapfollow= this->create_subscription<f1tenth_sim::msg::GapDir>(
        "gapfollow", 10, std::bind(&GapDrive::gapfollow_sub_callback, this, _1));
    // publisher publishes on ackermanndrivestamped
    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
  }

private:
  void gapfollow_sub_callback(const f1tenth_sim::msg::GapDir::SharedPtr gapdir_msg)
  {
    // publish drive msg
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = gapdir_msg->gap_direction/1.5;
    drive_msg.drive.steering_angle_velocity = 0.3;

    if(drive_msg.drive.steering_angle > 0.6)
      drive_msg.drive.speed = 0.5 / drive_msg.drive.steering_angle;
    else
      drive_msg.drive.speed = std::min(8.0, abs(1.0 / drive_msg.drive.steering_angle)); //max=8 for speilberg
    //0.66 max steer

    //decelarate before corners by examining distance to furthest point

    // publish
    publisher_drive->publish(drive_msg);
    // Publisher print:
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "steering angle  is:" << drive_msg.drive.steering_angle);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "drive velocity  is:" << drive_msg.drive.speed);
  }

  rclcpp::Subscription<f1tenth_sim::msg::GapDir>::SharedPtr subscription_gapfollow;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
};

int main(int argc, char *argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapDrive>());
  // std::cout << "enter kp, ki, kd, err: ";
  // std::cin >> GapDrive.control.kp >> GapDrive.control.ki >> GapDrive.control.kd >> GapDrive.control.err_scale;
  rclcpp::shutdown();
  return 0;
}
