#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_sim/msg/opp_gap_dir.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node control
class OppGapDrive : public rclcpp::Node
{
public:
  OppGapDrive()
      : Node("opp_drive_to_gap")
  {

    // subscriber subscibes topic opp_gapfollow
    subscription_opp_gapfollow= this->create_subscription<f1tenth_sim::msg::OppGapDir>(
        "opp_gapfollow", 10, std::bind(&OppGapDrive::opp_gapfollow_sub_callback, this, _1));
    // publisher publishes on ackermanndrivestamped
    publisher_opp_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("opp_drive", 10);
  }

private:
  void opp_gapfollow_sub_callback(const f1tenth_sim::msg::OppGapDir::SharedPtr opp_gapdir_msg)
  {
    // publish opp_drive msg
    auto opp_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    opp_drive_msg.drive.steering_angle = opp_gapdir_msg->opp_gap_direction/1.5;
    opp_drive_msg.drive.steering_angle_velocity = 0.3;

    if(opp_drive_msg.drive.steering_angle > 0.6)
      opp_drive_msg.drive.speed = 0.5 / opp_drive_msg.drive.steering_angle;
    else
      opp_drive_msg.drive.speed = std::min(8.0, abs(1.0 / opp_drive_msg.drive.steering_angle)); //max=8 for speilberg
    //0.66 max steer

    //decelarate before corners by examining distance to furthest point

    // publish
    publisher_opp_drive->publish(opp_drive_msg);
    // Publisher print:
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "steering angle  is:" << opp_drive_msg.opp_drive.steering_angle);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "drive velocity  is:" << opp_drive_msg.opp_drive.speed);
  }

  rclcpp::Subscription<f1tenth_sim::msg::OppGapDir>::SharedPtr subscription_opp_gapfollow;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_opp_drive;
};

int main(int argc, char *argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OppGapDrive>());
  // std::cout << "enter kp, ki, kd, err: ";
  // std::cin >> GapDrive.control.kp >> GapDrive.control.ki >> GapDrive.control.kd >> GapDrive.control.err_scale;
  rclcpp::shutdown();
  return 0;
}
