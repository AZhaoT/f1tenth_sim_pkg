#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_sim/msg/pid_input.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node control
class PidAngular : public rclcpp::Node
{
public:
  PidAngular()
      : Node("control")
  {
    // subscriber subscibes topic LaserScan
    subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PidAngular::scan_callback, this, _1));
    // subscriber subscibes topic piderror
    subscription_pid = this->create_subscription<f1tenth_sim::msg::PidInput>(
        "piderror", 10, std::bind(&PidAngular::piderror_sub_callback, this, _1));
    // publisher publishes on ackermanndrivestamped
    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    // publisher publishes on topic scan_range
    // publisher_dr = this->create_publisher<f1tenth_sim::msg::DriveParam>("drive_param", 10);
  }

private:
  double prev_time{};
  double curr_time{};
  double dt{};
  double kp{0.4};
  double ki{0.0};
  double kd{0.01};
  double err_scale{1};
  double integral_err{};
  //  double kp{3}; double ki{0.0}; double kd{0.44};

  // return a double instead of callback???
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    prev_time = curr_time;
    curr_time = scan_msg->header.stamp.nanosec;
    dt = (curr_time - prev_time) / 1e9d;
  }

  void piderror_sub_callback(const f1tenth_sim::msg::PidInput::SharedPtr pid_msg)
  {
    double diff_err = (pid_msg->pid_curr_error - pid_msg->pid_prev_error) / dt;
    integral_err += pid_msg->pid_curr_error * dt;
    double steering_angle = err_scale * (kp * pid_msg->pid_curr_error + kd * diff_err + ki * integral_err);
    double vel = 1.5; // (abs(steering_angle) + 100*abs(diff_err)); //drive faster if small error
    //last worked: 1

    // 
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = vel;
    // publish
    publisher_drive->publish(drive_msg);
    // Publisher print:
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "steering angle  is:" << drive_msg.drive.steering_angle);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "drive velocity  is:" << drive_msg.drive.speed);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laserscan;
  rclcpp::Subscription<f1tenth_sim::msg::PidInput>::SharedPtr subscription_pid;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
};

int main(int argc, char *argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidAngular>());
  // std::cout << "enter kp, ki, kd, err: ";
  // std::cin >> PidAngular.control.kp >> PidAngular.control.ki >> PidAngular.control.kd >> PidAngular.control.err_scale;
  rclcpp::shutdown();
  return 0;
}
