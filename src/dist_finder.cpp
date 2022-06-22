#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_sim/msg/pid_input.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node dist_finder, computes distance to the right wall
class WallDistance : public rclcpp::Node
{
public:
  WallDistance()
      : Node("dist_finder")
  {
    // subscriber subscibes topic LaserScan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallDistance::scan_sub_callback, this, _1));
    // publisher publishes on topic scan_range
    publisher_ = this->create_publisher<f1tenth_sim::msg::PidInput>("piderror", 10);
  }

private:
  double prev_err{};
  double err{};

  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
  {
    //const int arr_size = 1080;
    double angle1 = (90)*PI/180;
    double angle2 = (40)*PI/180;
    int dist_angle1_index = (angle1 - scan_msg->angle_min) / scan_msg->angle_increment;
    int dist_angle2_index = (angle2 - scan_msg->angle_min) / scan_msg->angle_increment;
  
    while(std::isinf(scan_msg->ranges[dist_angle1_index]) || std::isnan(scan_msg->ranges[dist_angle1_index]))
    {
      dist_angle1_index++;
      angle1 += scan_msg->angle_increment;
    }
    while(std::isinf(scan_msg->ranges[dist_angle2_index]) || std::isnan(scan_msg->ranges[dist_angle2_index]))
    {
      dist_angle2_index++;
      angle2 += scan_msg->angle_increment;
    }
    
    const double dist_angle1 = scan_msg->ranges[dist_angle1_index];
    const double dist_angle2 = scan_msg->ranges[dist_angle2_index];
    double theta = angle1 - angle2; //angel between to scan angles
    double dev_angle_param = (dist_angle2 * cos(theta) - dist_angle1)/ (dist_angle2 * sin(theta));
    double dev_angle = atan(dev_angle_param); //deviated angel from straight
    double dist_to_left_wall = dist_angle1 * cos(dev_angle);
    const double disp_btw_scans{};//velocity * time btw scans
    double dist_to_left_wall_predicted = disp_btw_scans * sin(dev_angle) + dist_to_left_wall;
    const double middle_to_left {1.0};
    //const double desired_dist{0.7};

    //update previous error
    prev_err = err;
    //update current error
    err = dist_to_left_wall_predicted - middle_to_left;
    double const vel = 0.5;
    // publish PidInput msg to the topic /piderror
    auto pid_input_message = f1tenth_sim::msg::PidInput();
    pid_input_message.pid_prev_error = prev_err;
    pid_input_message.pid_curr_error = err;
    pid_input_message.pid_vel = vel;//need modify
    // publish
    publisher_->publish(pid_input_message);
    // Publisher print:
    
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "prev_error is: " << pid_input_message.pid_prev_error);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "error is: " << pid_input_message.pid_curr_error);
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "velocity is:   " << pid_input_message.pid_vel);

  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<f1tenth_sim::msg::PidInput>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDistance>());
  rclcpp::shutdown();
  return 0;
}
