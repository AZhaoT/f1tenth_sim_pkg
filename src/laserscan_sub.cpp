#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_sim/msg/scan_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node laserscan_subscriber
class LaserScanMaxMin : public rclcpp::Node
{
public:
  LaserScanMaxMin()
      : Node("laserscan_subscriber")
  {
    // subscriber subscibes topic LaserScan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LaserScanMaxMin::scan_sub_callback, this, _1));
    // publisher publishes on topic scan_range
    publisher_ = this->create_publisher<f1tenth_sim::msg::ScanRange>("scan_range", 10);
  }

private:
  struct maxmin
  {
    double max{};
    double min{};
  };

  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    const int arr_size = 1080;
    const int range_max = 30;
    const int range_min = 0;
    maxmin max_min{}; // change
    max_min.min = range_max;

    // Loop to store largest number to max_min.max, smallest number to max_min.min
    for (int i = 0; i < arr_size; ++i)
    {
      if (max_min.max < msg->ranges[i] && msg->ranges[i] <= range_max) //(!std::isinf(msg->[i]))
        max_min.max = msg->ranges[i];

      if (max_min.min > msg->ranges[i] && msg->ranges[i] > range_min) //(!std::isnan(msg->[i]))
        max_min.min = msg->ranges[i];
    }
    // Subscriber print:
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "range max is: " <<  max_min.max);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "range min is: " <<  max_min.min);

    // publish ScanRange msg to the topic /scan_range
    auto message = f1tenth_sim::msg::ScanRange();
    message.farthest_point = max_min.max;
    message.closest_point = max_min.min;
    // publish
    publisher_->publish(message);
    // Publisher print:
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "farthest point is:" << message.farthest_point);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "closest point is:" << message.closest_point);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<f1tenth_sim::msg::ScanRange>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanMaxMin>());
  rclcpp::shutdown();
  return 0;
}
