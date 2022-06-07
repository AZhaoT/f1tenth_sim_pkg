#include <memory>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;




class LaserScanMaxMin : public rclcpp::Node
{
  public:
  LaserScanMaxMin()
    : Node("laserscan_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&LaserScanMaxMin::topic_callback, this, _1));
      publisher_ = this->create_publisher<ssensor_msgs::msg::LaserScan>(
      "farthest point", 10);

    }

  private:
    struct maxmin{
      double max{};
      double min{};
    }; 

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
      
      const int arr_size = 1081;
      const int range_max = 30;
      const int range_min = 0;
      maxmin max_min{};

      // Loop to store largest number to max_min.max, smallest number to max_min.min
      for (int i = 0; i < arr_size; ++i)
      {
        if (max_min.max < msg->ranges[i] && msg->ranges[i] <= range_max)
          max_min.max = msg->ranges[i];

        if (max_min.min > msg->ranges[i] && msg->ranges[i] >= range_min)
          max_min.min = msg->ranges[i];
      }
      //RCLCPP_INFO_STREAM("LaserScan (val,angle)=(%f,%f", msg->scan_time);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "range max is: " <<  max_min.max);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "range min is: " <<  max_min.min);
              
      publisher_->publish(message);

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanMaxMin>());
  rclcpp::shutdown();
  return 0;
}
