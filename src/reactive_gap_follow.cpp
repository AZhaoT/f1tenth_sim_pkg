#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_sim/msg/gap_dir.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node dist_finder, computes distance to the right wall
class GapFollow : public rclcpp::Node
{
public:
  GapFollow()
      : Node("gap_finder")
  {
    // subscriber subscibes topic LaserScan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&GapFollow::scan_sub_callback, this, _1));
    // publisher publishes on topic scan_range
    publisher_gapfollow = this->create_publisher<f1tenth_sim::msg::GapDir>("gapfollow", 10);
  }

private:
  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // laserscan range[] size is 1080
    // scan the front 180 degress and find the largest gap
    // const int scan_size = 1080;
    const double angle1 = (-90) * PI / 180; // 90 deg to the right
    const double angle2 = (90) * PI / 180;  // 90 deg to the left
    const int dist_angle1_index = (angle1 - scan_msg->angle_min) / scan_msg->angle_increment;
    const int dist_angle2_index = (angle2 - scan_msg->angle_min) / scan_msg->angle_increment;
    const int bubble_size{80};
    // int gaplen{};
    // int largest_gaplen{};
    // int largest_gap_end{};
    int furthest_point_index = dist_angle1_index;
    // find the largeset gap from -90 deg to 90 deg

    for (int i = dist_angle1_index; i < dist_angle2_index; i++)
    {
      //check for furthest point
      

      if ((scan_msg->ranges[i] - scan_msg->ranges[i-1]) > 0.3) // disparity: obstacle to the right of the scan
      {
        for (int j = 0; j < bubble_size; j++)
          scan_msg->ranges[i+j] = scan_msg->ranges[i-1]; // overwrite some elements after the disparity

        i += (bubble_size);  // jump forward to the next unoverwritten element
      }
      else if((scan_msg->ranges[i] - scan_msg->ranges[i+1]) > 0.3) // disparity obstacle to the left of the scan
      {
        for (int j = 0; j < bubble_size; j++)
          scan_msg->ranges[i-j] = scan_msg->ranges[i+1]; // overwrite some elements before the disparity

        i -= (bubble_size);                              // jump backward to the earliest overwriten element
        furthest_point_index -= bubble_size;
      }
      else{
        //no disparity
        if (scan_msg->ranges[furthest_point_index] < scan_msg->ranges[i])
          {
            furthest_point_index = i;
          }
        
      }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furtherst distance is: "
                      << scan_msg->ranges[furthest_point_index]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furthest direction is: "
                      << (scan_msg->angle_min + furthest_point_index * scan_msg->angle_increment)* 180/PI);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Largest gap angle left: "
    //                    << (scan_msg->angle_min + largest_gap_end * scan_msg->angle_increment) * 180/PI);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Largest gap angle right: "
    //                    << (scan_msg->angle_min + (largest_gap_end - largest_gaplen) * scan_msg->angle_increment) * 180/PI);

    auto gap_dir_msg = f1tenth_sim::msg::GapDir();
    gap_dir_msg.gap_direction = (scan_msg->angle_min + furthest_point_index * scan_msg->angle_increment);
    // publish
    publisher_gapfollow->publish(gap_dir_msg);
    // Publisher print:
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furthest direction is: "
    //                << (scan_msg->angle_min + furthest_point_index * scan_msg->angle_increment)* 180/PI);
    
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<f1tenth_sim::msg::GapDir>::SharedPtr publisher_gapfollow;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapFollow>());
  rclcpp::shutdown();
  return 0;
}
