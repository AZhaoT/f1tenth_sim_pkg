#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_sim/msg/opp_gap_dir.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node dist_finder, computes distance to the right wall
class OppGapFollow : public rclcpp::Node
{
public:
  OppGapFollow()
      : Node("opp_gap_finder")
  {
    // subscriber subscibes topic LaserScan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "opp_scan", 10, std::bind(&OppGapFollow::opp_scan_sub_callback, this, _1));
    // publisher publishes on topic scan_range
    publisher_opp_gapfollow = this->create_publisher<f1tenth_sim::msg::OppGapDir>("opp_gapfollow", 10);
  }

private:
  void opp_scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr opp_scan_msg)
  {
    // laserscan range[] size is 1080
    // scan the front 180 degress and find the largest gap
    // const int scan_size = 1080;
    const double angle1 = (-90) * PI / 180; // 90 deg to the right
    const double angle2 = (90) * PI / 180;  // 90 deg to the left
    const int dist_angle1_index = (angle1 - opp_scan_msg->angle_min) / opp_scan_msg->angle_increment;
    const int dist_angle2_index = (angle2 - opp_scan_msg->angle_min) / opp_scan_msg->angle_increment;
    const int bubble_size{80};
    // int gaplen{};
    // int largest_gaplen{};
    // int largest_gap_end{};
    int furthest_point_index = dist_angle1_index;
    // find the largeset gap from -90 deg to 90 deg

    for (int i = dist_angle1_index; i < dist_angle2_index; i++)
    {
      //check for furthest point
      

      if ((opp_scan_msg->ranges[i] - opp_scan_msg->ranges[i-1]) > 0.3) // disparity: obstacle to the right of the scan
      {
        for (int j = 0; j < bubble_size; j++)
          opp_scan_msg->ranges[i+j] = opp_scan_msg->ranges[i-1]; // overwrite some elements after the disparity

        i += (bubble_size);  // jump forward to the next unoverwritten element
      }
      else if((opp_scan_msg->ranges[i] - opp_scan_msg->ranges[i+1]) > 0.3) // disparity obstacle to the left of the scan
      {
        for (int j = 0; j < bubble_size; j++)
          opp_scan_msg->ranges[i-j] = opp_scan_msg->ranges[i+1]; // overwrite some elements before the disparity

        i -= (bubble_size);                              // jump backward to the earliest overwriten element
        furthest_point_index -= bubble_size;
      }
      else{
        //no disparity
        if (opp_scan_msg->ranges[furthest_point_index] < opp_scan_msg->ranges[i])
          {
            furthest_point_index = i;
          }
        
      }
    }

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furtherst distance is: "
    //                   << opp_scan_msg->ranges[furthest_point_index]);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furthest direction is: "
    //                   << (opp_scan_msg->angle_min + furthest_point_index * opp_scan_msg->angle_increment)* 180/PI);

    auto opp_gap_dir_msg = f1tenth_sim::msg::OppGapDir();
    opp_gap_dir_msg.opp_gap_direction = (opp_scan_msg->angle_min + furthest_point_index * opp_scan_msg->angle_increment);
    // publish
    publisher_opp_gapfollow->publish(opp_gap_dir_msg);
    // Publisher print:
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Furthest direction is: "
    //                << (opp_scan_msg->angle_min + furthest_point_index * opp_scan_msg->angle_increment)* 180/PI);
    
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<f1tenth_sim::msg::OppGapDir>::SharedPtr publisher_opp_gapfollow;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OppGapFollow>());
  rclcpp::shutdown();
  return 0;
}
