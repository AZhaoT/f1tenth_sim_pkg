#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "csv.h"
#include "math.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define PI 3.14159265

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node dist_finder, computes distance to the right wall
class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit()
      : Node("pure_pursuit")
  {
    // subscriber subscibes topic LaserScan
    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom", 10, std::bind(&PurePursuit::odom_sub_callback, this, _1));
    // subscription_tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
    //     "tf", 10, std::bind(&PurePursuit::tf_sub_callback, this, _1));
    
    // publisher publishes on topic scan_range
    publisher_rviz = this->create_publisher<visualization_msgs::msg::Marker>("rviz_m", 10);
    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    // read in all the data
    io::CSVReader<3> in("/sim_ws/src/convoy_ros/src/data.csv");
    double x;
    double y;
    double heading;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    geometry_msgs::msg::Point points;
    while (in.read_row(x, y, heading))
    {
        waypoint_x.push_back(x);
        waypoint_y.push_back(y);
        headings.push_back(heading);
        points.x = x;
        points.y = y;
        points.z = 0.0;
        marker.points.push_back(points);
        // markerArray.markers.push_back(marker);
    }
  }

private:
    visualization_msgs::msg::Marker marker;
    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    std::vector<double> headings;

    double x_pos{};
    double y_pos{};

    double dist(double x0, double y0, double x1, double y1)
    {
      return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
    }

    // void tf_sub_callback(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg){
    //   tf_msg->transforms->child_fram_id 
    // }

    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        double x_curr = odom_msg->pose.pose.position.x;
        double y_curr = odom_msg->pose.pose.position.y;
        double closest_dist{100};
        double curr_index{};
        //find the closest waypoint in data.csv
        for(int i=0; i < waypoint_x.size(); i++){
          if(dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr) < closest_dist ){
            curr_index = i;
            closest_dist = dist(waypoint_x[i], waypoint_y[i], x_curr, y_curr);
          }
        }

        const double look_ahead_dist{1.0};
        bool found = false;
        int goal_index = curr_index;

        //find the closest goal point right outside the look ahead distance
        while(!found){
          //if goal index reaches the last waypoint, then jump to the first waypoint
          if (goal_index == static_cast<int>(waypoint_x.size()) - 1){
            goal_index = 0;
          }
          //go to the next point if the current goal point is still inside the look ahead distance
          if(dist(waypoint_x[goal_index], waypoint_y[goal_index], x_curr, y_curr) < look_ahead_dist){
            goal_index++;
          }else{
            found = true;
          }
        }
        double goal_dist = dist(waypoint_x[goal_index], waypoint_y[goal_index], x_curr, y_curr);
        std::cout << "my point: " << x_curr << ", " << y_curr << '\n';
        std::cout << "goal point: " << waypoint_x[goal_index] << ", " << waypoint_y[goal_index] << '\n';
        std::cout << "goal point index: " << goal_index << '\n';



        //need heading angle in map frame
        double yaw =  atan2(2.0 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.z + odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.y) , 
                            - 1.0 + 2.0 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.w + odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.x));

        std::cout << "my yaw: " << yaw << '\n';
        double look_ahead_angle = atan2(waypoint_y[goal_index] - y_curr, waypoint_x[goal_index] - x_curr) - yaw;
        std::cout << "my tan is: " << atan2(waypoint_y[goal_index] - y_curr, waypoint_x[goal_index] - x_curr)/3.14*180 << '\n';

        
        if(look_ahead_angle > M_PI){
          look_ahead_angle -= 2*M_PI;
        }else if(look_ahead_angle < (-1.0)*M_PI){
          look_ahead_angle += 2*M_PI;
        }

        std::cout << "my lookahead angle is: " << look_ahead_angle/3.14*180 << '\n';
        double delta_y = goal_dist * sin(look_ahead_angle);


        //calculate turn curvature
        double curvature = 2 * delta_y / (goal_dist * goal_dist);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "curvature is: " << curvature);
        
        auto rviz_msg = visualization_msgs::msg::Marker();
        rviz_msg = marker;
        

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = curvature / 2;
        drive_msg.drive.steering_angle_velocity = 0.3;

        if (drive_msg.drive.steering_angle > 0.6)
          drive_msg.drive.speed = 0.5 / drive_msg.drive.steering_angle;
        else
          drive_msg.drive.speed = std::min(8.0, abs(1.0 / drive_msg.drive.steering_angle)); // max=8 for speilberg
        // 0.66 max steer

        // decelarate before corners by examining distance to furthest point

        // publish
        publisher_drive->publish(drive_msg);
        publisher_rviz->publish(rviz_msg);

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_rviz;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;

    
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
