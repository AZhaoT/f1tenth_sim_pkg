#include <memory>
#include <iostream>
#include <limits>
#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "f1tenth_sim/msg/transformation.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// create node scan_match_transform
class Transform : public rclcpp::Node
{
public:
  Transform()
      : Node("scan_match_transform")
  {
    // subscriber subscibes topic LaserScan
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Transform::scan_sub_callback, this, _1));
    // publisher publishes on topic scan_range
    //publisher_ = this->create_publisher<f1tenth_sim::msg::Transformation>("transform", 10);
  }

private:

  void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    const int arr_size = 1080;
    const int dim = 2;
    int scan_points_prev_frame[3][1080];
    int scan_points_curr_frame[3][1080];
    double transformation_matrix[3][3]{};
    transformation_matrix[2][0] = 0;
    transformation_matrix[2][1] = 0;
    transformation_matrix[2][2] = 1;
    //double prev_pos[3][1]{};
    double mult[3][1080]{};

    // 1080 sets of 3x1 scan coordinate:
    // x
    // y
    // 1
    for (int i = 0; i < 3; i++)
    {
      for(int j = 0; j < arr_size; j++){
            scan_points_curr_frame[0][j] = scan_msg->ranges[j] * cos(scan_msg->angle_min + j*scan_msg->angle_increment); // x coords
            scan_points_curr_frame[1][j] = scan_msg->ranges[j] * sin(scan_msg->angle_min + j*scan_msg->angle_increment); // y coords
            scan_points_curr_frame[2][j] = 1;
      }
    }
    // std::cout << "coords";


    // make transformation_matrix guesses
    const int iterations{3};
    double theta{};
    double diff_matrix[3][1080];
    double diff_length{};
    double min_diff_length{100000000.0};
    double min_rot{};
    double min_tx{};
    double min_ty{};

    //find 
    for (int i = 0; i < iterations; i++)
    {
      theta = i * 0.01;
      // rotation matrix
      set_rot_matrix(&transformation_matrix, theta);
      //std::cout << "rot matrix"<< "\n";

      for (int j = 0; j < iterations; j++)
      {
        // translation matrix???
        transformation_matrix[0][2] = 0.05 * j;
        transformation_matrix[1][2] = 0.05 * j;
        // multiply translation matrix with prev_pos
        for (int k = 0; k < 3; ++k)
          for (int l = 0; l < arr_size; ++l)
          {
            for (int m = 0; m < (dim + 1); ++m)
            {
              mult[i][j] += transformation_matrix[i][k] * scan_points_curr_frame[k][j];
            }
            diff_matrix[i][j] = mult[i][j] - scan_points_prev_frame[i][j];
          }
          // std::cout << "diff matrix" << '\n';
        // find matrix length
        diff_length = get_matrix_length(&diff_matrix);

        // compare matrices
        if (diff_length < min_diff_length)
        {
          min_diff_length = diff_length;
          // pointer set min_diff_transformation_matrix = transformation_matrix
          min_rot = theta;
          min_tx = transformation_matrix[0][2];
          min_ty = transformation_matrix[1][2];
        }
        // std::cout << "compare\n";
      }
    }

    // scan_points_prev_frame = scan_points_curr_frame;
    // publish ScanRange msg to the topic /scan_range
    // auto msg = f1tenth_sim::msg::Transformation();
    // msg.rotation_angle = min_theta;
    // msg.translation_x = min_diff_transformation_matrix[0][2];
    // msg.translation_y = min_diff_transformation_matrix[1][2];
    // publish
    // publisher_->publish(msg);
    // Publisher print:
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "rotation angle is: " << min_rot);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "rotation angle is: " << min_tx);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "rotation angle is: " << min_ty);
  }

  void set_rot_matrix(double (*transf_mtx)[3][3], double rot_angle)
  {
    (*transf_mtx)[0][0] = cos(rot_angle);
    (*transf_mtx)[0][1] = -sin(rot_angle);
    (*transf_mtx)[1][0] = sin(rot_angle);
    (*transf_mtx)[1][1] = cos(rot_angle);
  }

  double get_matrix_length(double (*diff_mtx)[3][1080])
  {
    double length_sq{};
    double length{};
    for (int j = 0; j < 1080; j++)
    {
      for (int i = 0; i < 3; i++)
      {
        length_sq += (*diff_mtx)[i][j] * (*diff_mtx)[i][j];
      }
      length += sqrt(length_sq);
      length_sq = 0;
    }

    return length;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<f1tenth_sim::msg::Transformation>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transform>());
  rclcpp::shutdown();
  return 0;
}
