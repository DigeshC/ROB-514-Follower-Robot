#include <memory>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

class ArucoFollower : public rclcpp::Node
{
public:
  ArucoFollower()
  : Node("aruco_follower")
  {
    image_topic_      = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    marker_size_      = this->declare_parameter<double>("marker_size", 0.05);   // meters
    desired_distance_ = this->declare_parameter<double>("desired_distance", 0.4);
    k_linear_         = this->declare_parameter<double>("k_linear", 0.5);
    k_angular_        = this->declare_parameter<double>("k_angular", 0.8);

    // TODO: load real camera intrinsics via params or camera_info
    camera_matrix_ = cv::Mat();
    dist_coeffs_   = cv::Mat();

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ArucoFollower::imageCallback, this, _1)
    );

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),   // 20 Hz
      std::bind(&ArucoFollower::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "aruco_follower_cpp node started");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);

    has_marker_ = false;

    if (!ids.empty()) {
      has_marker_ = true;

      // use first marker
      const auto & marker_corners = corners[0];
      cv::Point2f center(0.f, 0.f);
      for (const auto & p : marker_corners) {
        center += p;
      }
      center *= 0.25f;

      last_center_x_ = center.x;
      last_image_width_ = frame.cols;

      if (!camera_matrix_.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
        last_distance_ = static_cast<double>(tvecs[0][2]);
      } else {
        last_distance_ = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!has_marker_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;  // or a slow search turn
      cmd_pub_->publish(cmd);
      return;
    }

    double cx = static_cast<double>(last_center_x_);
    double width = static_cast<double>(last_image_width_);
    double error_x = (cx - width / 2.0) / (width / 2.0);  // [-1, 1]

    cmd.angular.z = -k_angular_ * error_x;  // flip sign if wrong direction

    if (!std::isnan(last_distance_)) {
      double error_d = last_distance_ - desired_distance_;
      cmd.linear.x = k_linear_ * error_d;
    } else {
      cmd.linear.x = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  // parameters
  std::string image_topic_;
  double marker_size_;
  double desired_distance_;
  double k_linear_;
  double k_angular_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // camera intrinsics (optional; load from params or camera_info)
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // state
  bool   has_marker_{false};
  float  last_center_x_{0.f};
  int    last_image_width_{1};
  double last_distance_{std::numeric_limits<double>::quiet_NaN()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoFollower>());
  rclcpp::shutdown();
  return 0;
}
