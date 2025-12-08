#include <memory>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

class ArucoFollower : public rclcpp::Node
{
public:
  ArucoFollower()
  : Node("aruco_follower")
  {
    // Parameters
    image_topic_            = this->declare_parameter<std::string>("image_topic", "/image_raw");
    marker_size_m_          = this->declare_parameter<double>("marker_size", 0.05);   // still useful for info
    desired_distance_m_     = this->declare_parameter<double>("desired_distance", 0.4); // unused now, kept for ref
    k_linear_               = this->declare_parameter<double>("k_linear", 0.5);
    k_angular_              = this->declare_parameter<double>("k_angular", 0.8);
    cmd_frame_id_           = this->declare_parameter<std::string>("cmd_frame_id", "base_link");
    desired_marker_size_px_ = this->declare_parameter<double>("desired_marker_size_px", 80.0);

    RCLCPP_INFO(this->get_logger(), "Starting ArucoFollower with:");
    RCLCPP_INFO(this->get_logger(), "  image_topic            = %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  marker_size (m)        = %.3f", marker_size_m_);
    RCLCPP_INFO(this->get_logger(), "  desired_distance (m)   = %.3f (not used; using pixel size instead)", desired_distance_m_);
    RCLCPP_INFO(this->get_logger(), "  desired_marker_size_px = %.1f px", desired_marker_size_px_);
    RCLCPP_INFO(this->get_logger(), "  k_linear               = %.3f", k_linear_);
    RCLCPP_INFO(this->get_logger(), "  k_angular              = %.3f", k_angular_);
    RCLCPP_INFO(this->get_logger(), "  cmd_frame_id           = %s", cmd_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  gesture topic          = /gesture_command (String: 'follow'/'stop')");

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    // Image subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ArucoFollower::imageCallback, this, _1)
    );

    // /cmd_vel as TwistStamped
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    // Publisher for go_home trigger
    go_home_pub_ = this->create_publisher<std_msgs::msg::Bool>("/go_home", 10);

    // Gesture commands: "follow" / "stop"
    gesture_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gesture_command",
      10,
      std::bind(&ArucoFollower::gestureCallback, this, _1)
    );

    // Control loop timer
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),   // 20 Hz
      std::bind(&ArucoFollower::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "aruco_follower_cpp node started. Waiting for images on %s",
                image_topic_.c_str());
  }

private:
  // --- Callbacks ---
  void gestureCallback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    const auto & cmd = msg->data;
    if (cmd == "FOLLOW") {
      follow_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "Gesture command: FOLLOW enabled.");
    } else if (cmd == "STOP") {
      follow_enabled_ = false;
      RCLCPP_INFO(this->get_logger(), "Gesture command: STOP; disabling follower.");
    } else if (cmd == "HOME") {
      follow_enabled_ = false;
      home_enabled_ = true;
      std_msgs::msg::Bool home_trigger;
      home_trigger.data = true;
      go_home_pub_->publish(home_trigger);
      RCLCPP_INFO(this->get_logger(), "HOME command received → triggering send_home");
    } else {
      follow_enabled_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "Unknown gesture command: '%s' (expected 'FOLLOW', 'STOP', or 'HOME')",
                  cmd.c_str());
    }
  }

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

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received image: %dx%d (encoding: %s)",
      frame.cols, frame.rows, msg->encoding.c_str());

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);

    if (ids.empty()) {
      has_marker_ = false;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No ArUco markers detected in current frame.");
      return;
    }

    has_marker_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Detected %zu ArUco marker(s). First id = %d",
      ids.size(), ids[0]);

    // Use the first marker for control
    const auto & marker_corners = corners[0];

    // Compute image-space center
    cv::Point2f center(0.f, 0.f);
    for (const auto & p : marker_corners) {
      center += p;
    }
    center *= 0.25f;

    last_center_x_ = center.x;
    last_image_width_ = frame.cols;

    // Estimate marker size in pixels: average of two edges
    // corners order: [top-left, top-right, bottom-right, bottom-left]
    double edge1 = cv::norm(marker_corners[0] - marker_corners[1]); // top edge
    double edge2 = cv::norm(marker_corners[1] - marker_corners[2]); // right edge
    last_marker_size_px_ = 0.5 * (edge1 + edge2);

    RCLCPP_INFO(
      this->get_logger(),
      "Marker center x=%.1f (image width=%d), approx size=%.1f px (edge1=%.1f, edge2=%.1f)",
      last_center_x_, last_image_width_,
      last_marker_size_px_, edge1, edge2);
  }

  void controlLoop()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.header.frame_id = cmd_frame_id_;

      // 1) HOME mode: don't publish anything, let send_home.py control
      if (!follow_enabled_ && home_enabled_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "HOME mode → not publishing cmd_vel (send_home.py controls robot)");
        return;
      }
  
      // 2) STOP mode from gesture (not HOME)
      else if (!follow_enabled_) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
  
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Follow disabled by gesture; publishing zero TwistStamped on /cmd_vel.");
        cmd_pub_->publish(cmd);
        return;
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Follow disabled by gesture; publishing zero TwistStamped on /cmd_vel.");
      cmd_pub_->publish(cmd);
      return;
    }

    // 2) Follow enabled, but no marker
    if (!has_marker_) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Follow enabled but no marker; publishing zero TwistStamped.");
      cmd_pub_->publish(cmd);
      return;
    }

    // 3) Follow enabled AND marker visible
    // Angular control from horizontal error
    double cx = static_cast<double>(last_center_x_);
    double width = static_cast<double>(last_image_width_);
    double error_x = (cx - width / 2.0) / (width / 2.0);  // [-1, 1]

    cmd.twist.angular.z = -k_angular_ * error_x;  // flip sign if needed

    // Linear control from pixel size error:
    // If marker is too small (far), move forward; too big (close), move backward.
    double size_error = desired_marker_size_px_ - last_marker_size_px_;
    // Normalize by desired size so gains are nicer
    double size_error_norm = size_error / std::max(1.0, desired_marker_size_px_);
    cmd.twist.linear.x = k_linear_ * size_error_norm;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "Publishing TwistStamped /cmd_vel: lin.x=%.3f, ang.z=%.3f "
      "(err_x=%.3f, marker_size_px=%.1f, desired_px=%.1f, follow_enabled=%d)",
      cmd.twist.linear.x, cmd.twist.angular.z,
      error_x, last_marker_size_px_, desired_marker_size_px_,
      follow_enabled_ ? 1 : 0);

    cmd_pub_->publish(cmd);
  }

  // --- Members ---

  // parameters
  std::string image_topic_;
  double marker_size_m_;
  double desired_distance_m_;  // not used anymore; just kept for info
  double k_linear_;
  double k_angular_;
  std::string cmd_frame_id_;
  double desired_marker_size_px_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // state
  bool   has_marker_{false};
  bool   follow_enabled_{false};  // controlled by /gesture_command
  bool   home_enabled_{false};    // controlled by /gesture_command
  float  last_center_x_{0.f};
  int    last_image_width_{1};
  double last_marker_size_px_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoFollower>());
  rclcpp::shutdown();
  return 0;
}
