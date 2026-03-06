/**
 * Camera Viewer Node
 *
 * Subscribes to camera image topics and displays them using OpenCV.
 * Shows EE camera (RGB + depth) and scene camera (RGB + depth).
 *
 * Topics:
 *   /camera/color/image_raw       - EE RGB image
 *   /camera/depth/image_raw       - EE Depth image
 *   /scene_camera/color/image_raw - Scene camera RGB
 *   /scene_camera/depth/image_raw - Scene camera Depth
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

class CameraViewer : public rclcpp::Node
{
public:
  CameraViewer() : Node("camera_viewer")
  {
    // Declare parameters
    this->declare_parameter<std::string>("ee_color_topic", "/camera/color/image_raw");
    this->declare_parameter<std::string>("ee_depth_topic", "/camera/depth/image_raw");
    this->declare_parameter<std::string>("scene_color_topic", "/scene_camera/color/image_raw");
    this->declare_parameter<std::string>("scene_depth_topic", "/scene_camera/depth/image_raw");
    this->declare_parameter<bool>("show_ee_camera", true);
    this->declare_parameter<bool>("show_ee_depth", true);
    this->declare_parameter<bool>("show_scene_camera", true);
    this->declare_parameter<bool>("show_scene_depth", true);

    std::string ee_color_topic = this->get_parameter("ee_color_topic").as_string();
    std::string ee_depth_topic = this->get_parameter("ee_depth_topic").as_string();
    std::string scene_color_topic = this->get_parameter("scene_color_topic").as_string();
    std::string scene_depth_topic = this->get_parameter("scene_depth_topic").as_string();
    show_ee_camera_ = this->get_parameter("show_ee_camera").as_bool();
    show_ee_depth_ = this->get_parameter("show_ee_depth").as_bool();
    show_scene_camera_ = this->get_parameter("show_scene_camera").as_bool();
    show_scene_depth_ = this->get_parameter("show_scene_depth").as_bool();

    RCLCPP_INFO(get_logger(), "Camera Viewer Node Started");
    RCLCPP_INFO(get_logger(), "========================================");

    // Subscribe to EE camera RGB
    if (show_ee_camera_) {
      RCLCPP_INFO(get_logger(), "EE RGB: %s", ee_color_topic.c_str());
      ee_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          ee_color_topic, 10,
          std::bind(&CameraViewer::eeColorCallback, this, std::placeholders::_1));
      cv::namedWindow("EE Camera (RGB)", cv::WINDOW_AUTOSIZE);
    }

    // Subscribe to EE camera Depth
    if (show_ee_depth_) {
      RCLCPP_INFO(get_logger(), "EE Depth: %s", ee_depth_topic.c_str());
      ee_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          ee_depth_topic, 10,
          std::bind(&CameraViewer::eeDepthCallback, this, std::placeholders::_1));
      cv::namedWindow("EE Camera (Depth)", cv::WINDOW_AUTOSIZE);
    }

    // Subscribe to Scene camera RGB
    if (show_scene_camera_) {
      RCLCPP_INFO(get_logger(), "Scene RGB: %s", scene_color_topic.c_str());
      scene_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          scene_color_topic, 10,
          std::bind(&CameraViewer::sceneColorCallback, this, std::placeholders::_1));
      cv::namedWindow("Scene Camera (RGB)", cv::WINDOW_AUTOSIZE);
    }

    // Subscribe to Scene camera Depth
    if (show_scene_depth_) {
      RCLCPP_INFO(get_logger(), "Scene Depth: %s", scene_depth_topic.c_str());
      scene_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          scene_depth_topic, 10,
          std::bind(&CameraViewer::sceneDepthCallback, this, std::placeholders::_1));
      cv::namedWindow("Scene Camera (Depth)", cv::WINDOW_AUTOSIZE);
    }

    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Press 'q' in any window to quit");
  }

  ~CameraViewer()
  {
    cv::destroyAllWindows();
  }

private:
  void eeColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::imshow("EE Camera (RGB)", cv_ptr->image);
      checkQuitKey();
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "EE color cv_bridge exception: %s", e.what());
    }
  }

  void eeDepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    displayDepthImage(msg, "EE Camera (Depth)");
  }

  void sceneColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::imshow("Scene Camera (RGB)", cv_ptr->image);
      checkQuitKey();
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Scene color cv_bridge exception: %s", e.what());
    }
  }

  void sceneDepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    displayDepthImage(msg, "Scene Camera (Depth)");
  }

  void displayDepthImage(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& window_name)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat depth_display;

      if (msg->encoding == "32FC1")
      {
        cv::Mat depth_normalized;
        // Replace infinity values with 0 for visualization
        cv::Mat depth_clean = cv_ptr->image.clone();
        depth_clean.setTo(0, depth_clean != depth_clean);  // Replace NaN with 0
        depth_clean.setTo(0, depth_clean > 100.0);  // Replace infinity/far values with 0

        double min_val, max_val;
        cv::minMaxLoc(depth_clean, &min_val, &max_val);

        // Use a reasonable max depth for normalization (e.g., 10 meters)
        double norm_max = (max_val > 0 && max_val < 100.0) ? max_val : 10.0;
        if (norm_max > 0) {
          depth_clean.convertTo(depth_normalized, CV_8UC1, 255.0 / norm_max);
          cv::applyColorMap(depth_normalized, depth_display, cv::COLORMAP_JET);
        }
      }
      else if (msg->encoding == "16UC1")
      {
        cv::Mat depth_normalized;
        cv_ptr->image.convertTo(depth_normalized, CV_8UC1, 255.0 / 10000.0);
        cv::applyColorMap(depth_normalized, depth_display, cv::COLORMAP_JET);
      }
      else
      {
        depth_display = cv_ptr->image;
      }

      if (!depth_display.empty()) {
        cv::imshow(window_name, depth_display);
      }
      checkQuitKey();
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Depth cv_bridge exception: %s", e.what());
    }
  }

  void checkQuitKey()
  {
    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q')
    {
      RCLCPP_INFO(get_logger(), "Quit requested");
      rclcpp::shutdown();
    }
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ee_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ee_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr scene_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr scene_depth_sub_;
  bool show_ee_camera_;
  bool show_ee_depth_;
  bool show_scene_camera_;
  bool show_scene_depth_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraViewer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

