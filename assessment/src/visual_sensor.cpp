// Copyright (c) 2025 University of York and others
//
// Contributors:
//   * Pedro Ribeiro - revised C++ implementation
//
// C++ port of the Python item_sensor node (ROS2).
//

#include <memory>
#include <map>
#include <opencv2/core/ocl.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <assessment_interfaces/msg/barrel.hpp>
#include <assessment_interfaces/msg/barrel_list.hpp>
#include <assessment_interfaces/msg/zone.hpp>
#include <assessment_interfaces/msg/zone_list.hpp>

using std::placeholders::_1;

using assessment_interfaces::msg::Zone;
using assessment_interfaces::msg::ZoneList;
using assessment_interfaces::msg::Barrel;
using assessment_interfaces::msg::BarrelList;

using namespace cv;
using namespace std;

// Converts an RGB color to HSV and returns it as a cv::Vec3b (H, S, V)
Vec3b rgbToHsv(int r, int g, int b) {
    Mat rgb(1, 1, CV_8UC3, cv::Scalar(r, g, b));  // 1x1 image with RGB color
    Mat hsv;
    cvtColor(rgb, hsv, COLOR_RGB2HSV);
    return hsv.at<Vec3b>(0, 0);
}

class VisualSensor : public rclcpp::Node
{
  public: VisualSensor() : Node("visual_sensor")
  {
    // Subscriber to raw camera images
    image_sub_ = image_transport::create_subscription(
      this, "camera/image_raw",
      std::bind(&VisualSensor::image_callback, this, _1),
      "raw");

    // Declare parameters with defaults
    this->declare_parameter<bool>("skip_similar_frames", true);
    this->declare_parameter<bool>("publish_image_zones", false);
    this->declare_parameter<bool>("publish_image_barrels", false);
    this->declare_parameter<bool>("publish_image_zones_mask", false);
    this->declare_parameter<double>("frame_divider", 0.5);

    // Get parameter values
    this->get_parameter("skip_similar_frames", skip_similar_frames_);
    this->get_parameter("publish_image_zones", publish_image_zones_);
    this->get_parameter("publish_image_barrels", publish_image_barrels_);
    this->get_parameter("publish_image_zones_mask", publish_image_zones_mask_);
    this->get_parameter("frame_divider", frame_divider_);

    // Publisher for annotated images
    image_pub_ = image_transport::create_publisher(this, "camera/image_barrels"); 

    // Publisher for list of items
    barrel_list_pub_ = this->create_publisher<assessment_interfaces::msg::BarrelList>("barrels", 10);

    // Publisher for zone images annotated
    zone_image_pub_ = image_transport::create_publisher(this, "camera/image_zones"); 

    // Publisher for zone images annotated
    zone_mask_pub_ = image_transport::create_publisher(this, "camera/image_zones_mask"); 

    // Publisher for list of zones detected
    zone_list_pub_ = this->create_publisher<assessment_interfaces::msg::ZoneList>("zones", 10);

    // Setup colours
    red_    = rgbToHsv(240, 49, 20);
    cyan_   = rgbToHsv(0, 255, 255);
    green_  = rgbToHsv(46, 139, 87);
    blue_   = rgbToHsv(0, 0, 248);

    // Check for OpenCL as some operations could be accelerated, if present
    if (cv::ocl::haveOpenCL()) {
        cv::ocl::Context context;
        if (!context.create(cv::ocl::Device::TYPE_GPU)) {
            std::cout << "No GPU OpenCL context created\n";
        }

        cv::ocl::Device device = context.device(0);
        std::cout << "OpenCL device in use: " << device.name() << std::endl;
        std::cout << "Vendor: " << device.vendorName() << std::endl;
        std::cout << "Version: " << device.OpenCLVersion() << std::endl;
    } else {
        std::cout << "OpenCL not available.\n";
    }
  }

private:

  // Node parameters
  bool skip_similar_frames_;
  bool publish_image_zones_;
  bool publish_image_barrels_;
  bool publish_image_zones_mask_;

  // HSV colours
  cv::Vec3b cyan_, green_, red_, blue_;

  // ROS2 publishers and subscribers
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher zone_image_pub_;
  image_transport::Publisher zone_mask_pub_;

  rclcpp::Publisher<assessment_interfaces::msg::BarrelList>::SharedPtr barrel_list_pub_;
  rclcpp::Publisher<assessment_interfaces::msg::ZoneList>::SharedPtr zone_list_pub_;

  // store previous grayscale frame
  cv::UMat prev_gray_;
  // % of changed pixels
  const double CHANGE_THRESHOLD = 1.0; 
  // brightness difference for pixel considered "changed"
  const int PIXEL_DIFF_THRESHOLD = 20; 

  double frame_divider_;
  double accumulator = 0.0;
  double alpha = 0.3;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
      accumulator += frame_divider_;
      if (accumulator < 1.0) return; // skip
      accumulator -= 1.0;

      // Convert ROS Image to cv::UMat
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
          cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      } catch (cv_bridge::Exception &e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }

      // Copy to UMat for OpenCL acceleration (if available)
      cv::UMat frame_u;
      cv_ptr->image.copyTo(frame_u);

      // If the camera publishes RGB8, convert to BGR for OpenCV
      if (msg->encoding == "rgb8")
      {
          cv::cvtColor(frame_u, frame_u, cv::COLOR_RGB2BGR);
      }
      // If it’s already BGR, no conversion needed
      else if (msg->encoding == "bgr8")
      {
          // do nothing
      }
      else
      {
          RCLCPP_WARN(this->get_logger(), "Unexpected image encoding: %s", msg->encoding.c_str());
          return;
      }

      if (skip_similar_frames_) {
        // Grayscale conversion for frame-change gating
        cv::UMat gray_u;
        cv::cvtColor(frame_u, gray_u, cv::COLOR_BGR2GRAY);

        // Simple frame-change detection
        if (!prev_gray_.empty())
        {
            cv::UMat diff_u, mask_u;
            cv::absdiff(gray_u, prev_gray_, diff_u);
            cv::threshold(diff_u, mask_u, PIXEL_DIFF_THRESHOLD, 255, cv::THRESH_BINARY);

            double changed_pixels = cv::countNonZero(mask_u);
            double total_pixels = frame_u.rows * frame_u.cols;
            double change_percent = (changed_pixels / total_pixels) * 100.0;

            if (change_percent < CHANGE_THRESHOLD)
            {
                RCLCPP_DEBUG(this->get_logger(), "Frame skipped (%.2f%% change)", change_percent);
                return;
            }
            RCLCPP_DEBUG(this->get_logger(), "Frame processed (%.2f%% change)", change_percent);
        }

        prev_gray_ = gray_u.clone();
      }

      // Convert frame to HSV in UMat
      cv::UMat hsv_u;
      cv::cvtColor(frame_u, hsv_u, cv::COLOR_BGR2HSV);

      // Process zones
      process_zones(frame_u, hsv_u, msg);

      // Process barells
      process_barrels(frame_u, hsv_u, msg);
  }

  void process_barrels(const cv::UMat& frame_u, 
                       const cv::UMat& hsv_u, 
                       const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    
    // Colour masks in UMat
    cv::UMat red_mask1_u, red_mask2_u, red_mask_u;
    cv::UMat blue_mask_u;

    // HSV thresholds
    const int SATURATION_LOWER = 100, SATURATION_UPPER = 255;
    const int VALUE_LOWER = 20, VALUE_UPPER = 255;

    cv::inRange(hsv_u, cv::Scalar(0, 100, 20), cv::Scalar(10, 255, 255), red_mask1_u);
    cv::inRange(hsv_u, cv::Scalar(170, 100, 20), cv::Scalar(180, 255, 255), red_mask2_u);
    cv::bitwise_or(red_mask1_u, red_mask2_u, red_mask_u);

    cv::inRange(hsv_u, cv::Scalar(110, 100, 20), cv::Scalar(130, 255, 255), blue_mask_u);

    // Prepare overlay for drawing
    cv::UMat overlay_u;
    frame_u.copyTo(overlay_u);

    BarrelList barrel_list_msg;

    for (auto colour : { Barrel::RED, Barrel::BLUE }) {

        cv::UMat mask_u;
        uint8_t barrel_type;
        std::string text;

        switch (colour) {
            case Barrel::RED:
                mask_u = red_mask_u; 
                barrel_type = Barrel::RED; 
                text = "R"; 
                break;
            case Barrel::BLUE:
                mask_u = blue_mask_u; 
                barrel_type = Barrel::BLUE; 
                text = "B"; 
                break;
        }

        cv::UMat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(mask_u, labels, stats, centroids, 8, CV_32S);

        // Only download small summary matrices (rows = number of labels)
        cv::Mat stats_mat = stats.getMat(cv::ACCESS_READ);
        cv::Mat centroids_mat = centroids.getMat(cv::ACCESS_READ);

        // Draw rectangles directly on overlay_u (GPU)
        for (int label = 1; label < num_labels; ++label) { // skip background
            int area = stats_mat.at<int>(label, cv::CC_STAT_AREA);
            if (area < 150) continue;

            int x = stats_mat.at<int>(label, cv::CC_STAT_LEFT);
            int y = stats_mat.at<int>(label, cv::CC_STAT_TOP);
            int w = stats_mat.at<int>(label, cv::CC_STAT_WIDTH);
            int h = stats_mat.at<int>(label, cv::CC_STAT_HEIGHT);

            float aspect_ratio = (w > 0) ? static_cast<float>(h) / w : 0.0f;
            if (aspect_ratio < 1.0f) aspect_ratio = 1.0f / aspect_ratio;
            if (aspect_ratio > 5.0f) continue;

            float center_x = static_cast<float>(centroids_mat.at<double>(label, 0));
            float center_y = static_cast<float>(centroids_mat.at<double>(label, 1));

            // Publish barrel info
            Barrel barrel_msg;
            barrel_msg.x = static_cast<int16_t>(center_x);
            barrel_msg.y = static_cast<int16_t>(center_y);
            barrel_msg.size = static_cast<float>(area);
            barrel_msg.colour = barrel_type;
            barrel_list_msg.data.push_back(barrel_msg);

            // Draw filled rectangle on overlay (GPU)
            if (publish_image_barrels_) {
                cv::rectangle(overlay_u, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
            }
        }
    }

    // Publish results
    barrel_list_pub_->publish(barrel_list_msg);

    // Convert overlay to Mat only once, after all rectangles drawn
    if (publish_image_barrels_) {
        cv::UMat blended_u;
        cv::addWeighted(overlay_u, alpha, frame_u, 1.0 - alpha, 0, blended_u);

        cv::Mat overlay_mat = blended_u.getMat(cv::ACCESS_READ);

        // Optional: draw text for all barrels in one loop
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 1.0;
        int thickness = 2;
        int baseline = 0;

        for (const auto& barrel : barrel_list_msg.data) {
            std::string text;
            switch (barrel.colour) {
                case Barrel::RED:
                    text = "R"; 
                    break;
                case Barrel::BLUE:
                    text = "B"; 
                    break;
            }
            cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
            cv::Point text_origin(barrel.x - text_size.width / 2,
                                  barrel.y + text_size.height / 2);

            cv::putText(overlay_mat, text, text_origin, font, font_scale,
                        cv::Scalar(0, 0, 0), thickness * 4, cv::LINE_AA);
            cv::putText(overlay_mat, text, text_origin, font, font_scale,
                        cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
        }
        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", overlay_mat).toImageMsg();
        image_pub_.publish(out_msg);
    }
  }

  void process_zones(const cv::UMat& frame, 
                     const cv::UMat& hsv, 
                     const sensor_msgs::msg::Image::ConstSharedPtr msg) {

    ZoneList zone_list_msg;

    int image_width  = frame.cols;
    int image_height = frame.rows;

    cv::UMat augmented;
    cv::Mat hsv_norm = hsv.getMat(cv::ACCESS_READ);
    frame.copyTo(augmented);

    // HSV thresholds
    const int SATURATION_LOWER = 100, SATURATION_UPPER = 255;
    const int VALUE_LOWER = 20, VALUE_UPPER = 255;

    // Create masks for each color
    cv::UMat cyan_mask, green_mask;

    cv::inRange(hsv,
        cv::Scalar(cyan_[0], SATURATION_LOWER, VALUE_LOWER),
        cv::Scalar(cyan_[0], SATURATION_UPPER, VALUE_UPPER),
        cyan_mask);

    cv::inRange(hsv,
        cv::Scalar(green_[0], SATURATION_LOWER, VALUE_LOWER),
        cv::Scalar(green_[0]+1, SATURATION_UPPER, VALUE_UPPER),
        green_mask);

    // Iterate over colours
    for (auto colour : { Zone::ZONE_CYAN, Zone::ZONE_GREEN }) {
        cv::UMat mask;
        uint8_t zone_type;
        std::string text;

        switch (colour) {
            case Zone::ZONE_CYAN:
                mask = cyan_mask; 
                zone_type = Zone::ZONE_CYAN; 
                text = "CYAN ZONE"; 
                break;
            case Zone::ZONE_GREEN:
                mask = green_mask; 
                zone_type = Zone::ZONE_GREEN; 
                text = "GREEN ZONE"; 
                break;
        }

        cv::Moments moments = cv::moments(mask, true);

        if (moments.m00 != 0) {
            int centre_x = static_cast<int>(moments.m10 / moments.m00);
            int centre_y = static_cast<int>(moments.m01 / moments.m00);

            if (publish_image_zones_) {
              const cv::Scalar black(0, 0, 0);
              const cv::Scalar white(255, 255, 255);

              int font = cv::FONT_HERSHEY_SIMPLEX;
              double font_scale = 1.0;
              int font_thickness = 2;

              int baseline = 0;
              cv::Size text_size = cv::getTextSize(text, font, font_scale, font_thickness, &baseline);
              cv::Point text_position(
                  centre_x - text_size.width / 2,
                  centre_y + text_size.height * 2
              );

              cv::putText(augmented, text, text_position, font, font_scale, black, font_thickness * 4, cv::LINE_AA);
              cv::putText(augmented, text, text_position, font, font_scale, white, font_thickness, cv::LINE_AA);

              cv::circle(augmented, cv::Point(centre_x, centre_y), 10, black, -1, cv::LINE_AA);
              cv::circle(augmented, cv::Point(centre_x, centre_y), 6, white, -1, cv::LINE_AA);
            }

            Zone msg;
            msg.zone = zone_type;
            msg.x = static_cast<int>((image_width / 2) - centre_x);
            msg.y = static_cast<int>((image_height / 2) - centre_y);

            // Calculate size ratio (non-zero pixels)
            msg.size = static_cast<double>(cv::countNonZero(mask)) / (image_width * image_height);

            zone_list_msg.data.push_back(msg);
        }
    }

    // Publish zone list
    zone_list_pub_->publish(zone_list_msg);

    // Publish augmented image
    if (publish_image_zones_) {
      auto augmented_msg = cv_bridge::CvImage(msg->header, "bgr8", augmented.getMat(cv::ACCESS_READ)).toImageMsg();
      zone_image_pub_.publish(augmented_msg);
    }

    // Combine all masks into one
    if (publish_image_zones_mask_) {
      cv::UMat combined_mask;
      cv::bitwise_or(cyan_mask, green_mask, combined_mask);

      auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", combined_mask.getMat(cv::ACCESS_READ)).toImageMsg();
      zone_mask_pub_.publish(mask_msg);
    }
  };

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualSensor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
