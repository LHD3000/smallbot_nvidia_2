#include <aruco_det/pose_utils.h>
#include <aruco_det/msg/marker_array.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace aruco_det_core;

class ArucoNode : public rclcpp::Node {
 public:
  ArucoNode() : Node("aruco_det") {
    const int dictionary_id = declare_parameter("dictionary_id", 10);
    if (dictionary_id < 0 || dictionary_id > cv::aruco::DICT_APRILTAG_36h11)
      throw std::runtime_error("dictionary_id must be in [0,20]");
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    parameters_ = cv::aruco::DetectorParameters();
    parameters_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // Supplied print: 100 mm total including 10 mm white margin on each edge.
    // OpenCV uses the detected BLACK square, so the metric side is 80 mm.
    length_ = declare_parameter("aruco_length", 0.08);
    max_error_ = declare_parameter("max_reprojection_error", 3.0);
    primary_id_ = declare_parameter("primary_id", 0);
    publish_tf_ = declare_parameter("publish_tf", true);
    fallback_frame_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
    marker_prefix_ = declare_parameter<std::string>("marker_frame_prefix", "aruco_");
    if (!std::isfinite(length_) || length_ <= 0 || !std::isfinite(max_error_) || max_error_ <= 0)
      throw std::runtime_error("aruco_length and max_reprojection_error must be finite and positive");
    if (primary_id_ < 0 || primary_id_ >= dictionary_.bytesList.rows)
      throw std::runtime_error("primary_id is outside this dictionary");
    const auto target_ids = declare_parameter<std::vector<int64_t>>("target_aruco_ids", std::vector<int64_t>{});
    for (int64_t id : target_ids) {
      if (id < 0 || id >= dictionary_.bytesList.rows) throw std::runtime_error("Invalid target_aruco_ids");
      target_ids_.insert(id);
    }
    const auto path = declare_parameter<std::string>("camera_param_path", "");
    from_file_ = !declare_parameter("use_camera_info", false) && !path.empty();
    if (from_file_) {
      calibration_ = loadCalibration(path, declare_parameter("calibration_width", 640),
                                    declare_parameter("calibration_height", 480));
      calibrated_ = true;
      RCLCPP_INFO(get_logger(), "Loaded calibration %s (%dx%d)", path.c_str(), calibration_.width, calibration_.height);
    } else {
      info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          declare_parameter<std::string>("camera_info_topic", "/usb_cam/camera_info"),
          rclcpp::SensorDataQoS(), std::bind(&ArucoNode::cameraInfo, this, std::placeholders::_1));
    }
    markers_pub_ = create_publisher<aruco_det::msg::MarkerArray>("aruco/markers", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("aruco/pose", 1);
    camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("aruco/camera_pose", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("aruco_det_image", rclcpp::QoS(5));
    auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto image_reliability = declare_parameter<std::string>("image_qos_reliability", "best_effort");
    if (image_reliability == "reliable") image_qos.reliable();
    else if (image_reliability != "best_effort")
      throw std::runtime_error("image_qos_reliability must be reliable or best_effort");
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("sub_image_topic", "/usb_cam/image_raw"),
        image_qos, std::bind(&ArucoNode::image, this, std::placeholders::_1));
    last_image_ = std::chrono::steady_clock::now();
    watchdog_ = create_wall_timer(std::chrono::seconds(2), std::bind(&ArucoNode::watchdog, this));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "ArUco ready: dictionary=%d, black marker side=%.4f m, primary ID=%d",
             dictionary_id, length_, primary_id_);
  }

 private:
  void cameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    try {
      checkDistortionModel(msg->distortion_model);
      if (msg->binning_x > 1 || msg->binning_y > 1 || msg->roi.x_offset || msg->roi.y_offset ||
          (msg->roi.width && msg->roi.width != msg->width) ||
          (msg->roi.height && msg->roi.height != msg->height))
        throw std::runtime_error("Binned/cropped CameraInfo is unsupported; supply matching raw calibration");
      Calibration c;
      c.width = msg->width;
      c.height = msg->height;
      c.k = cv::Mat(3, 3, CV_64F);
      std::copy(msg->k.begin(), msg->k.end(), c.k.ptr<double>());
      c.d = cv::Mat(msg->d).clone();
      c.validate();
      calibration_ = c;
      info_frame_ = msg->header.frame_id;
      calibrated_ = true;
    } catch (const std::exception& e) {
      calibrated_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid CameraInfo: %s", e.what());
    }
  }

  void image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    last_image_ = std::chrono::steady_clock::now();
    try {
      // Process each new frame once and preserve its acquisition stamp.
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      aruco_det::msg::MarkerArray detections;
      detections.header = msg->header;
      if (detections.header.frame_id.empty()) detections.header.frame_id = fallback_frame_;
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::ArucoDetector detector(dictionary_, parameters_);
      detector.detectMarkers(frame, corners, ids);
      if (!ids.empty()) cv::aruco::drawDetectedMarkers(frame, corners, ids);
      const bool dimensions_ok = calibrated_ && frame.cols == calibration_.width &&
                                 frame.rows == calibration_.height;
      const bool frame_ok = from_file_ || info_frame_.empty() || info_frame_ == detections.header.frame_id;
      if (!dimensions_ok || !frame_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Pose disabled: missing/invalid calibration, resolution or frame mismatch. "
                          "Image=%dx%d, calibration=%dx%d", frame.cols, frame.rows,
                          calibration_.width, calibration_.height);
        cv::putText(frame, "NO MATCHING CALIBRATION - POSE DISABLED", cv::Point(8, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
      } else if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, length_, calibration_.k, calibration_.d, rvecs, tvecs);
        const float h = length_ / 2;
        const std::vector<cv::Point3f> object_corners = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
        std::set<int> seen;
        for (std::size_t i = 0; i < ids.size(); ++i) {
          const int id = ids[i];
          if ((!target_ids_.empty() && !target_ids_.count(id)) || !seen.insert(id).second) continue;
          if (!cv::checkRange(cv::Mat(rvecs[i])) || !cv::checkRange(cv::Mat(tvecs[i])) || tvecs[i][2] <= 0) continue;
          std::vector<cv::Point2f> projected;
          cv::projectPoints(object_corners, rvecs[i], tvecs[i], calibration_.k, calibration_.d, projected);
          double squared_error = 0;
          for (std::size_t j = 0; j < 4; ++j) {
            const cv::Point2f delta = projected[j] - corners[i][j];
            squared_error += delta.dot(delta);
          }
          const double error = std::sqrt(squared_error / 4);
          if (!std::isfinite(error) || error > max_error_) continue;
          const auto transform = toTransform(rvecs[i], tvecs[i]);
          aruco_det::msg::Marker marker;
          marker.id = id;
          marker.pose = toPose(transform);
          marker.reprojection_error = error;
          detections.markers.push_back(marker);
          geometry_msgs::msg::PoseStamped pose;
          pose.header = detections.header;
          pose.pose = marker.pose;
          auto pub = id_publishers_.find(id);
          if (pub == id_publishers_.end())
            pub = id_publishers_.emplace(id, create_publisher<geometry_msgs::msg::PoseStamped>(
                "aruco/id_" + std::to_string(id) + "/pose", 1)).first;
          pub->second->publish(pose);
          const std::string marker_frame = marker_prefix_ + std::to_string(id);
          if (id == primary_id_) {
            pose_pub_->publish(pose);
            geometry_msgs::msg::PoseStamped camera_pose;
            camera_pose.header = detections.header;
            camera_pose.header.frame_id = marker_frame;
            camera_pose.pose = toPose(transform.inverse());
            camera_pose_pub_->publish(camera_pose);
          }
          if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = detections.header;
            tf.child_frame_id = marker_frame;
            tf.transform.translation.x = pose.pose.position.x;
            tf.transform.translation.y = pose.pose.position.y;
            tf.transform.translation.z = pose.pose.position.z;
            tf.transform.rotation = pose.pose.orientation;
            broadcaster_->sendTransform(tf);
          }
          cv::drawFrameAxes(frame, calibration_.k, calibration_.d, rvecs[i], tvecs[i], length_ * 0.5);
        }
      }
      // Display the same camera-frame centre coordinates that subscribers receive.
      // A row per accepted marker keeps the full XYZ label visible near image edges.
      for (std::size_t i = 0; i < detections.markers.size(); ++i) {
        const auto& marker = detections.markers[i];
        const auto& p = marker.pose.position;
        const std::string label = cv::format("ID %d  X=%+.3f  Y=%+.3f  Z=%.3f m",
                                             marker.id, p.x, p.y, p.z);
        const cv::Point origin(8, 24 + 24 * static_cast<int>(i));
        cv::putText(frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
      markers_pub_->publish(detections);
      image_pub_->publish(*cv_bridge::CvImage(detections.header, "bgr8", frame).toImageMsg());
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Image processing failed: %s", e.what());
    }
  }

  void watchdog() {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_image_).count() > 3.0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No new camera images for >3 s; check usb_cam and image topic. No pose is republished.");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<aruco_det::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, camera_pose_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::chrono::steady_clock::time_point last_image_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> id_publishers_;
  std::set<int> target_ids_;
  cv::aruco::Dictionary dictionary_;
  cv::aruco::DetectorParameters parameters_;
  Calibration calibration_;
  bool calibrated_ = false, from_file_ = false, publish_tf_ = true;
  int primary_id_ = 0;
  double length_ = 0.08, max_error_ = 3;
  std::string fallback_frame_, marker_prefix_, info_frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<ArucoNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("aruco_det"), "Cannot run aruco_det: %s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
