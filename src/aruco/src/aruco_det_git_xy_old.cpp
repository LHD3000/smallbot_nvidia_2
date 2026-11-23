#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#define PI 3.14159265358979323846

class ArucoDetectorNode : public rclcpp::Node {
public:
  ArucoDetectorNode() : Node("aruco_det_git") {
    // 1. 声明参数（与原 launch 参数名对齐，并支持内参直接配置）
    this->declare_parameter<std::string>("camera_topic", "/image_raw/uncompressed");
    this->declare_parameter<std::string>("output_topic", "/prometheus/camera/rgb/image_aruco_det");
    this->declare_parameter<std::string>("pose_topic", "/camera_link_state");
    this->declare_parameter<std::string>("camera_frame", "camera");
    this->declare_parameter<int>("dictionary_type", 10);
    this->declare_parameter<double>("marker_length", 0.16);

    // 内参配置：优先使用 camera_matrix 参数；若 camera_parameters_yaml 非空则从 YAML 文件加载
    this->declare_parameter<std::string>("camera_parameters_yaml", "");
    this->declare_parameter<std::vector<double>>(
        "camera_matrix", {600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "dist_coeffs", {0.0, 0.0, 0.0, 0.0, 0.0});

    // 原代码 1205 大小码分级检测逻辑开关（默认 false，使用统一的 marker_length）
    this->declare_parameter<bool>("enable_dual_marker_mode", false);
    this->declare_parameter<int>("large_marker_id", 19);
    this->declare_parameter<double>("large_marker_length", 0.404);
    this->declare_parameter<int>("small_marker_id", 43);
    this->declare_parameter<double>("small_marker_length", 0.044);
    this->declare_parameter<double>("switch_distance_z", 5.0);

    // 2. 读取参数
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    dictionary_id_ = this->get_parameter("dictionary_type").as_int();
    marker_length_ = this->get_parameter("marker_length").as_double();

    enable_dual_marker_mode_ = this->get_parameter("enable_dual_marker_mode").as_bool();
    large_marker_id_ = this->get_parameter("large_marker_id").as_int();
    large_marker_length_ = this->get_parameter("large_marker_length").as_double();
    small_marker_id_ = this->get_parameter("small_marker_id").as_int();
    small_marker_length_ = this->get_parameter("small_marker_length").as_double();
    switch_distance_z_ = this->get_parameter("switch_distance_z").as_double();

    std::string yaml_path = this->get_parameter("camera_parameters_yaml").as_string();
    if (!yaml_path.empty()) {
      if (!readCameraParameters(yaml_path, cam_matrix_, dist_coeffs_)) {
        RCLCPP_FATAL(this->get_logger(), "无法读取相机内参文件: %s", yaml_path.c_str());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "已从 YAML 加载相机内参: %s", yaml_path.c_str());
    } else {
      auto cam_mat_vec = this->get_parameter("camera_matrix").as_double_array();
      auto dist_vec = this->get_parameter("dist_coeffs").as_double_array();
      if (cam_mat_vec.size() != 9) {
        RCLCPP_FATAL(this->get_logger(), "camera_matrix 长度必须为 9！");
        rclcpp::shutdown();
        return;
      }
      cam_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 9; ++i) {
        cam_matrix_.at<double>(i / 3, i % 3) = cam_mat_vec[i];
      }
      dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_vec.size()), CV_64F);
      for (size_t i = 0; i < dist_vec.size(); ++i) {
        dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_vec[i];
      }
      RCLCPP_INFO(this->get_logger(), "已从 Launch 参数加载相机内参");
    }

    // 3. 初始化 ArUco 字典与检测器（兼容 OpenCV 4.7+ 与旧版）
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    detector_params_ = cv::aruco::DetectorParameters();
    detector_ = std::make_unique<cv::aruco::ArucoDetector>(dictionary_, detector_params_);
#else
    detector_params_ = cv::aruco::DetectorParameters::create();
#endif

    // 4. 初始化订阅与发布
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ArucoDetectorNode::cameraCallback, this, std::placeholders::_1));

    cam_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pose_topic_, 10);
    cam_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic_ + "_stamped", 10);
    aruco_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "ArUco 检测节点启动成功 | 订阅: %s | 发布位姿: %s | 边长: %.4f m | 字典: %d",
                camera_topic_.c_str(), pose_topic_.c_str(), marker_length_, dictionary_id_);
  }

private:
  static bool readCameraParameters(const std::string &filename, cv::Mat &camMatrix,
                                   cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      return false;
    }
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return !camMatrix.empty();
  }

  // 单 Marker 位姿解算
  bool estimateSingleMarkerPose(const std::vector<cv::Point2f> &corners, double length,
                                cv::Vec3d &rvec, cv::Vec3d &tvec) {
    double half = length * 0.5;
    std::vector<cv::Point3f> obj_points = {
        cv::Point3f(-half,  half, 0.0f),
        cv::Point3f( half,  half, 0.0f),
        cv::Point3f( half, -half, 0.0f),
        cv::Point3f(-half, -half, 0.0f)};
    return cv::solvePnP(obj_points, corners, cam_matrix_, dist_coeffs_, rvec, tvec, false,
                        cv::SOLVEPNP_IPPE_SQUARE);
  }

  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cam_image;
    try {
      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
      return;
    }

    cv::Mat frame = cam_image->image;
    if (frame.empty()) {
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    detector_->detectMarkers(frame, corners, ids, rejected);
#else
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
#endif

    cv::Mat output = frame.clone();
    double disp_x = 0.0, disp_y = 0.0, disp_z = 0.0;
    bool valid_target_found = false;

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(output, corners, ids);

      for (size_t i = 0; i < ids.size(); ++i) {
        double current_len = marker_length_;

        if (enable_dual_marker_mode_) {
          if (ids[i] == large_marker_id_) {
            current_len = large_marker_length_;
          } else if (ids[i] == small_marker_id_) {
            current_len = small_marker_length_;
          } else {
            continue;
          }
        }

        cv::Vec3d rvec, tvec;
        if (!estimateSingleMarkerPose(corners[i], current_len, rvec, tvec)) {
          continue;
        }

        // 双码距离切换过滤（与原代码 z >= 5 / z < 5 逻辑一致）
        if (enable_dual_marker_mode_) {
          if (ids[i] == large_marker_id_ && tvec[2] < switch_distance_z_) {
            continue;
          }
          if (ids[i] == small_marker_id_ && tvec[2] >= switch_distance_z_) {
            continue;
          }
        }

        // 绘制坐标轴
        cv::drawFrameAxes(output, cam_matrix_, dist_coeffs_, rvec, tvec,
                          static_cast<float>(current_len * 0.5));

        // 旋转向量 -> 旋转矩阵 -> 四元数 & 欧拉角
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        Eigen::Matrix3d rotation_matrix_eigen;
        cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
        Eigen::Quaterniond q(rotation_matrix_eigen);
        q.normalize();

        // 1. 发布标准相机坐标系下二维码位姿 (geometry_msgs::msg::PoseStamped)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = camera_frame_;
        pose_msg.pose.position.x = tvec[0];
        pose_msg.pose.position.y = tvec[1];
        pose_msg.pose.position.z = tvec[2];
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        cam_pose_pub_->publish(pose_msg);

        // 2. 发布与原代码 /camera_link_state 兼容的 Odometry 消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = camera_frame_;
        odom_msg.child_frame_id = "aruco_" + std::to_string(ids[i]);

        // 保留原代码坐标轴映射（或直接读取 tvec[0], tvec[1], tvec[2]）
        odom_msg.pose.pose.position.x = tvec[0];
        odom_msg.pose.pose.position.y = tvec[1];
        odom_msg.pose.pose.position.z = tvec[2];
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 计算航向角 yaw_ego（保留原代码角度换算逻辑）
        Eigen::Vector3d euler_zyx = rotation_matrix_eigen.eulerAngles(2, 1, 0);
        double pitch_deg = euler_zyx[1] * 180.0 / PI;
        double yaw_ego = (pitch_deg >= 0.0) ? (90.0 + pitch_deg) : (90.0 - (180.0 + pitch_deg));
        if (yaw_ego < 0.0) {
          yaw_ego = -yaw_ego;
        }
        odom_msg.twist.twist.linear.x = yaw_ego;
        odom_msg.twist.twist.angular.x = euler_zyx[2] * 180.0 / PI;
        odom_msg.twist.twist.angular.y = pitch_deg;
        odom_msg.twist.twist.angular.z = euler_zyx[0] * 180.0 / PI;

        cam_odom_pub_->publish(odom_msg);

        if (!valid_target_found) {
          disp_x = tvec[0];
          disp_y = tvec[1];
          disp_z = tvec[2];
          valid_target_found = true;
        }
      }
    }

    // 在画面左上角渲染检测到的相机系 XYZ 坐标
    std::ostringstream oss;
    if (valid_target_found) {
      oss << std::fixed << std::setprecision(3)
          << "Cam X: " << disp_x << "  Y: " << disp_y << "  Z: " << disp_z << " m";
    } else {
      oss << "No Target Detected";
    }
    cv::putText(output, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    sensor_msgs::msg::Image::SharedPtr out_img_msg =
        cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, output).toImageMsg();
    aruco_img_pub_->publish(*out_img_msg);
  }

  std::string camera_topic_;
  std::string output_topic_;
  std::string pose_topic_;
  std::string camera_frame_;
  int dictionary_id_;
  double marker_length_;

  bool enable_dual_marker_mode_;
  int large_marker_id_;
  double large_marker_length_;
  int small_marker_id_;
  double small_marker_length_;
  double switch_distance_z_;

  cv::Mat cam_matrix_;
  cv::Mat dist_coeffs_;

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::Dictionary dictionary_;
  cv::aruco::DetectorParameters detector_params_;
  std::unique_ptr<cv::aruco::ArucoDetector> detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cam_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aruco_img_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}