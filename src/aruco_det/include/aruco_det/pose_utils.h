#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace aruco_det_core {
struct Calibration {
  cv::Mat k, d;
  int width = 0, height = 0;
  void validate() const {
    if (width <= 0 || height <= 0 || k.rows != 3 || k.cols != 3 ||
        !cv::checkRange(k) || !cv::checkRange(d) ||
        k.at<double>(0, 0) <= 0 || k.at<double>(1, 1) <= 0 ||
        std::abs(k.at<double>(2, 2) - 1) > 1e-9 ||
        std::abs(k.at<double>(2, 0)) > 1e-9 || std::abs(k.at<double>(2, 1)) > 1e-9)
      throw std::runtime_error("Invalid calibration: check image dimensions and camera matrix K");
    const auto n = d.total();
    if (n != 4 && n != 5 && n != 8 && n != 12 && n != 14)
      throw std::runtime_error("Distortion must have 4, 5, 8, 12 or 14 coefficients");
  }
};

inline void checkDistortionModel(const std::string& model) {
  if (model != "plumb_bob" && model != "rational_polynomial")
    throw std::runtime_error("Unsupported distortion_model: " + model +
                             "; supply an unrectified pinhole camera image");
}

inline Calibration loadCalibration(const std::string& path, int legacy_width, int legacy_height) {
  const YAML::Node c = YAML::LoadFile(path);
  Calibration result;
  if (c["camera_matrix"]) {
    result.width = c["image_width"].as<int>();
    result.height = c["image_height"].as<int>();
    checkDistortionModel(c["distortion_model"].as<std::string>());
    const auto k = c["camera_matrix"]["data"].as<std::vector<double>>();
    const auto d = c["distortion_coefficients"]["data"].as<std::vector<double>>();
    if (k.size() != 9 || d.empty()) throw std::runtime_error("Malformed calibration YAML");
    result.k = cv::Mat(k).reshape(1, 3).clone();
    result.d = cv::Mat(d).clone();
  } else {
    result.width = legacy_width;
    result.height = legacy_height;
    const double cx = c["cx"] ? c["cx"].as<double>() : c["x0"].as<double>();
    const double cy = c["cy"] ? c["cy"].as<double>() : c["y0"].as<double>();
    result.k = (cv::Mat_<double>(3, 3) << c["fx"].as<double>(), 0, cx,
                0, c["fy"].as<double>(), cy, 0, 0, 1);
    result.d = (cv::Mat_<double>(5, 1) << c["k1"].as<double>(), c["k2"].as<double>(),
                c["p1"].as<double>(), c["p2"].as<double>(), c["k3"].as<double>());
    std::clog << "Legacy calibration: resolution assumed from calibration_width/height ("
              << legacy_width << "x" << legacy_height << ")\n";
  }
  result.validate();
  if (c["calibration_verified"] && !c["calibration_verified"].as<bool>())
    std::clog << "Using UNVERIFIED legacy intrinsics. Calibrate this USB camera before trusting distances.\n";
  return result;
}

inline tf2::Transform toTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
  cv::Mat r;
  cv::Rodrigues(rvec, r);
  tf2::Matrix3x3 rotation(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
                        r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
                        r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2));
  tf2::Quaternion q;
  rotation.getRotation(q);  // Stable for the front-facing ~180 degree rotation.
  q.normalize();
  return tf2::Transform(q, tf2::Vector3(tvec[0], tvec[1], tvec[2]));
}

inline geometry_msgs::msg::Pose toPose(const tf2::Transform& t) {
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  p.orientation.x = t.getRotation().x();
  p.orientation.y = t.getRotation().y();
  p.orientation.z = t.getRotation().z();
  p.orientation.w = t.getRotation().w();
  return p;
}
}  // namespace
