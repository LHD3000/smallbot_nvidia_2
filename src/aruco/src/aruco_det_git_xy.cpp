#include <cmath>
#include <memory>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp" // [修改处 1-a] 新增压缩图像消息头文件
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp> // [修改处 1-b] 新增 imgcodecs 头文件以调用 cv::imdecode
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#define PI 3.14159265358979323846

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_det_git")
    {
        // [修改处 2] 默认话题改为 compressed 话题
        this->declare_parameter<std::string>("camera_topic", "/image_raw/compressed");
        this->declare_parameter<std::string>("output_topic", "/prometheus/camera/rgb/image_aruco_det");
        this->declare_parameter<std::string>("camera_parameters", "");
        this->declare_parameter<int>("dictionary_type", 10);
        this->declare_parameter<double>("marker_length", 0.16);
        this->declare_parameter<std::string>("camera_frame", "camera");

        this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>{});

        camera_topic_ = this->get_parameter("camera_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        std::string yaml_path = this->get_parameter("camera_parameters").as_string();
        int dictionary_id = this->get_parameter("dictionary_type").as_int();
        marker_length_ = this->get_parameter("marker_length").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();

        auto cam_mat_param = this->get_parameter("camera_matrix").as_double_array();
        auto dist_param = this->get_parameter("distortion_coefficients").as_double_array();

        if (cam_mat_param.size() == 9 && dist_param.size() >= 4)
        {
            cam_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; ++i)
                cam_matrix_.at<double>(i / 3, i % 3) = cam_mat_param[i];

            dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_param.size()), CV_64F);
            for (size_t i = 0; i < dist_param.size(); ++i)
                dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_param[i];

            RCLCPP_INFO(this->get_logger(), "Loaded camera parameters directly from launch arguments.");
        }
        else if (!yaml_path.empty())
        {
            if (!readCameraParameters(yaml_path, cam_matrix_, dist_coeffs_))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid camera parameter file: %s", yaml_path.c_str());
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Loaded camera parameters from YAML: %s", yaml_path.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No valid camera_parameters yaml or camera_matrix provided!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "camera_topic: %s | marker_length: %.4f | dictionary: %d",
                    camera_topic_.c_str(), marker_length_, dictionary_id);

        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        detector_params_ = cv::aruco::DetectorParameters();
        detector_ = std::make_unique<cv::aruco::ArucoDetector>(dictionary_, detector_params_);
#else
        detector_params_ = cv::aruco::DetectorParameters::create();
#endif

        // [修改处 3] 订阅者消息类型改为 sensor_msgs::msg::CompressedImage
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::cameraCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/camera_pose", 10);
        cam_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/camera_link_state", 10);
        aruco_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
    }

private:
    bool readCameraParameters(const std::string &filename, cv::Mat &camMatrix, cv::Mat &distCoeffs)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
            return false;
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        return !camMatrix.empty() && !distCoeffs.empty();
    }

    // [修改处 4] 回调参数改为 CompressedImage::SharedPtr，并用 cv::imdecode 直接解码
    void cameraCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image.");
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        detector_->detectMarkers(frame, corners, ids, rejected);
#else
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
#endif

        cv::Mat frameCopy = frame.clone();
        double disp_x = 0.0, disp_y = 0.0, disp_z = 0.0;

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);

            double half_len = marker_length_ / 2.0;
            std::vector<cv::Point3f> obj_points = {
                cv::Point3f(-half_len,  half_len, 0.0f),
                cv::Point3f( half_len,  half_len, 0.0f),
                cv::Point3f( half_len, -half_len, 0.0f),
                cv::Point3f(-half_len, -half_len, 0.0f)};

            for (size_t i = 0; i < ids.size(); ++i)
            {
                cv::Vec3d rvec, tvec;
                bool ok = cv::solvePnP(obj_points, corners[i], cam_matrix_, dist_coeffs_,
                                       rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
                if (!ok)
                    continue;

                cv::drawFrameAxes(frameCopy, cam_matrix_, dist_coeffs_, rvec, tvec,
                                  static_cast<float>(marker_length_ * 0.5f));

                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                Eigen::Matrix3d rotation_matrix_eigen;
                cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
                Eigen::Quaterniond q(rotation_matrix_eigen);
                q.normalize();

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
                pose_pub_->publish(pose_msg);

                if (i == 0)
                {
                    disp_x = tvec[0];
                    disp_y = tvec[1];
                    disp_z = tvec[2];

                    Eigen::Vector3d eulerAngles = rotation_matrix_eigen.eulerAngles(2, 1, 0);
                    double sight_angle_y = eulerAngles[1] * 180.0 / PI;

                    nav_msgs::msg::Odometry cam_pose;
                    cam_pose.header.stamp = msg->header.stamp;
                    cam_pose.header.frame_id = camera_frame_;
                    cam_pose.pose.pose.position.x = -tvec[0];
                    cam_pose.pose.pose.position.y = -tvec[2];
                    cam_pose.pose.pose.position.z =  tvec[1];
                    cam_pose.pose.pose.orientation = pose_msg.pose.orientation;

                    Eigen::Matrix3d inv_rot = rotation_matrix_eigen.inverse();
                    Eigen::Vector3d inv_euler = inv_rot.eulerAngles(0, 2, 1);
                    cam_pose.twist.twist.angular.x = inv_euler[2] * 180.0 / PI;
                    cam_pose.twist.twist.angular.y = inv_euler[1] * 180.0 / PI;
                    cam_pose.twist.twist.angular.z = inv_euler[0] * 180.0 / PI;

                    double yaw_ego = (sight_angle_y >= 0.0) ? (90.0 + sight_angle_y)
                                                            : (90.0 - (180.0 + sight_angle_y));
                    if (yaw_ego < 0.0)
                        yaw_ego = -yaw_ego;

                    cam_pose.twist.twist.linear.x = yaw_ego;
                    cam_pose_pub_->publish(cam_pose);
                }
            }
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "Cam X: " << disp_x << "  Y: " << disp_y << "  Z: " << disp_z;
        cv::putText(frameCopy, oss.str(), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frameCopy).toImageMsg();
        aruco_img_pub_->publish(*out_msg);
    }

    std::string camera_topic_;
    std::string output_topic_;
    std::string camera_frame_;
    double marker_length_;

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

    // [修改处 5] 订阅者成员变量类型同步更新为 CompressedImage
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cam_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aruco_img_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}