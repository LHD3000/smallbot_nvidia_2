#include <cmath>
#include <memory>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <fstream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#define PI 3.14159265358979323846

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_det_git")
    {
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

        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::cameraCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/camera_pose", 10);
        cam_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/camera_link_state", 10);
        aruco_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

        // 打开数据记录文件（原始 / 滤波后 / 各自时间戳）
        const char* home = std::getenv("HOME");
        std::string data_dir = std::string(home ? home : "/home/nvidia") +
                               "/jeston/smallbot_ws/src/aruco/src/";

        raw_x_file_.open(data_dir + "vision_raw_x.txt");
        filt_x_file_.open(data_dir + "vision_filtered_x.txt");
        ts_raw_x_file_.open(data_dir + "vision_timestamp_raw_x.txt");
        ts_filt_x_file_.open(data_dir + "vision_timestamp_filtered_x.txt");

        raw_y_file_.open(data_dir + "vision_raw_y.txt");
        filt_y_file_.open(data_dir + "vision_filtered_y.txt");
        ts_raw_y_file_.open(data_dir + "vision_timestamp_raw_y.txt");
        ts_filt_y_file_.open(data_dir + "vision_timestamp_filtered_y.txt");

        raw_z_file_.open(data_dir + "vision_raw_z.txt");
        filt_z_file_.open(data_dir + "vision_filtered_z.txt");
        ts_raw_z_file_.open(data_dir + "vision_timestamp_raw_z.txt");
        ts_filt_z_file_.open(data_dir + "vision_timestamp_filtered_z.txt");

        raw_yaw_file_.open(data_dir + "vision_raw_yaw.txt");
        filt_yaw_file_.open(data_dir + "vision_filtered_yaw.txt");
        ts_raw_yaw_file_.open(data_dir + "vision_timestamp_raw_yaw.txt");
        ts_filt_yaw_file_.open(data_dir + "vision_timestamp_filtered_yaw.txt");

        if (!raw_x_file_)
            RCLCPP_WARN(this->get_logger(), "Cannot open vision data files in %s", data_dir.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "Vision data log dir: %s", data_dir.c_str());
    }

    ~ArucoDetectorNode()
    {
        raw_x_file_.close();  filt_x_file_.close();
        ts_raw_x_file_.close();  ts_filt_x_file_.close();
        raw_y_file_.close();  filt_y_file_.close();
        ts_raw_y_file_.close();  ts_filt_y_file_.close();
        raw_z_file_.close();  filt_z_file_.close();
        ts_raw_z_file_.close();  ts_filt_z_file_.close();
        raw_yaw_file_.close();  filt_yaw_file_.close();
        ts_raw_yaw_file_.close();  ts_filt_yaw_file_.close();
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
        double disp_angle_y_deg = 0.0;

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

                    // 二维码法线（局部 z 轴）投影到相机 x-z 平面，绕相机 y 轴相对偏航角
                    Eigen::Vector3d normal = rotation_matrix_eigen.col(2);
                    double angle_y_rad = std::atan2(-normal.x(), -normal.z());
                    disp_angle_y_deg = angle_y_rad * 180.0 / PI;

                    // 一阶低通滤波（发布前）：f = α·raw + (1-α)·f_prev
                    if (!filtered_init_)
                    {
                        filt_x_ = tvec[0];
                        filt_y_ = tvec[1];
                        filt_z_ = tvec[2];
                        filt_angle_y_ = disp_angle_y_deg;
                        filtered_init_ = true;
                    }
                    else
                    {
                        filt_x_   = lpf_alpha_ * tvec[0]               + (1 - lpf_alpha_) * filt_x_;
                        filt_y_   = lpf_alpha_ * tvec[1]               + (1 - lpf_alpha_) * filt_y_;
                        filt_z_   = lpf_alpha_ * tvec[2]               + (1 - lpf_alpha_) * filt_z_;
                        filt_angle_y_ = lpf_alpha_ * disp_angle_y_deg  + (1 - lpf_alpha_) * filt_angle_y_;
                    }

                    // 记录原始 / 滤波后数据与时间戳
                    if (raw_x_file_.is_open())
                    {
                        double now_ts = this->now().seconds();

                        raw_x_file_  << tvec[0] << std::endl;
                        filt_x_file_ << filt_x_ << std::endl;
                        ts_raw_x_file_  << std::fixed << std::setprecision(9) << now_ts << std::endl;
                        ts_filt_x_file_ << std::fixed << std::setprecision(9) << now_ts << std::endl;

                        raw_y_file_  << tvec[1] << std::endl;
                        filt_y_file_ << filt_y_ << std::endl;
                        ts_raw_y_file_  << std::fixed << std::setprecision(9) << now_ts << std::endl;
                        ts_filt_y_file_ << std::fixed << std::setprecision(9) << now_ts << std::endl;

                        raw_z_file_  << tvec[2] << std::endl;
                        filt_z_file_ << filt_z_ << std::endl;
                        ts_raw_z_file_  << std::fixed << std::setprecision(9) << now_ts << std::endl;
                        ts_filt_z_file_ << std::fixed << std::setprecision(9) << now_ts << std::endl;

                        raw_yaw_file_  << disp_angle_y_deg << std::endl;
                        filt_yaw_file_ << filt_angle_y_ << std::endl;
                        ts_raw_yaw_file_  << std::fixed << std::setprecision(9) << now_ts << std::endl;
                        ts_filt_yaw_file_ << std::fixed << std::setprecision(9) << now_ts << std::endl;
                    }

                    nav_msgs::msg::Odometry cam_pose;
                    cam_pose.header.stamp = msg->header.stamp;
                    cam_pose.header.frame_id = camera_frame_;
                    // 位置发布滤波后的值
                    cam_pose.pose.pose.position.x = filt_x_;
                    cam_pose.pose.pose.position.y = filt_y_;
                    cam_pose.pose.pose.position.z = filt_z_;
                    cam_pose.pose.pose.orientation = pose_msg.pose.orientation;

                    Eigen::Matrix3d inv_rot = rotation_matrix_eigen.inverse();
                    Eigen::Vector3d inv_euler = inv_rot.eulerAngles(0, 2, 1);
                    cam_pose.twist.twist.angular.x = inv_euler[2] * 180.0 / PI;
                    cam_pose.twist.twist.angular.y = inv_euler[1] * 180.0 / PI;
                    cam_pose.twist.twist.angular.z = inv_euler[0] * 180.0 / PI;

                    // 航向角发布滤波后的 AngleY
                    cam_pose.twist.twist.linear.x = filt_angle_y_;
                    cam_pose_pub_->publish(cam_pose);
                }
            }
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "Cam X: " << disp_x << "  Y: " << disp_y << "  Z: " << disp_z
            << " | AngleY: " << disp_angle_y_deg << " deg";

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

    // 一阶低通滤波状态（成员变量，跨帧保持）
    bool filtered_init_ = false;
    double lpf_alpha_ = 0.4;   // 0~1：越小越平滑、滞后越大
    double filt_x_ = 0.0, filt_y_ = 0.0, filt_z_ = 0.0, filt_angle_y_ = 0.0;

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_params_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;
#else
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
#endif

    // 数据记录文件：位置 X/Y/Z、航向角 Yaw，原始 + 滤波后 + 各自时间戳
    std::ofstream raw_x_file_, filt_x_file_, ts_raw_x_file_, ts_filt_x_file_;
    std::ofstream raw_y_file_, filt_y_file_, ts_raw_y_file_, ts_filt_y_file_;
    std::ofstream raw_z_file_, filt_z_file_, ts_raw_z_file_, ts_filt_z_file_;
    std::ofstream raw_yaw_file_, filt_yaw_file_, ts_raw_yaw_file_, ts_filt_yaw_file_;

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
