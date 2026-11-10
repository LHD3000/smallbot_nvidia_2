#include <aruco_det/pose_utils.h>
#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <limits>

using namespace aruco_det_core;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Calibration, StandardAndLegacyAgree) {
  const auto standard = loadCalibration(std::string(TEST_DATA_DIR) + "/camera.yaml", 1, 1);
  const auto legacy = loadCalibration(std::string(TEST_DATA_DIR) + "/legacy_camera.yaml", 640, 480);
  EXPECT_EQ(standard.width, 640);
  EXPECT_EQ(standard.height, 480);
  EXPECT_EQ(cv::norm(standard.k - legacy.k), 0);
  EXPECT_EQ(cv::norm(standard.d - legacy.d), 0);
}

TEST(Calibration, RejectInvalidData) {
  auto c = loadCalibration(std::string(TEST_DATA_DIR) + "/camera.yaml", 640, 480);
  c.k.at<double>(0, 0) = 0;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c.k.at<double>(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.validate(), std::runtime_error);
  c.k.at<double>(0, 0) = 800;
  c.width = 0;
  EXPECT_THROW(c.validate(), std::runtime_error);
  c.width = 640;
  c.d = cv::Mat::zeros(3, 1, CV_64F);
  EXPECT_THROW(c.validate(), std::runtime_error);
  EXPECT_THROW(checkDistortionModel("equidistant"), std::runtime_error);
  EXPECT_THROW(loadCalibration("/does/not/exist.yaml", 640, 480), YAML::Exception);
}

TEST(Pose, Exactly180DegreesIsFiniteAndInvertible) {
  const auto transform = toTransform(cv::Vec3d(CV_PI, 0, 0), cv::Vec3d(0.1, 0.2, 0.8));
  const auto pose = toPose(transform);
  EXPECT_TRUE(std::isfinite(pose.orientation.w));
  EXPECT_NEAR(std::abs(pose.orientation.x), 1, 1e-12);
  EXPECT_NEAR(transform.getRotation().length2(), 1, 1e-12);
  const auto inverse = toPose(transform.inverse());
  EXPECT_NEAR(inverse.position.x, -0.1, 1e-12);
  EXPECT_NEAR(inverse.position.y, 0.2, 1e-12);
  EXPECT_NEAR(inverse.position.z, 0.8, 1e-12);
  EXPECT_NEAR((transform * transform.inverse()).getOrigin().length(), 0, 1e-12);
}

TEST(Pose, SyntheticMultiMarkerHasCorrectIdentityAndMetricScale) {
  auto dict = cv::aruco::getPredefinedDictionary(10);
  cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));
  cv::Mat marker;
  cv::aruco::generateImageMarker(dict, 0, 80, marker);
  marker.copyTo(image(cv::Rect(280, 200, 80, 80)));
  cv::aruco::generateImageMarker(dict, 18, 80, marker);
  // Off-centre in BOTH axes: X=-0.20 m (left), Y=+0.06 m (down).
  marker.copyTo(image(cv::Rect(80, 260, 80, 80)));
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  auto params = cv::aruco::DetectorParameters();
  params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  cv::aruco::ArucoDetector detector(dict, params);
  detector.detectMarkers(image, corners, ids);
  ASSERT_EQ(ids.size(), 2u);
  const auto c = loadCalibration(std::string(TEST_DATA_DIR) + "/camera.yaml", 640, 480);
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, 0.08, c.k, c.d, rvecs, tvecs);
  bool found_zero = false, found_eighteen = false;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const auto pose = toPose(toTransform(rvecs[i], tvecs[i]));
    EXPECT_NEAR(pose.position.z, 0.8, 0.02);
    EXPECT_TRUE(std::isfinite(pose.orientation.w));
    EXPECT_GT(std::abs(pose.orientation.x), 0.99);
    if (ids[i] == 0) {
      found_zero = true;
      EXPECT_NEAR(pose.position.x, 0, 0.005);
      EXPECT_NEAR(pose.position.y, 0, 0.005);
    } else if (ids[i] == 18) {
      found_eighteen = true;
      EXPECT_NEAR(pose.position.x, -0.2, 0.005);
      EXPECT_NEAR(pose.position.y, 0.06, 0.005);
    }
  }
  EXPECT_TRUE(found_zero && found_eighteen);
  image.setTo(255);
  cv::aruco::ArucoDetector detector2(dict, params);
  detector2.detectMarkers(image, corners, ids);
  EXPECT_TRUE(ids.empty());
}
