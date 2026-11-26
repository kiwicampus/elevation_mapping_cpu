/*
 * SensorProcessorFrustumTest.cpp
 */

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// gtest
#include <gtest/gtest.h>

using elevation_mapping::SensorProcessorBase;

namespace {

// Exposes readParameters(); the sensor model is not under test.
class TestSensorProcessor : public SensorProcessorBase {
 public:
  using SensorProcessorBase::readParameters;
  using SensorProcessorBase::SensorProcessorBase;

 private:
  bool computeVariances(const elevation_mapping::PointCloudType::ConstPtr /*pointCloud*/,
                        const Eigen::Matrix<double, 6, 6>& /*robotPoseCovariance*/, Eigen::VectorXf& /*variances*/) override {
    return true;
  }
};

class SensorProcessorFrustumTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Reads the parameters of input "cam", whose frustum has the given model type and a valid depth camera geometry.
  std::unique_ptr<TestSensorProcessor> configure(const std::string& modelType) {
    const std::string prefix = "cam.sensor_processor.frustum.";
    rclcpp::NodeOptions options;
    options.parameter_overrides({{prefix + "model_type", modelType},
                                 {prefix + "horizontal_fov_angle", 1.2},
                                 {prefix + "vertical_fov_angle", 0.72},
                                 {prefix + "min_range", 0.2},
                                 {prefix + "max_range", 10.0}});
    node_ = std::make_shared<rclcpp::Node>("sensor_processor_frustum_test", options);
    auto processor = std::make_unique<TestSensorProcessor>(node_, SensorProcessorBase::GeneralParameters());
    std::string inputSourceName = "cam";
    EXPECT_TRUE(processor->readParameters(inputSourceName));
    return processor;
  }

  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace

TEST_F(SensorProcessorFrustumTest, ValidFrustumIsBuilt) {  // NOLINT
  EXPECT_NE(configure("depth_camera")->frustum(), nullptr);
}

TEST_F(SensorProcessorFrustumTest, InvalidFrustumKeepsTheInput) {  // NOLINT
  EXPECT_EQ(configure("not_a_model")->frustum(), nullptr);
}
