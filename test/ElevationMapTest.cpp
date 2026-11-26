/*
 * ElevationMapTest.cpp
 *
 *  Created on: Nov 27, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/ElevationMap.hpp"
#include "grid_map_core/GridMap.hpp"

#include <rclcpp/rclcpp.hpp>

// gtest
#include <gtest/gtest.h>

using elevation_mapping::ElevationMap;
using elevation_mapping::PointCloudType;

class ElevationMapDecayTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("elevation_map_test");
    node_->declare_parameter("postprocessor_num_threads", 0);
    map_ = std::make_unique<ElevationMap>(node_);
    map_->setGeometry(grid_map::Length(1.0, 1.0), 0.1, grid_map::Position(0.0, 0.0));
    // The map counts cell times from its construction, so stamps are offsets from now.
    start_ = node_->get_clock()->now();
  }

  void addPoint(double x, double y, float z, double stampSeconds) { addPoints(x, y, {z}, stampSeconds); }

  // One cloud with a point per height, all in the same cell, added in order.
  void addPoints(double x, double y, const std::vector<float>& heights, double stampSeconds) {
    PointCloudType::Ptr cloud(new PointCloudType);
    for (const float z : heights) {
      pcl::PointXYZRGBConfidenceRatio point;
      point.x = x;
      point.y = y;
      point.z = z;
      cloud->push_back(point);
    }
    Eigen::VectorXf variances = Eigen::VectorXf::Constant(heights.size(), kPointVariance);
    ASSERT_TRUE(map_->add(cloud, variances, stamp(stampSeconds), Eigen::Affine3d::Identity()));
  }

  static constexpr float kPointVariance = 0.0001;

  bool isValid(double x, double y) { return map_->getRawGridMap().isValid(index(x, y)); }

  grid_map::Index index(double x, double y) {
    grid_map::Index index;
    map_->getRawGridMap().getIndex(grid_map::Position(x, y), index);
    return index;
  }

  rclcpp::Time stamp(double seconds) { return start_ + rclcpp::Duration::from_seconds(seconds); }

  rclcpp::Time start_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<ElevationMap> map_;
};

TEST_F(ElevationMapDecayTest, CellSeenOnceExpires) {  // NOLINT
  addPoint(0.05, 0.05, 0.1, 10.0);
  map_->decay(stamp(12.0), 3.0);
  EXPECT_TRUE(isValid(0.05, 0.05));
  map_->decay(stamp(13.5), 3.0);
  EXPECT_FALSE(isValid(0.05, 0.05));
}

// Stamping a new cell's first point also makes later points of the same cloud count as the same scan: the higher one
// wins, instead of keeping the first and adding multi-height noise to its variance.
TEST_F(ElevationMapDecayTest, NewCellKeepsHigherPointOfOneCloud) {  // NOLINT
  addPoints(0.05, 0.05, {0.0, 0.5}, 10.0);
  addPoints(0.35, 0.35, {0.5, 0.0}, 10.0);
  const grid_map::GridMap& map = map_->getRawGridMap();
  for (const auto& cell : {index(0.05, 0.05), index(0.35, 0.35)}) {
    EXPECT_FLOAT_EQ(map.at("elevation", cell), 0.5);
    EXPECT_FLOAT_EQ(map.at("variance", cell), kPointVariance);
  }
}

TEST_F(ElevationMapDecayTest, RefreshedCellIsKept) {  // NOLINT
  addPoint(0.05, 0.05, 0.1, 10.0);
  addPoint(0.35, 0.35, 0.1, 10.0);
  addPoint(0.35, 0.35, 0.1, 12.0);
  map_->decay(stamp(14.0), 3.0);
  EXPECT_FALSE(isValid(0.05, 0.05));
  EXPECT_TRUE(isValid(0.35, 0.35));
}

// Narrow camera at (-1, 0.35, 0.1) looking along +x: sees the cell at (0.35, 0.35), not the one at (0.35, -0.35).
class ElevationMapFrustumDecayTest : public ElevationMapDecayTest {
 protected:
  void SetUp() override {
    ElevationMapDecayTest::SetUp();
    frustum_ = makeFrustum(5.0);
  }

  static elevation_mapping::Frustum::Ptr makeFrustum(double decayAcceleration) {
    elevation_mapping::FrustumParameters parameters;
    parameters.modelType = "depth_camera";
    parameters.horizontalFov = 0.2;
    parameters.verticalFov = 0.2;
    parameters.minRange = 0.2;
    parameters.maxRange = 5.0;
    parameters.decayAcceleration = decayAcceleration;
    elevation_mapping::Frustum::Ptr frustum = elevation_mapping::Frustum::create(parameters);
    Eigen::Affine3d sensorToMap = Eigen::Affine3d::Identity();
    sensorToMap.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    sensorToMap.translation() << -1.0, 0.35, 0.1;
    frustum->setPose(sensorToMap);
    return frustum;
  }

  elevation_mapping::Frustum::Ptr frustum_;
};

TEST_F(ElevationMapFrustumDecayTest, ZeroAccelerationKeepsNominalDecay) {  // NOLINT
  const elevation_mapping::Frustum::Ptr visualizationOnly = makeFrustum(0.0);
  addPoint(0.35, 0.35, 0.1, 10.0);
  map_->decay(stamp(11.8), 5.0, visualizationOnly.get());
  EXPECT_TRUE(isValid(0.35, 0.35));
  map_->decay(stamp(15.5), 5.0, visualizationOnly.get());
  EXPECT_FALSE(isValid(0.35, 0.35));
}

TEST_F(ElevationMapFrustumDecayTest, UnseenCellInsideFrustumExpiresFirst) {  // NOLINT
  addPoint(0.35, 0.35, 0.1, 10.0);
  addPoint(0.35, -0.35, 0.1, 10.0);
  // Age 1.8 s: 5 - 1.8 - 5 * 1.8^3 / 6 < 0 inside the frustum; 5 - 1.8 > 0 outside.
  map_->decay(stamp(11.8), 5.0, frustum_.get());
  EXPECT_FALSE(isValid(0.35, 0.35));
  EXPECT_TRUE(isValid(0.35, -0.35));
}

TEST_F(ElevationMapFrustumDecayTest, RefreshedCellInsideFrustumIsKept) {  // NOLINT
  addPoint(0.35, 0.35, 0.1, 10.0);
  addPoint(0.35, 0.35, 0.1, 11.75);
  map_->decay(stamp(11.8), 5.0, frustum_.get());
  EXPECT_TRUE(isValid(0.35, 0.35));
}

TEST_F(ElevationMapFrustumDecayTest, RepeatedMissesCompound) {  // NOLINT
  addPoint(0.35, 0.35, 0.1, 10.0);
  addPoint(0.35, -0.35, 0.1, 10.0);
  map_->decay(stamp(10.5), 5.0, frustum_.get());
  map_->decay(stamp(11.0), 5.0, frustum_.get());
  EXPECT_TRUE(isValid(0.35, 0.35));
  // A single pass at age 1.5 s keeps the cell (5 - 1.5 - 2.8 > 0); the age added by the earlier misses removes it.
  map_->decay(stamp(11.5), 5.0, frustum_.get());
  EXPECT_FALSE(isValid(0.35, 0.35));
  EXPECT_TRUE(isValid(0.35, -0.35));
}

TEST_F(ElevationMapFrustumDecayTest, OlderCloudDoesNotDelayDecay) {  // NOLINT
  addPoint(0.35, 0.35, 0.1, 10.0);
  // A cloud stamped before the cell was measured (out of order) must not move the cell time forward.
  map_->decay(stamp(8.0), 5.0, frustum_.get());
  map_->decay(stamp(11.8), 5.0, frustum_.get());
  EXPECT_FALSE(isValid(0.35, 0.35));
}

TEST_F(ElevationMapDecayTest, NeverMeasuredCellIsKept) {  // NOLINT
  addPoint(0.05, 0.05, 0.1, 10.0);
  // Like the initializer patch: height set without a measurement time.
  map_->getRawGridMap().at("elevation", index(-0.35, -0.35)) = 0.0;
  map_->getRawGridMap().at("variance", index(-0.35, -0.35)) = 0.0;
  map_->decay(stamp(100.0), 3.0);
  EXPECT_FALSE(isValid(0.05, 0.05));
  EXPECT_TRUE(isValid(-0.35, -0.35));
}
