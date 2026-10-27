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

  void addPoint(double x, double y, float z, double stampSeconds) {
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::PointXYZRGBConfidenceRatio point;
    point.x = x;
    point.y = y;
    point.z = z;
    cloud->push_back(point);
    Eigen::VectorXf variances = Eigen::VectorXf::Constant(1, 0.0001);
    ASSERT_TRUE(map_->add(cloud, variances, stamp(stampSeconds), Eigen::Affine3d::Identity()));
  }

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

TEST_F(ElevationMapDecayTest, RefreshedCellIsKept) {  // NOLINT
  addPoint(0.05, 0.05, 0.1, 10.0);
  addPoint(0.35, 0.35, 0.1, 10.0);
  addPoint(0.35, 0.35, 0.1, 12.0);
  map_->decay(stamp(14.0), 3.0);
  EXPECT_FALSE(isValid(0.05, 0.05));
  EXPECT_TRUE(isValid(0.35, 0.35));
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
