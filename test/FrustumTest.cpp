/*
 * FrustumTest.cpp
 */

#include "elevation_mapping/Frustum.hpp"

#include <cmath>
#include <stdexcept>

// gtest
#include <gtest/gtest.h>

using elevation_mapping::Frustum;
using elevation_mapping::FrustumParameters;

namespace {

FrustumParameters depthCamera() {
  FrustumParameters parameters;
  parameters.modelType = "depth_camera";
  parameters.horizontalFov = M_PI / 2.0;
  parameters.verticalFov = M_PI / 3.0;
  parameters.minRange = 0.2;
  parameters.maxRange = 5.0;
  parameters.decayAcceleration = 5.0;
  return parameters;
}

FrustumParameters livox() {
  FrustumParameters parameters;
  parameters.modelType = "lidar";
  parameters.horizontalFov = 2.0 * M_PI;
  parameters.verticalFovStart = -0.12;
  parameters.verticalFovEnd = 0.91;
  parameters.minRange = 0.1;
  parameters.maxRange = 20.0;
  parameters.decayAcceleration = 20.0;
  return parameters;
}

//! Optical frame (z forward, x right, y down) at (1, 2, 0.5) looking along map +x.
Eigen::Affine3d cameraLookingAlongX() {
  Eigen::Affine3d sensorToMap = Eigen::Affine3d::Identity();
  sensorToMap.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  sensorToMap.translation() << 1.0, 2.0, 0.5;
  return sensorToMap;
}

}  // namespace

TEST(Frustum, DisabledWithoutModel) {  // NOLINT
  EXPECT_EQ(Frustum::create(FrustumParameters()), nullptr);
  // Without acceleration the frustum is still built, for visualization.
  FrustumParameters parameters = depthCamera();
  parameters.decayAcceleration = 0.0;
  EXPECT_NE(Frustum::create(parameters), nullptr);
}

TEST(Frustum, RejectsInvalidParameters) {  // NOLINT
  FrustumParameters parameters = depthCamera();
  parameters.modelType = "sonar";
  EXPECT_THROW(Frustum::create(parameters), std::invalid_argument);
  parameters = depthCamera();
  parameters.verticalFov = 0.0;
  EXPECT_THROW(Frustum::create(parameters), std::invalid_argument);
  parameters = livox();
  parameters.verticalFovEnd = parameters.verticalFovStart;
  EXPECT_THROW(Frustum::create(parameters), std::invalid_argument);
}

TEST(Frustum, ConstructorsRejectInvalidParameters) {  // NOLINT
  FrustumParameters parameters = depthCamera();
  parameters.horizontalFov = M_PI;
  EXPECT_THROW(elevation_mapping::DepthCameraFrustum{parameters}, std::invalid_argument);
  parameters = depthCamera();
  parameters.maxRange = parameters.minRange;
  EXPECT_THROW(elevation_mapping::DepthCameraFrustum{parameters}, std::invalid_argument);
  parameters = livox();
  parameters.verticalFovEnd = parameters.verticalFovStart;
  EXPECT_THROW(elevation_mapping::LidarFrustum{parameters}, std::invalid_argument);
}

TEST(DepthCameraFrustum, FollowsTheSensorPose) {  // NOLINT
  const Frustum::Ptr frustum = Frustum::create(depthCamera());
  ASSERT_NE(frustum, nullptr);
  frustum->setPose(cameraLookingAlongX());
  EXPECT_TRUE(frustum->isInside({3.0, 2.0, 0.5}));    // 2 m straight ahead
  EXPECT_TRUE(frustum->isInside({2.0, 2.9, 0.5}));    // 42 deg left, inside +-45
  EXPECT_FALSE(frustum->isInside({2.0, 3.5, 0.5}));   // 56 deg left
  EXPECT_FALSE(frustum->isInside({2.0, 2.0, 1.2}));   // 35 deg up, outside +-30
  EXPECT_FALSE(frustum->isInside({-1.0, 2.0, 0.5}));  // behind
  EXPECT_FALSE(frustum->isInside({1.1, 2.0, 0.5}));   // closer than min range
  EXPECT_FALSE(frustum->isInside({7.0, 2.0, 0.5}));   // beyond max range
}

TEST(DepthCameraFrustum, OutlineMatchesIsInside) {  // NOLINT
  const Frustum::Ptr frustum = Frustum::create(depthCamera());
  ASSERT_NE(frustum, nullptr);
  frustum->setPose(cameraLookingAlongX());
  const std::vector<Eigen::Vector3d> outline = frustum->outlineInMap();
  ASSERT_EQ(outline.size(), 24U);  // 12 edges
  // A point on the axis, half way between the near and far planes (optical axis = map +x from (1, 2, 0.5)).
  const Eigen::Vector3d center(1.0 + (0.2 + 5.0) / 2.0, 2.0, 0.5);
  for (const Eigen::Vector3d& corner : outline) {
    EXPECT_TRUE(frustum->isInside(corner + 0.01 * (center - corner)));   // just inside the drawn corner
    EXPECT_FALSE(frustum->isInside(corner - 0.01 * (center - corner)));  // just outside it
  }
}

TEST(LidarFrustum, FullCircleWithAsymmetricVerticalFov) {  // NOLINT
  const Frustum::Ptr frustum = Frustum::create(livox());
  ASSERT_NE(frustum, nullptr);
  Eigen::Affine3d sensorToMap = Eigen::Affine3d::Identity();
  sensorToMap.translation() << 0.0, 0.0, 1.0;
  frustum->setPose(sensorToMap);
  EXPECT_TRUE(frustum->isInside({-3.0, 0.0, 1.0}));   // behind, level
  EXPECT_TRUE(frustum->isInside({0.0, 3.0, 1.5}));    // left, 9 deg up
  EXPECT_FALSE(frustum->isInside({1.0, 0.0, 0.0}));   // 45 deg down, below the -7 deg edge
  EXPECT_FALSE(frustum->isInside({0.5, 0.0, 3.0}));   // 76 deg up, above the 52 deg edge
  EXPECT_FALSE(frustum->isInside({25.0, 0.0, 1.0}));  // beyond max range
}

TEST(LidarFrustum, LimitedHorizontalFov) {  // NOLINT
  FrustumParameters parameters = livox();
  parameters.horizontalFov = M_PI / 2.0;
  const Frustum::Ptr frustum = Frustum::create(parameters);
  ASSERT_NE(frustum, nullptr);
  EXPECT_TRUE(frustum->isInside({3.0, 1.0, 0.0}));    // 18 deg left
  EXPECT_FALSE(frustum->isInside({3.0, 4.0, 0.0}));   // 53 deg left
  EXPECT_FALSE(frustum->isInside({-3.0, 0.0, 0.0}));  // behind
}
