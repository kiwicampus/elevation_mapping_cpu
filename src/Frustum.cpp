/*
 * Frustum.cpp
 */

#include "elevation_mapping/Frustum.hpp"

#include <cmath>
#include <stdexcept>

namespace elevation_mapping {

Frustum::Ptr Frustum::create(const FrustumParameters& parameters) {
  if (parameters.modelType.empty()) {
    return nullptr;
  }
  if (parameters.horizontalFov <= 0.0 || parameters.maxRange <= parameters.minRange) {
    throw std::invalid_argument("horizontal_fov_angle must be > 0 and max_range > min_range");
  }
  if (parameters.modelType == "depth_camera") {
    if (parameters.verticalFov <= 0.0 || parameters.horizontalFov >= M_PI || parameters.verticalFov >= M_PI) {
      throw std::invalid_argument("depth_camera needs 0 < vertical_fov_angle, horizontal_fov_angle < pi");
    }
    return std::make_unique<DepthCameraFrustum>(parameters);
  }
  if (parameters.modelType == "lidar") {
    if (parameters.verticalFovEnd <= parameters.verticalFovStart) {
      throw std::invalid_argument("lidar needs vertical_fov_end_angle > vertical_fov_start_angle");
    }
    return std::make_unique<LidarFrustum>(parameters);
  }
  throw std::invalid_argument("unknown model_type '" + parameters.modelType + "' (expected depth_camera or lidar)");
}

std::vector<Eigen::Vector3d> Frustum::outlineInMap() const {
  std::vector<Eigen::Vector3d> points = outline();
  for (Eigen::Vector3d& point : points) {
    point = sensorToMap_ * point;
  }
  return points;
}

DepthCameraFrustum::DepthCameraFrustum(const FrustumParameters& parameters)
    : Frustum(parameters.decayAcceleration),
      tanHalfHorizontalFov_(std::tan(parameters.horizontalFov / 2.0)),
      tanHalfVerticalFov_(std::tan(parameters.verticalFov / 2.0)),
      minRange_(parameters.minRange),
      maxRange_(parameters.maxRange) {}

bool DepthCameraFrustum::isInsideSensorFrame(const Eigen::Vector3d& point) const {
  const double depth = point.z();
  return depth >= minRange_ && depth <= maxRange_ && std::abs(point.x()) <= depth * tanHalfHorizontalFov_ &&
         std::abs(point.y()) <= depth * tanHalfVerticalFov_;
}

std::vector<Eigen::Vector3d> DepthCameraFrustum::outline() const {
  // Near and far rectangles, corners in the same order, then the four edges joining them.
  std::vector<Eigen::Vector3d> corners;
  for (const double depth : {minRange_, maxRange_}) {
    const double x = depth * tanHalfHorizontalFov_;
    const double y = depth * tanHalfVerticalFov_;
    corners.insert(corners.end(), {{-x, -y, depth}, {x, -y, depth}, {x, y, depth}, {-x, y, depth}});
  }
  std::vector<Eigen::Vector3d> lines;
  for (size_t i = 0; i < 4; ++i) {
    const size_t next = (i + 1) % 4;
    lines.insert(lines.end(), {corners[i], corners[next], corners[i + 4], corners[next + 4], corners[i], corners[i + 4]});
  }
  return lines;
}

LidarFrustum::LidarFrustum(const FrustumParameters& parameters)
    : Frustum(parameters.decayAcceleration),
      halfHorizontalFov_(parameters.horizontalFov / 2.0),
      fullCircle_(parameters.horizontalFov >= 2.0 * M_PI - 1e-3),
      verticalFovStart_(parameters.verticalFovStart),
      verticalFovEnd_(parameters.verticalFovEnd),
      minRange_(parameters.minRange),
      maxRange_(parameters.maxRange) {}

bool LidarFrustum::isInsideSensorFrame(const Eigen::Vector3d& point) const {
  const double range = std::hypot(point.x(), point.y());
  if (range < minRange_ || range > maxRange_) {
    return false;
  }
  const double elevationAngle = std::atan2(point.z(), range);
  if (elevationAngle < verticalFovStart_ || elevationAngle > verticalFovEnd_) {
    return false;
  }
  return fullCircle_ || std::abs(std::atan2(point.y(), point.x())) <= halfHorizontalFov_;
}

std::vector<Eigen::Vector3d> LidarFrustum::outline() const {
  // Arcs at both ranges on the lower and upper edge, plus a radial edge every few segments.
  constexpr int kSegments = 32;
  constexpr int kSegmentsPerRay = 4;
  const double firstAzimuth = fullCircle_ ? -M_PI : -halfHorizontalFov_;
  const double azimuthSpan = fullCircle_ ? 2.0 * M_PI : 2.0 * halfHorizontalFov_;
  const auto point = [](double range, double azimuth, double elevationAngle) {
    return Eigen::Vector3d(range * std::cos(azimuth), range * std::sin(azimuth), range * std::tan(elevationAngle));
  };
  std::vector<Eigen::Vector3d> lines;
  for (int i = 0; i <= kSegments; ++i) {
    const double azimuth = firstAzimuth + azimuthSpan * i / kSegments;
    const double nextAzimuth = firstAzimuth + azimuthSpan * (i + 1) / kSegments;
    for (const double elevationAngle : {verticalFovStart_, verticalFovEnd_}) {
      if (i % kSegmentsPerRay == 0) {
        lines.insert(lines.end(), {point(minRange_, azimuth, elevationAngle), point(maxRange_, azimuth, elevationAngle)});
      }
      if (i == kSegments) {
        continue;
      }
      for (const double range : {minRange_, maxRange_}) {
        lines.insert(lines.end(), {point(range, azimuth, elevationAngle), point(range, nextAzimuth, elevationAngle)});
      }
    }
  }
  return lines;
}

}  // namespace elevation_mapping
