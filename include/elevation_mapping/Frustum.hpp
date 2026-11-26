/*
 * Frustum.hpp
 *
 * Sensor field-of-view models used to accelerate the decay of cells the sensor should see but did not
 * re-measure, following the spatio_temporal_voxel_layer (STVL) frustum models.
 */

#pragma once

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace elevation_mapping {

struct FrustumParameters {
  //! "depth_camera" or "lidar". Empty disables the frustum.
  std::string modelType;
  //! Horizontal field of view [rad]. Lidar: >= 2*pi means a full circle.
  double horizontalFov{0.0};
  //! Depth camera: symmetric vertical field of view [rad].
  double verticalFov{0.0};
  //! Lidar: lowest and highest elevation angle seen by the sensor [rad] (e.g. Livox Mid-360: -0.12, 0.91).
  double verticalFovStart{0.0};
  double verticalFovEnd{0.0};
  //! Depth camera: distance along the optical axis [m]. Lidar: horizontal distance [m].
  double minRange{0.0};
  double maxRange{0.0};
  //! STVL decay acceleration [1/s^2]: an unseen cell inside the frustum ages a*t^3/6 faster. 0 disables the faster
  //! decay; the frustum is still built, so it can be published for debugging.
  double decayAcceleration{0.0};
};

/**
 * @brief Field of view of one sensor, placed at the sensor pose of the last processed point cloud.
 */
class Frustum {
 public:
  using Ptr = std::unique_ptr<Frustum>;

  /**
   * @brief Builds the model named in parameters.modelType.
   * @return nullptr when no model is configured.
   * @throws std::invalid_argument for an unknown model or an invalid field of view.
   */
  static Ptr create(const FrustumParameters& parameters);

  virtual ~Frustum() = default;

  //! Places the frustum at the sensor pose given as the sensor-to-map transformation.
  void setPose(const Eigen::Affine3d& sensorToMap) {
    sensorToMap_ = sensorToMap;
    mapToSensor_ = sensorToMap.inverse();
  }

  //! True if a point in the map frame lies inside the sensor field of view (occlusions are ignored, as in STVL).
  bool isInside(const Eigen::Vector3d& pointInMap) const { return isInsideSensorFrame(mapToSensor_ * pointInMap); }

  //! Edges of the field of view in the map frame, as consecutive point pairs (for visualization).
  std::vector<Eigen::Vector3d> outlineInMap() const;

  double decayAcceleration() const { return decayAcceleration_; }

  //! False when the frustum is only built for visualization (decay_acceleration 0).
  bool acceleratesDecay() const { return decayAcceleration_ > 0.0; }

 protected:
  //! @throws std::invalid_argument for an empty horizontal field of view or max_range <= min_range.
  explicit Frustum(const FrustumParameters& parameters);

  virtual bool isInsideSensorFrame(const Eigen::Vector3d& point) const = 0;

  //! Edges of the field of view in the sensor frame, as consecutive point pairs.
  virtual std::vector<Eigen::Vector3d> outline() const = 0;

 private:
  double decayAcceleration_;
  Eigen::Affine3d sensorToMap_{Eigen::Affine3d::Identity()};
  Eigen::Affine3d mapToSensor_{Eigen::Affine3d::Identity()};
};

/**
 * @brief Pinhole frustum of a depth camera in its optical frame (z forward, x right, y down).
 */
class DepthCameraFrustum : public Frustum {
 public:
  //! @throws std::invalid_argument unless both fields of view are in (0, pi), plus the Frustum checks.
  explicit DepthCameraFrustum(const FrustumParameters& parameters);

 protected:
  bool isInsideSensorFrame(const Eigen::Vector3d& point) const override;
  std::vector<Eigen::Vector3d> outline() const override;

 private:
  double tanHalfHorizontalFov_;
  double tanHalfVerticalFov_;
  double minRange_;
  double maxRange_;
};

/**
 * @brief Frustum of a rotating 3D lidar in its frame (x forward, z up), with an asymmetric vertical field of view.
 */
class LidarFrustum : public Frustum {
 public:
  //! @throws std::invalid_argument unless vertical_fov_end_angle > vertical_fov_start_angle, plus the Frustum checks.
  explicit LidarFrustum(const FrustumParameters& parameters);

 protected:
  bool isInsideSensorFrame(const Eigen::Vector3d& point) const override;
  std::vector<Eigen::Vector3d> outline() const override;

 private:
  double halfHorizontalFov_;
  bool fullCircle_;
  double verticalFovStart_;
  double verticalFovEnd_;
  double minRange_;
  double maxRange_;
};

}  // namespace elevation_mapping
