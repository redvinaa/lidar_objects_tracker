/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_
#define LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_

#include <vector>
#include <map>
#include <memory>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "lidar_objects_tracker/kalman_filter.hpp"

namespace lidar_objects_tracker
{

struct Track
{
  std::shared_ptr<KalmanFilter2D> kf;
  float existence_probability;
};

class LMBTracker
{
public:
  explicit LMBTracker(rclcpp::Clock::SharedPtr clock)
  : clock_(clock),
    last_update_time_(clock->now())
  {}

  void updateTracks(const std::vector<Eigen::Vector2f> & measurements)
  {
    predictTracks();

  }

  inline const std::map<uint32_t, Track> & getTracks() const
  {
    return tracks_;
  }

private:
  /// @brief Predict all existing tracks
  void predictTracks()
  {
    const rclcpp::Time current_time = clock_->now();
    const float dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    if (dt < 1e-6 || dt > 10.0) {
      RCLCPP_WARN(
        logger_,
        "Unrealistic dt for prediction: %.3f s. Skipping prediction step.", dt);
      return;
    }

    for (auto & [id, track] : tracks_) {
      track.kf->predict(dt);
    }
  }

  std::map<uint32_t, Track> tracks_;
  rclcpp::Clock::SharedPtr clock_;  // To get dt
  rclcpp::Logger logger_ = rclcpp::get_logger("JPDATracker");

  float max_dt_;
  rclcpp::Time last_update_time_;
  float gating_mahal2_dist_ = 6.0f;  // ~95% confidence
  float birth_existence_prob_ = 0.5f;
  float death_existence_prob_ = 0.05f;
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_
