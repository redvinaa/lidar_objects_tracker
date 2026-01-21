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
#include <set>
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

struct UpdateInfo
{
  // Global update info
  float dt;
  std::set<uint32_t> births;
  std::set<uint32_t> deaths;

  struct TrackUpdateInfo
  {
    std::map<size_t, float> measurement_weights;
  };

  // Per track update info
  std::map<uint32_t, TrackUpdateInfo> updates;
};

/** @brief LMB Tracker for 2D point measurements
 *
 * Note: uint32_t is used for IDs, size_t for indices
 */
class LMBTracker
{
public:
  LMBTracker(
    rclcpp::Clock::SharedPtr clock,
    float max_dt,
    float gate_threshold,
    float birth_existence_prob,
    float death_existence_prob,
    float survival_prob,
    float detection_prob,
    float kf_pos_uncertainty,
    float kf_vel_uncertainty,
    float kf_acc_uncertainty)
  : clock_(clock),
    last_update_time_(clock->now()),
    max_dt_(max_dt),
    gate_threshold_(gate_threshold),
    birth_existence_prob_(birth_existence_prob),
    death_existence_prob_(death_existence_prob),
    survival_prob_(survival_prob),
    detection_prob_(detection_prob),
    kf_pos_uncertainty_(kf_pos_uncertainty),
    kf_vel_uncertainty_(kf_vel_uncertainty),
    kf_acc_uncertainty_(kf_acc_uncertainty)
  {}

  UpdateInfo updateTracks(const std::vector<Eigen::Vector2f> & measurements)
  {
    UpdateInfo update_info;

    // Used for births
    std::set<size_t> used_measurements;

    // Get dt since last update
    const rclcpp::Time current_time = clock_->now();
    const float dt = (current_time - last_update_time_).seconds();
    update_info.dt = dt;
    last_update_time_ = current_time;
    if (dt < 1e-6 || dt > max_dt_) {
      std::stringstream ss;
      ss << "Unrealistic dt (" << dt << " s), skipping prediction step.";
      throw std::runtime_error(ss.str());
    }
    RCLCPP_DEBUG(logger_, "===============================");
    RCLCPP_DEBUG(logger_, "Updating tracks with dt: %.3f s", dt);

    for (auto & [id, track] : tracks_) {
      update_info.updates.emplace(id, UpdateInfo::TrackUpdateInfo{});

      // Predict kalman filter and update existence probability
      track.kf->predict(dt);
      auto & r = track.existence_probability;
      r = std::clamp(r * survival_prob_, 0.0f, 1.0f);  // Clamp to [0, 1]

      RCLCPP_DEBUG(
        logger_, "Track ID: %u, Existence Probability: %.3f",
        id, track.existence_probability);

      // Gate measurements
      std::set<size_t> gated_indices;
      std::stringstream ss;
      ss << "Using measurements: ";
      for (size_t i = 0; i < measurements.size(); ++i) {
        const auto & measurement = measurements[i];
        const float dist2 = track.kf->mahalanobisDistance2(measurement);
        if (dist2 < gate_threshold_) {
          gated_indices.insert(i);
          used_measurements.insert(i);
          ss << i << " ";
        }
      }
      RCLCPP_DEBUG(logger_, "%s", ss.str().c_str());

      // If no measurements, missed detection
      if (gated_indices.empty()) {
        track.existence_probability *= 1.0f - detection_prob_;

        RCLCPP_DEBUG(
          logger_, "Missed detection for Track ID: %u, New Existence Probability: %.3f",
          id, track.existence_probability);

        continue;
      }

      // Update with all gated measurements (simplified, equal weights)
      // TODO(redvinaa): Implement proper weighting
      Eigen::Vector2f combined_measurement = Eigen::Vector2f::Zero();
      const float w = 1.0f / static_cast<float>(gated_indices.size());
      for (const auto & meas_idx : gated_indices) {
        combined_measurement += measurements[meas_idx] / w;
        update_info.updates[id].measurement_weights[meas_idx] = w;
      }
      track.kf->update(combined_measurement);

      // Update existence probability
      r = 1 - (1 - r) * (1 - detection_prob_);

      RCLCPP_DEBUG(
        logger_, "Updated Track ID: %u, New Existence Probability: %.3f",
        id, track.existence_probability);
    }

    // Deaths
    std::vector<uint32_t> tracks_to_erase;
    for (const auto & [id, track] : tracks_) {
      if (track.existence_probability < death_existence_prob_) {
        tracks_to_erase.push_back(id);
        update_info.deaths.insert(id);
        RCLCPP_DEBUG(logger_, "Deleting Track ID: %u due to low existence probability", id);
      }
    }
    for (const auto & id : tracks_to_erase) {
      tracks_.erase(id);
    }

    // Births
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (used_measurements.find(i) != used_measurements.end()) {
        continue;  // Measurement already used
      }

      // Create new track
      const auto & meas = measurements[i];
      Eigen::Vector4f x0;
      x0 << meas(0), meas(1), 0.0f, 0.0f;  // Initial velocity zero
      auto kf = std::make_shared<KalmanFilter2D>(
        x0,
        kf_pos_uncertainty_,
        kf_vel_uncertainty_,
        kf_acc_uncertainty_);

      // Assign new ID
      uint32_t new_id = 0;
      while (tracks_.find(new_id) != tracks_.end()) {
        ++new_id;
      }

      Track new_track;
      new_track.kf = kf;
      new_track.existence_probability = birth_existence_prob_;
      tracks_.emplace(new_id, std::move(new_track));

      update_info.births.insert(new_id);

      RCLCPP_DEBUG(
        logger_, "Created new Track ID: %u at position (%.2f, %.2f)",
        new_id, meas(0), meas(1));
    }

    return update_info;
  }

  inline const std::map<uint32_t, Track> & getTracks() const
  {
    return tracks_;
  }

private:
  std::map<uint32_t, Track> tracks_;
  rclcpp::Clock::SharedPtr clock_;  // To get dt
  rclcpp::Logger logger_ = rclcpp::get_logger("LMBTracker");

  float max_dt_;
  rclcpp::Time last_update_time_;
  float gate_threshold_;  // ~95% confidence
  float birth_existence_prob_;  // Keep low so multiple confirmations needed
  float death_existence_prob_;
  float survival_prob_;  // P(existing object survives next step)
  float detection_prob_;  // P(existing object is detected)
  float kf_pos_uncertainty_;  // for Kalman Filter initialization, m
  float kf_vel_uncertainty_;  // for Kalman Filter initialization, m/s
  float kf_acc_uncertainty_;  // for Kalman Filter initialization, m/s^2
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LMB_TRACKER_HPP_
