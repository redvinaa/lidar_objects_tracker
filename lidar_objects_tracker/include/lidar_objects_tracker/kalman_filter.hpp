/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__KALMAN_FILTER_HPP_
#define LIDAR_OBJECTS_TRACKER__KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace lidar_objects_tracker
{

/** @brief 2D Kalman Filter for tracking position and velocity in a plane
 * State vector: [x, y, vx, vy]
 * Measurement vector: [x, y]
 */
class KalmanFilter2D
{
public:
  /** @brief Initialize Kalman filter
   * @param x0 Initial state vector [x, y, vx, vy]
   * @param P0 Initial state covariance
   * @param pos_uncertainty Position measurement uncertainty (std, used for P0 and R)
   * @param vel_uncertainty Velocity process uncertainty (std, used for P0)
   * @param acc_uncertainty Acceleration process uncertainty (std, used for Q)
   */
  KalmanFilter2D(
    const Eigen::Vector4f & x0,
    float pos_uncertainty,
    float vel_uncertainty,
    float acc_uncertainty)
  : state(x_), covariance(P_)
  {
    float pos_var = pos_uncertainty * pos_uncertainty;
    float vel_var = vel_uncertainty * vel_uncertainty;
    acc_var_ = acc_uncertainty * acc_uncertainty;

    // Initialize measurement noise covariance
    R_ <<
      pos_var, 0.0f,
      0.0f, pos_var;

    // Initialize measurement matrix
    H_ <<
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f;

    x_ = x0;
    P_ = Eigen::Matrix4f::Zero();
    P_(0, 0) = pos_var;
    P_(1, 1) = pos_var;
    P_(2, 2) = vel_var;
    P_(3, 3) = vel_var;
  }

  /** @brief Predict the next state
   * @param dt Time step
   */
  void predict(float dt)
  {
    const Eigen::Matrix4f F = getF(dt);

    // Predict state
    x_ = F * x_;

    // Predict covariance
    P_ = F * P_ * F.transpose() + getQ(dt);
  }

  /** @brief Update the state with a new measurement
   * @param z Measurement vector [x, y]
   * @return Squared mahalanobis distance of the measurement residual
   */
  float update(const Eigen::Vector2f & z)
  {
    // Measurement residual
    Eigen::Vector2f y = z - H_ * x_;

    // Residual covariance
    Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix2f S_inv = S.inverse();

    // Kalman gain
    Eigen::Matrix<float, 4, 2> K = P_ * H_.transpose() * S_inv;

    // Update state
    x_ = x_ + K * y;

    // Update covariance
    // P_ = (I - K * H_) * P_;
    // Joseph form (more numerically stable)
    static const Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    P_ = (I - K * H_) * P_ * (I - K * H_).transpose() +
      K * R_ * K.transpose();

    // Mahalanobis distance
    float mahalanobis_dist2 = y.transpose() * S_inv * y;
    return mahalanobis_dist2;
  }

  /** @brief Calculate squared Mahalanobis distance for a given measurement 
   * without updating the state
   * @param z Measurement vector [x, y]
   * @return Squared Mahalanobis distance
   */
  float mahalanobisDistance2(const Eigen::Vector2f & z) const
  {
    Eigen::Vector2f y = z - H_ * x_;
    Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix2f S_inv = S.inverse();
    float mahalanobis_dist2 = y.transpose() * S_inv * y;
    return mahalanobis_dist2;
  }

  Eigen::Vector4f & state;  // Mirror of x_
  Eigen::Matrix4f & covariance;  // Mirror of P_

private:
  Eigen::Vector4f x_;  // [x, y, vx, vy]
  Eigen::Matrix4f P_;  // State covariance
  Eigen::Matrix2f R_;  // Measurement noise covariance
  Eigen::Matrix<float, 2, 4> H_;  // Measurement matrix

  float acc_var_;  // Acceleration variance for process noise

  inline Eigen::Matrix4f getF(float dt)
  {
    Eigen::Matrix4f F;
    F <<
      1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;
    return F;
  }

  inline Eigen::Matrix4f getQ(float dt)
  {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    Eigen::Matrix4f Q;
    Q <<
      dt4 / 4, 0, dt3 / 2, 0,
      0, dt4 / 4, 0, dt3 / 2,
      dt3 / 2, 0, dt2, 0,
      0, dt3 / 2, 0, dt2;
    Q *= acc_var_;
    return Q;
  }
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__KALMAN_FILTER_HPP_
