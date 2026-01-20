/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#include <gtest/gtest.h>

#include "lidar_objects_tracker/kalman_filter.hpp"

class KalmanFilter2DTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// Test proper initialization of state and covariance
TEST_F(KalmanFilter2DTest, Initialization)
{
    const Eigen::Vector4f x0{1.0f, 1.0f, 0.0f, 0.0f};
    lidar_objects_tracker::KalmanFilter2D kf(x0);

    const Eigen::Vector4f x = kf.state;
    EXPECT_NEAR(x(0), 1.0f, 1e-6);
    EXPECT_NEAR(x(1), 1.0f, 1e-6);
    EXPECT_NEAR(x(2), 0.0f, 1e-6);
    EXPECT_NEAR(x(3), 0.0f, 1e-6);

    const Eigen::Matrix4f P = kf.covariance;
    EXPECT_NEAR(P(0, 0), 0.01f, 1e-6);
    EXPECT_NEAR(P(1, 1), 0.01f, 1e-6);
    EXPECT_NEAR(P(2, 2), 0.0625f, 1e-6);
    EXPECT_NEAR(P(3, 3), 0.0625f, 1e-6);
}

// Test prediction step with no external update
TEST_F(KalmanFilter2DTest, Predict)
{
    Eigen::Vector4f x0{1.0f, 1.0f, 0.0f, 0.0f};
    lidar_objects_tracker::KalmanFilter2D kf(x0);

    const Eigen::Matrix4f P0 = kf.covariance;
    kf.predict(1.0f);

    const Eigen::Vector4f x_pred = kf.state;
    EXPECT_NEAR(x_pred(0), 1.0f, 1e-6);
    EXPECT_NEAR(x_pred(1), 1.0f, 1e-6);
    EXPECT_NEAR(x_pred(2), 0.0f, 1e-6);
    EXPECT_NEAR(x_pred(3), 0.0f, 1e-6);

    const Eigen::Matrix4f P_pred = kf.covariance;
    EXPECT_GT(P_pred(0, 0), P0(0, 0));
    EXPECT_GT(P_pred(1, 1), P0(1, 1));
    EXPECT_GT(P_pred(2, 2), P0(2, 2));
    EXPECT_GT(P_pred(3, 3), P0(3, 3));
}

// Test prediction + update convergence
TEST_F(KalmanFilter2DTest, PredictAndUpdate)
{
    Eigen::Vector4f x0{0.0f, 0.0f, 1.0f, 1.0f};
    lidar_objects_tracker::KalmanFilter2D kf(x0);

    // Predict forward by 1 second
    kf.predict(1.0f);
    Eigen::Vector4f pred = kf.state;
    EXPECT_NEAR(pred(0), 1.0f, 1e-3);
    EXPECT_NEAR(pred(1), 1.0f, 1e-3);

    // Perform repeated updates with a noisy measurement
    Eigen::Vector2f z{1.2f, 0.9f};

    Eigen::Matrix4f P_before = kf.covariance;
    kf.update(z);
    Eigen::Vector4f x1 = kf.state;
    Eigen::Matrix4f P_after = kf.covariance;

    // Covariance should decrease after update
    EXPECT_LT(P_after(0, 0), P_before(0, 0));
    EXPECT_LT(P_after(1, 1), P_before(1, 1));

    // State should move toward measurement
    EXPECT_GT(x1(0), pred(0));
    EXPECT_LT(x1(1), pred(1));

    // Repeated update should converge closer to measurement
    kf.update(z);
    Eigen::Vector4f x2 = kf.state;

    EXPECT_NEAR(x2(0), z(0), 1e-2);
    EXPECT_NEAR(x2(1), z(1), 1e-2);

    // Changes should reduce after successive updates (convergence)
    Eigen::Vector4f delta = (x2 - x1).cwiseAbs();
    EXPECT_LT(delta(0), 1e-2);
    EXPECT_LT(delta(1), 1e-2);
}
