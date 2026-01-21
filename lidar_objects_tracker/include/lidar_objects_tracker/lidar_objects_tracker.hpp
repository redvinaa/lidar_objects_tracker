/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#ifndef LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_
#define LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_

#include <open3d/geometry/PointCloud.h>

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lidar_objects_tracker/lmb_tracker.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lidar_objects_tracker_msgs/msg/tracked_object.hpp"
#include "lidar_objects_tracker_msgs/msg/tracked_objects.hpp"

namespace lidar_objects_tracker
{

class ObjectsTracker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ObjectsTracker(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&ObjectsTracker::scanCallback, this, std::placeholders::_1));

    tracked_objects_pub_->on_activate();
    marker_pub_->on_activate();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    tracked_objects_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);

  static open3d::geometry::PointCloud laserScanToPointCloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  std::vector<open3d::geometry::PointCloud> segment(
    const open3d::geometry::PointCloud & pc) const;
  static Eigen::Vector2f calculateCentroid(const open3d::geometry::PointCloud & cluster);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  // TODO(redvinaa): publish static and dynamic scans?

  rclcpp_lifecycle::LifecyclePublisher<lidar_objects_tracker_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;

  std::unique_ptr<LMBTracker> tracker_;

  // Parameters
  double cluster_neighbor_radius_;
  size_t cluster_min_points_;
  size_t cluster_max_points_;
  bool visualize_;
};

}  // namespace lidar_objects_tracker
#endif  // LIDAR_OBJECTS_TRACKER__LIDAR_OBJECTS_TRACKER_HPP_
