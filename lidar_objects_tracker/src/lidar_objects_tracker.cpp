/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#include <optional>
#include "lidar_objects_tracker/lidar_objects_tracker.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <open3d/geometry/BoundingVolume.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_objects_tracker
{

ObjectsTracker::ObjectsTracker(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("objects_tracker", options)
{
  // Get parameters
  declare_parameter<std::string>("target_frame", "odom");
  target_frame_ = get_parameter("target_frame").as_string();

  declare_parameter<double>("cluster_neighbor_radius", 0.2);
  cluster_neighbor_radius_ = get_parameter("cluster_neighbor_radius").as_double();

  declare_parameter<int>("cluster_min_points", 6);
  cluster_min_points_ = get_parameter("cluster_min_points").as_int();

  declare_parameter<int>("cluster_max_points", 50);
  cluster_max_points_ = get_parameter("cluster_max_points").as_int();

  declare_parameter<double>("max_dt", 1.0);
  float max_dt = get_parameter("max_dt").as_double();

  declare_parameter<double>("gate_threshold", 6.0);
  float gate_threshold = get_parameter("gate_threshold").as_double();

  declare_parameter<double>("birth_existence_prob", 0.2);
  float birth_existence_prob = get_parameter("birth_existence_prob").as_double();

  declare_parameter<double>("death_existence_prob", 0.05);
  float death_existence_prob = get_parameter("death_existence_prob").as_double();

  declare_parameter<double>("survival_prob", 0.99);
  float survival_prob = get_parameter("survival_prob").as_double();

  declare_parameter<double>("detection_prob", 0.99);
  float detection_prob = get_parameter("detection_prob").as_double();

  declare_parameter<double>("kf_pos_uncertainty", 0.05);
  float kf_pos_uncertainty = get_parameter("kf_pos_uncertainty").as_double();

  declare_parameter<double>("kf_vel_uncertainty", 0.1);
  float kf_vel_uncertainty = get_parameter("kf_vel_uncertainty").as_double();

  declare_parameter<double>("kf_acc_uncertainty", 0.5);
  float kf_acc_uncertainty = get_parameter("kf_acc_uncertainty").as_double();

  declare_parameter<bool>("visualize", true);
  visualize_ = get_parameter("visualize").as_bool();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tracker_ = std::make_unique<LMBTracker>(
    this->get_clock(),
    max_dt,
    gate_threshold,
    birth_existence_prob,
    death_existence_prob,
    survival_prob,
    detection_prob,
    kf_pos_uncertainty,
    kf_vel_uncertainty,
    kf_acc_uncertainty);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    std::bind(&ObjectsTracker::scanCallback, this, std::placeholders::_1));

  tracked_objects_pub_ = create_publisher<lidar_objects_tracker_msgs::msg::TrackedObjects>(
    "tracked_objects", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "tracked_objects_markers", 10);
}

void ObjectsTracker::scanCallback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Node is not active. Skipping scan processing.");
    return;
  }

  // Convert LaserScan to 2D points
  open3d::geometry::PointCloud pc = laserScanToPointCloud(msg);

  {  // Transform
    geometry_msgs::msg::TransformStamped tf_target_frame;
    try {
      tf_target_frame = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "Could not transform %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }
    Eigen::Matrix4d transform = tf2::transformToEigen(tf_target_frame.transform).matrix();
    pc = pc.Transform(transform);
  }

  // Segment and calculate centroids
  const std::vector<open3d::geometry::PointCloud> clusters = segment(pc);
  std::vector<std::optional<open3d::geometry::AxisAlignedBoundingBox>> bboxes;
  std::vector<Eigen::Vector2f> centroids;
  for (const auto & cluster : clusters) {
    if (cluster_max_points_ > 0 &&
      cluster.points_.size() > static_cast<size_t>(cluster_max_points_))
    {
      continue;
    }

    centroids.push_back(calculateCentroid(cluster));
    try {
      bboxes.push_back(cluster.GetAxisAlignedBoundingBox());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not compute bounding box for cluster: %s", e.what());
      bboxes.push_back(std::nullopt);
    }
  }

  // Update tracker
  UpdateInfo track_update_info;
  try {
    track_update_info = tracker_->updateTracks(centroids);
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(get_logger(), "Tracker update failed: %s", e.what());
    return;
  }
  RCLCPP_DEBUG(
    get_logger(), "Track update: %zu births, %zu deaths, %zu total tracks",
    track_update_info.births.size(),
    track_update_info.deaths.size(),
    tracker_->getTracks().size());

  // Publish
  lidar_objects_tracker_msgs::msg::TrackedObjects tracked_objects_msg;
  const auto & tracks = tracker_->getTracks();
  tracked_objects_msg.objects.reserve(tracks.size());
  for (const auto & [id, track] : tracks) {
    lidar_objects_tracker_msgs::msg::TrackedObject tracked_object_msg;
    const Eigen::Vector4f & state = track.kf->state;
    tracked_object_msg.header.frame_id = target_frame_;
    tracked_object_msg.header.stamp = msg->header.stamp;
    tracked_object_msg.id = id;
    tracked_object_msg.position.x = state(0);
    tracked_object_msg.position.y = state(1);
    tracked_object_msg.velocity.x = state(2);
    tracked_object_msg.velocity.y = state(3);
    tracked_objects_msg.objects.push_back(tracked_object_msg);
  }
  tracked_objects_pub_->publish(tracked_objects_msg);

  // Visualize
  if (visualize_) {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    // Cluster centroids and bounding boxes
    visualization_msgs::msg::Marker marker_centroids;
    marker_centroids.header.frame_id = target_frame_;
    marker_centroids.header.stamp = msg->header.stamp;
    marker_centroids.ns = "centroids";
    marker_centroids.id = 0;
    marker_centroids.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_centroids.action = visualization_msgs::msg::Marker::ADD;
    marker_centroids.scale.x = 0.2;
    marker_centroids.scale.y = 0.2;
    marker_centroids.scale.z = 0.01;
    marker_centroids.color.r = 1.0;
    marker_centroids.color.g = 0.0;
    marker_centroids.color.b = 0.0;
    marker_centroids.color.a = 0.5;

    visualization_msgs::msg::Marker default_bbox;
    default_bbox.header.frame_id = target_frame_;
    default_bbox.header.stamp = msg->header.stamp;
    default_bbox.ns = "bounding_boxes";
    // default_bbox.id = 0;
    default_bbox.type = visualization_msgs::msg::Marker::LINE_LIST;
    default_bbox.action = visualization_msgs::msg::Marker::ADD;
    default_bbox.scale.x = 0.01;
    default_bbox.color.r = 1.0;
    default_bbox.color.g = 0.0;
    default_bbox.color.b = 0.0;
    default_bbox.color.a = 1.0;

    for (size_t i = 0; i < centroids.size(); ++i) {
      // Get centroid and bbox
      const auto & centroid = centroids[i];
      const auto & bbox = bboxes[i];

      // Centroid marker
      geometry_msgs::msg::Point p;
      p.x = centroid.x();
      p.y = centroid.y();
      p.z = 0.0;
      marker_centroids.points.push_back(p);

      // Bounding box marker
      if (bbox.has_value()) {
        visualization_msgs::msg::Marker bbox_marker = default_bbox;
        bbox_marker.id = static_cast<int>(i);
        const Eigen::Vector3d min_bound = bbox->min_bound_;
        const Eigen::Vector3d max_bound = bbox->max_bound_;
        std::vector<Eigen::Vector3d> corners(8);
        corners[0] = Eigen::Vector3d(min_bound.x(), min_bound.y(), 0.0);
        corners[1] = Eigen::Vector3d(max_bound.x(), min_bound.y(), 0.0);
        corners[2] = Eigen::Vector3d(max_bound.x(), max_bound.y(), 0.0);
        corners[3] = Eigen::Vector3d(min_bound.x(), max_bound.y(), 0.0);
        // Lines
        for (size_t j = 0; j < 4; ++j) {
          geometry_msgs::msg::Point p1, p2;
          p1.x = corners[j].x();
          p1.y = corners[j].y();
          p1.z = 0.0;
          p2.x = corners[(j + 1) % 4].x();
          p2.y = corners[(j + 1) % 4].y();
          p2.z = 0.0;
          bbox_marker.points.push_back(p1);
          bbox_marker.points.push_back(p2);
        }
        marker_array->markers.push_back(bbox_marker);
      }
    }
    marker_array->markers.push_back(marker_centroids);

    // Tracked objects
    for (const auto & [id, track] : tracks) {
      const Eigen::Vector4f & state = track.kf->state;
      const Eigen::Matrix4f & P = track.kf->covariance;

      // Size and alpha based on covariance
      const float pos_std = std::sqrt((P(0, 0) + P(1, 1)) / 2.0f);
      const float scale = pos_std;
      const float alpha = 1.0f - pos_std;

      visualization_msgs::msg::Marker marker_track;
      marker_track.header.frame_id = target_frame_;
      marker_track.header.stamp = msg->header.stamp;
      marker_track.ns = "tracked_objects";
      marker_track.id = id;
      marker_track.type = visualization_msgs::msg::Marker::SPHERE;
      marker_track.action = visualization_msgs::msg::Marker::ADD;
      marker_track.scale.x = scale;
      marker_track.scale.y = scale;
      marker_track.scale.z = 0.01;
      marker_track.color.r = 0.0;
      marker_track.color.g = 1.0;
      marker_track.color.b = 0.0;
      marker_track.color.a = alpha;

      marker_track.pose.position.x = state(0);
      marker_track.pose.position.y = state(1);
      marker_track.pose.position.z = 0.0;
      marker_array->markers.push_back(marker_track);

      // Velocity arrow
      visualization_msgs::msg::Marker marker_velocity;
      marker_velocity.header.frame_id = target_frame_;
      marker_velocity.header.stamp = msg->header.stamp;
      marker_velocity.ns = "tracked_object_velocities";
      marker_velocity.id = id;
      marker_velocity.type = visualization_msgs::msg::Marker::ARROW;
      marker_velocity.action = visualization_msgs::msg::Marker::ADD;
      marker_velocity.scale.x = 0.03;  // shaft diameter
      marker_velocity.scale.y = 0.06;  // head diameter
      marker_velocity.scale.z = 0.06;  // head length
      marker_velocity.color.r = 0.0;
      marker_velocity.color.g = 1.0;
      marker_velocity.color.b = 0.0;
      marker_velocity.color.a = alpha;
      geometry_msgs::msg::Point start_point;
      start_point.x = state(0);
      start_point.y = state(1);
      start_point.z = 0.0;
      marker_velocity.points.push_back(start_point);
      geometry_msgs::msg::Point end_point;
      end_point.x = state(0) + state(2) * 1.0f;  // 1 second ahead
      end_point.y = state(1) + state(3) * 1.0f;
      end_point.z = 0.0;
      marker_velocity.points.push_back(end_point);
      marker_array->markers.push_back(marker_velocity);

      // Add text marker for ID
      visualization_msgs::msg::Marker marker_id;
      marker_id.header.frame_id = target_frame_;
      marker_id.header.stamp = msg->header.stamp;
      marker_id.ns = "tracked_object_ids";
      marker_id.id = id;
      marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker_id.action = visualization_msgs::msg::Marker::ADD;
      marker_id.scale.z = 0.1;
      marker_id.color.r = 1.0;
      marker_id.color.g = 1.0;
      marker_id.color.b = 1.0;
      marker_id.color.a = 1.0;
      marker_id.pose.position.x = state(0);
      marker_id.pose.position.y = state(1);
      marker_id.pose.position.z = 0.5;
      std::stringstream ss;
      ss << "id:" << id << "\n";
      ss << "std:" << std::setprecision(2) << pos_std << "\n";
      ss << "exist_prob:" << std::setprecision(2) << track.existence_probability;
      marker_id.text = ss.str();
      marker_array->markers.push_back(marker_id);
    }

    // Delete dead tracks' markers
    for (const auto & id : track_update_info.deaths) {
      visualization_msgs::msg::Marker marker_delete;
      marker_delete.header.frame_id = target_frame_;
      marker_delete.header.stamp = msg->header.stamp;
      marker_delete.ns = "tracked_objects";
      marker_delete.id = id;
      marker_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_delete);

      visualization_msgs::msg::Marker marker_velocity_delete;
      marker_velocity_delete.header.frame_id = target_frame_;
      marker_velocity_delete.header.stamp = msg->header.stamp;
      marker_velocity_delete.ns = "tracked_object_velocities";
      marker_velocity_delete.id = id;
      marker_velocity_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_velocity_delete);

      visualization_msgs::msg::Marker marker_id_delete;
      marker_id_delete.header.frame_id = target_frame_;
      marker_id_delete.header.stamp = msg->header.stamp;
      marker_id_delete.ns = "tracked_object_ids";
      marker_id_delete.id = id;
      marker_id_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_id_delete);
    }

    marker_pub_->publish(std::move(marker_array));
  }
}

open3d::geometry::PointCloud ObjectsTracker::laserScanToPointCloud(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  open3d::geometry::PointCloud pc;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float & range = msg->ranges[i];
    if (std::isfinite(range)) {
      float angle = msg->angle_min + i * msg->angle_increment;
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      pc.points_.emplace_back(Eigen::Vector3d(x, y, 0.0));
    }
  }

  return pc;
}

std::vector<open3d::geometry::PointCloud> ObjectsTracker::segment(
  const open3d::geometry::PointCloud & pc) const
{
  std::vector<int> labels = pc.ClusterDBSCAN(cluster_neighbor_radius_, cluster_min_points_, false);
  std::vector<open3d::geometry::PointCloud> clusters;
  int max_label = *std::max_element(labels.begin(), labels.end());
  clusters.resize(max_label + 1);
  for (size_t i = 0; i < labels.size(); ++i) {
    int label = labels[i];
    if (label != -1) {
      clusters[label].points_.push_back(pc.points_[i]);
    }
  }

  return clusters;
}

Eigen::Vector2f ObjectsTracker::calculateCentroid(const open3d::geometry::PointCloud & cluster)
{
  Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
  for (const auto & point : cluster.points_) {
    centroid.x() += static_cast<float>(point.x());
    centroid.y() += static_cast<float>(point.y());
  }

  centroid /= static_cast<float>(cluster.points_.size());
  return centroid;
}

}  // namespace lidar_objects_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_objects_tracker::ObjectsTracker)
