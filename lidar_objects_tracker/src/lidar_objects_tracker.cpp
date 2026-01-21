/* Copyright 2025 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2025
 */

#include "lidar_objects_tracker/lidar_objects_tracker.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_objects_tracker
{

ObjectsTracker::ObjectsTracker(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("objects_tracker", options)
{
  // Get parameters
  declare_parameter<double>("cluster_neighbor_radius", 0.2);
  cluster_neighbor_radius_ = get_parameter("cluster_neighbor_radius").as_double();

  declare_parameter<double>("max_dt", 1.0);
  float max_dt = get_parameter("max_dt").as_double();

  declare_parameter<int>("cluster_min_points", 6);
  cluster_min_points_ = get_parameter("cluster_min_points").as_int();

  declare_parameter<int>("cluster_max_points", 50);
  cluster_max_points_ = get_parameter("cluster_max_points").as_int();

  declare_parameter<bool>("visualize", true);
  visualize_ = get_parameter("visualize").as_bool();

  tracker_ = std::make_unique<LMBTracker>(get_clock());

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
  const open3d::geometry::PointCloud pc = laserScanToPointCloud(msg);

  // Segment and calculate centroids
  const auto clusters = segment(pc);
  std::vector<Eigen::Vector2f> centroids;
  for (const auto & cluster : clusters) {
    if (cluster.points_.size() < static_cast<size_t>(cluster_min_points_) ||
      cluster.points_.size() > static_cast<size_t>(cluster_max_points_))
    {
      continue;
    }

    centroids.push_back(calculateCentroid(cluster));
  }

  // Update tracker
  const UpdateInfo track_update_info = tracker_->updateTracks(centroids);
  RCLCPP_INFO(
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
    tracked_object_msg.header = msg->header;
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

    // Cluster centroids
    visualization_msgs::msg::Marker marker_centroids;
    marker_centroids.header = msg->header;
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
    marker_centroids.color.a = 1.0;

    for (const auto & centroid : centroids) {
      geometry_msgs::msg::Point p;
      p.x = centroid.x();
      p.y = centroid.y();
      p.z = 0.0;
      marker_centroids.points.push_back(p);
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
      marker_track.header = msg->header;
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
      marker_velocity.header = msg->header;
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
      end_point.x = state(0) + state(2) * track_update_info.dt * 3;  // scale for visibility
      end_point.y = state(1) + state(3) * track_update_info.dt * 3;
      end_point.z = 0.0;
      marker_velocity.points.push_back(end_point);
      marker_array->markers.push_back(marker_velocity);

      // Add text marker for ID
      visualization_msgs::msg::Marker marker_id;
      marker_id.header = msg->header;
      marker_id.ns = "tracked_object_ids";
      marker_id.id = id;
      marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker_id.action = visualization_msgs::msg::Marker::ADD;
      marker_id.scale.z = 0.2;
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
      ss << "exist_prob:" << std::setprecision(2) << track.existence_probability << "\n";
      if (track_update_info.updates.find(id) != track_update_info.updates.end()) {
        ss << "meas:";
        const auto & weights = track_update_info.updates.at(id).measurement_weights;
        for (const auto & [meas_idx, weight] : weights) {
          ss << meas_idx << ",";
        }
      } else {
        ss << "missed_det";
      }
      marker_id.text = ss.str();
      marker_array->markers.push_back(marker_id);
    }

    // Delete dead tracks' markers
    for (const auto & id : track_update_info.deaths) {
      visualization_msgs::msg::Marker marker_delete;
      marker_delete.header = msg->header;
      marker_delete.ns = "tracked_objects";
      marker_delete.id = id;
      marker_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_delete);

      visualization_msgs::msg::Marker marker_velocity_delete;
      marker_velocity_delete.header = msg->header;
      marker_velocity_delete.ns = "tracked_object_velocities";
      marker_velocity_delete.id = id;
      marker_velocity_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_velocity_delete);

      visualization_msgs::msg::Marker marker_id_delete;
      marker_id_delete.header = msg->header;
      marker_id_delete.ns = "tracked_object_ids";
      marker_id_delete.id = id;
      marker_id_delete.action = visualization_msgs::msg::Marker::DELETE;
      marker_array->markers.push_back(marker_id_delete);
    }

    RCLCPP_INFO(
      get_logger(), "Publishing %zu markers for visualization",
      marker_array->markers.size());
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
