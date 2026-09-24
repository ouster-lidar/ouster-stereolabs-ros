// Copyright 2026 Ouster, Inc.
//
// Global (initial-guess-free) extrinsic registration between the Ouster lidar
// and the ZED camera point clouds, using the PCL FPFH + SAC-IA pipeline.

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ouster_zed {

/**
 * Subscribes to the Ouster and ZED point clouds, pairs them by timestamp and
 * estimates the rigid transform that aligns them:
 *
 *   voxel grid -> normals -> FPFH -> SAC-IA (global) -> ICP (optional refine)
 *
 * Both clouds are first expressed in `base_frame` using the poses currently
 * published by ouster_zed.launch.py, so the estimate is a *correction* on top
 * of those poses. The corrected poses are printed as launch arguments ready to
 * be pasted into `ros2 launch ouster_zed ouster_zed.launch.py ...`.
 *
 * The ZED cloud (a partial frontal view) is registered as the source into the
 * Ouster cloud (a full 360 degree view) as the target: every ZED point has a
 * true counterpart in the lidar scan, which is the well-posed direction.
 */
class GlobalRegistrationNode : public rclcpp::Node
{
public:
  explicit GlobalRegistrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<PointT>;
  using Normals = pcl::PointCloud<pcl::Normal>;
  using Features = pcl::PointCloud<pcl::FPFHSignature33>;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
  using Clock = std::chrono::steady_clock;

  /// A downsampled cloud and its FPFH descriptors (index-aligned), in base_frame.
  struct PreparedCloud
  {
    Cloud::Ptr points;
    Features::Ptr features;
  };

  struct Params
  {
    std::string ouster_topic;
    std::string zed_topic;
    std::string base_frame;
    std::string ouster_frame;      // frame whose pose the launch file sets (os_sensor)
    std::string zed_frame;         // frame whose pose the launch file sets (zed_camera_link)
    std::string reference_sensor;  // "ouster" or "zed": the sensor that keeps its pose

    int sync_queue_size;
    double sync_max_interval;  // [s] max stamp difference within a pair

    double ouster_min_range, ouster_max_range;  // [m] crop in the sensor's own frame
    double zed_min_range, zed_max_range;

    double voxel_leaf_size;  // [m]
    double normal_radius;    // [m]
    double feature_radius;   // [m] must be larger than normal_radius
    int num_threads;         // 0 = use all cores

    int sac_max_iterations;
    int sac_num_samples;
    int sac_correspondence_randomness;
    double sac_min_sample_distance;       // [m]
    double sac_max_correspondence_distance;  // [m]

    bool icp_refine;
    int icp_max_iterations;
    double icp_max_correspondence_distance;  // [m]

    double inlier_distance;  // [m] used for the reported overlap score
    double consensus_translation;  // [m] two runs agree if closer than this ...
    double consensus_rotation;     // [deg] ... and this
    int min_agreeing_runs;         // consensus size needed to trust (and save) a result
    int target_agreeing_runs;      // stop registering at this consensus size; 0 = never

    std::string calibration_file;  // consensus poses are written here; empty = don't save
    double period;           // [s] min time between two registrations
    int max_runs;            // 0 = keep running
  };

  void declare_params();

  /// Synchronized callback: runs the whole pipeline on one Ouster/ZED pair.
  void on_clouds(const CloudMsg::ConstSharedPtr & ouster_msg,
                 const CloudMsg::ConstSharedPtr & zed_msg);

  /// base_frame <- frame, from the (static) TF tree. std::nullopt if unavailable.
  std::optional<Eigen::Isometry3d> lookup(const std::string & frame) const;

  /// Convert to PCL, drop invalid/out-of-range points and express in base_frame.
  static Cloud::Ptr to_base_frame(const CloudMsg & msg, const Eigen::Isometry3d & base_T_cloud,
                                  double min_range, double max_range);

  /// Voxel grid + normal estimation + FPFH. `viewpoint` orients the normals.
  PreparedCloud prepare(const Cloud::ConstPtr & cloud, const Eigen::Vector3d & viewpoint) const;

  /// Fraction of `source` points having a `target` neighbour closer than `max_dist`.
  static double inlier_ratio(const Cloud & source, const Cloud::ConstPtr & target,
                             double max_dist);

  /// Largest group of mutually agreeing estimates in `estimates_` and its mean pose.
  std::pair<Eigen::Isometry3d, std::size_t> consensus() const;

  /// Write both sensor poses to params_.calibration_file (atomically). False on failure.
  bool save_calibration(const Eigen::Isometry3d & base_T_ouster, const Eigen::Isometry3d & base_T_zed,
                        std::size_t agreeing_runs) const;

  /// {x, y, z, roll, pitch, yaw} of a pose, with the RPY convention of
  /// tf2 static_transform_publisher.
  static std::array<double, 6> to_xyzrpy(const Eigen::Isometry3d & pose);

  /// Launch arguments `<prefix>_x:=.. <prefix>_y:=.. ... <prefix>_yaw:=..` for a pose.
  static std::string pose_as_launch_args(const std::string & prefix,
                                         const Eigen::Isometry3d & pose);

  Params params_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<CloudMsg> ouster_sub_;
  message_filters::Subscriber<CloudMsg> zed_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  /// Estimated base_frame pose of the moved (non-reference) sensor, one per run.
  /// SAC-IA is randomized and may occasionally lock onto a wrong alignment, so
  /// the reported result is the consensus over runs rather than the last one.
  std::vector<Eigen::Isometry3d> estimates_;

  std::optional<Clock::time_point> last_run_;
  int runs_ = 0;
  bool done_ = false;  // target_agreeing_runs reached
};

}  // namespace ouster_zed
