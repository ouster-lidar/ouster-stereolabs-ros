// Copyright 2026 Ouster, Inc.

#include "ouster_zed/global_registration_node.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace ouster_zed {

namespace {

double rotation_angle(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

Eigen::Isometry3d to_isometry(const Eigen::Matrix4f & m)
{
  Eigen::Isometry3d iso;
  iso.matrix() = m.cast<double>();
  return iso;
}

bool is_finite(const pcl::FPFHSignature33 & f)
{
  for (float v : f.histogram) {
    if (!std::isfinite(v)) {return false;}
  }
  return true;
}

}  // namespace

GlobalRegistrationNode::GlobalRegistrationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("global_registration", options)
{
  declare_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Both drivers publish their clouds with a best-effort (sensor data) QoS; a
  // best-effort subscription is compatible with reliable publishers too.
  ouster_sub_.subscribe(this, params_.ouster_topic, rmw_qos_profile_sensor_data);
  zed_sub_.subscribe(this, params_.zed_topic, rmw_qos_profile_sensor_data);

  // The two sensors are not hardware synchronized, so pair messages by the
  // closest stamps rather than requiring identical ones.
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(params_.sync_queue_size), ouster_sub_, zed_sub_);
  sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(params_.sync_max_interval));
  sync_->registerCallback(std::bind(&GlobalRegistrationNode::on_clouds, this,
                                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Waiting for synchronized clouds on '%s' and '%s' (max stamp gap %.3f s)",
              params_.ouster_topic.c_str(), params_.zed_topic.c_str(), params_.sync_max_interval);
}

void GlobalRegistrationNode::declare_params()
{
  auto & p = params_;
  p.ouster_topic = declare_parameter<std::string>("ouster_topic", "/ouster/points");
  p.zed_topic = declare_parameter<std::string>(
    "zed_topic", "/zed/zed_node/point_cloud/cloud_registered");
  p.base_frame = declare_parameter<std::string>("base_frame", "base_link");
  p.ouster_frame = declare_parameter<std::string>("ouster_frame", "os_sensor");
  p.zed_frame = declare_parameter<std::string>("zed_frame", "zed_camera_link");
  p.reference_sensor = declare_parameter<std::string>("reference_sensor", "ouster");
  if (p.reference_sensor != "ouster" && p.reference_sensor != "zed") {
    throw std::invalid_argument("reference_sensor must be 'ouster' or 'zed'");
  }

  p.sync_queue_size = declare_parameter<int>("sync_queue_size", 10);
  p.sync_max_interval = declare_parameter<double>("sync_max_interval", 0.05);

  p.ouster_min_range = declare_parameter<double>("ouster_min_range", 0.5);
  p.ouster_max_range = declare_parameter<double>("ouster_max_range", 20.0);
  p.zed_min_range = declare_parameter<double>("zed_min_range", 0.3);
  p.zed_max_range = declare_parameter<double>("zed_max_range", 10.0);

  p.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.1);
  p.normal_radius = declare_parameter<double>("normal_radius", 0.3);
  p.feature_radius = declare_parameter<double>("feature_radius", 0.6);
  p.num_threads = declare_parameter<int>("num_threads", 0);

  p.sac_max_iterations = declare_parameter<int>("sac_max_iterations", 1000);
  p.sac_num_samples = declare_parameter<int>("sac_num_samples", 3);
  p.sac_correspondence_randomness = declare_parameter<int>("sac_correspondence_randomness", 10);
  p.sac_min_sample_distance = declare_parameter<double>("sac_min_sample_distance", 0.5);
  p.sac_max_correspondence_distance =
    declare_parameter<double>("sac_max_correspondence_distance", 0.5);

  p.icp_refine = declare_parameter<bool>("icp_refine", true);
  p.icp_max_iterations = declare_parameter<int>("icp_max_iterations", 50);
  p.icp_max_correspondence_distance =
    declare_parameter<double>("icp_max_correspondence_distance", 0.3);

  p.inlier_distance = declare_parameter<double>("inlier_distance", 0.1);
  p.consensus_translation = declare_parameter<double>("consensus_translation", 0.1);
  p.consensus_rotation = declare_parameter<double>("consensus_rotation", 5.0);
  p.min_agreeing_runs = declare_parameter<int>("min_agreeing_runs", 3);
  p.target_agreeing_runs = declare_parameter<int>("target_agreeing_runs", 10);

  p.calibration_file = declare_parameter<std::string>("calibration_file", "");
  p.period = declare_parameter<double>("period", 5.0);
  p.max_runs = declare_parameter<int>("max_runs", 0);
}

std::optional<Eigen::Isometry3d> GlobalRegistrationNode::lookup(const std::string & frame) const
{
  try {
    // All frames involved are static, so the latest transform is the right one.
    return tf2::transformToEigen(
      tf_buffer_->lookupTransform(params_.base_frame, frame, tf2::TimePointZero));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s <- %s unavailable: %s",
                         params_.base_frame.c_str(), frame.c_str(), e.what());
    return std::nullopt;
  }
}

GlobalRegistrationNode::Cloud::Ptr GlobalRegistrationNode::to_base_frame(
  const CloudMsg & msg, const Eigen::Isometry3d & base_T_cloud, double min_range, double max_range)
{
  // Only x/y/z are read; the other fields (intensity, rgb, ...) are ignored.
  Cloud raw;
  pcl::fromROSMsg(msg, raw);

  auto cloud = std::make_shared<Cloud>();
  cloud->reserve(raw.size());
  const double min_sq = min_range * min_range;
  const double max_sq = max_range * max_range;
  for (const auto & pt : raw) {
    const double r_sq = pt.getVector3fMap().squaredNorm();
    // Written so that NaN (ZED invalid depth) also fails the test. Ouster
    // invalid returns sit at the origin and are removed by min_range.
    if (!(r_sq >= min_sq && r_sq <= max_sq)) {continue;}
    cloud->push_back(pt);
  }
  cloud->is_dense = true;

  pcl::transformPointCloud(*cloud, *cloud, base_T_cloud.matrix().cast<float>());
  return cloud;
}

GlobalRegistrationNode::PreparedCloud GlobalRegistrationNode::prepare(
  const Cloud::ConstPtr & cloud, const Eigen::Vector3d & viewpoint) const
{
  // 1. Voxel grid: evens out the very different densities of a stereo depth
  //    map and a spinning lidar, and bounds the cost of the later stages.
  auto down = std::make_shared<Cloud>();
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud);
  const auto leaf = static_cast<float>(params_.voxel_leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*down);

  // 2. Surface normals. The OMP variant is a multi-threaded subclass of
  //    pcl::NormalEstimation with identical results. Orienting the normals
  //    towards the sensor keeps FPFH signatures consistent between sensors.
  auto normals = std::make_shared<Normals>();
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne(params_.num_threads);
  ne.setInputCloud(down);
  ne.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
  ne.setRadiusSearch(params_.normal_radius);
  ne.setViewPoint(viewpoint.x(), viewpoint.y(), viewpoint.z());
  ne.compute(*normals);

  // Points without enough neighbours get NaN normals, which would propagate
  // into NaN features and poison the feature-space nearest neighbour search.
  auto points = std::make_shared<Cloud>();
  auto valid_normals = std::make_shared<Normals>();
  for (std::size_t i = 0; i < down->size(); ++i) {
    if (pcl::isFinite((*normals)[i])) {
      points->push_back((*down)[i]);
      valid_normals->push_back((*normals)[i]);
    }
  }

  // 3. FPFH descriptors (multi-threaded subclass of pcl::FPFHEstimation). The
  //    search radius must be larger than the one used for the normals.
  auto features = std::make_shared<Features>();
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh(params_.num_threads);
  fpfh.setInputCloud(points);
  fpfh.setInputNormals(valid_normals);
  fpfh.setSearchMethod(std::make_shared<pcl::search::KdTree<PointT>>());
  fpfh.setRadiusSearch(params_.feature_radius);
  fpfh.compute(*features);

  // Isolated points can still yield degenerate histograms; keep clouds and
  // features index-aligned while dropping them.
  PreparedCloud out{std::make_shared<Cloud>(), std::make_shared<Features>()};
  for (std::size_t i = 0; i < features->size(); ++i) {
    if (is_finite((*features)[i])) {
      out.points->push_back((*points)[i]);
      out.features->push_back((*features)[i]);
    }
  }
  return out;
}

double GlobalRegistrationNode::inlier_ratio(const Cloud & source, const Cloud::ConstPtr & target,
                                            double max_dist)
{
  if (source.empty() || target->empty()) {return 0.0;}
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  std::vector<int> index(1);
  std::vector<float> sq_dist(1);
  std::size_t inliers = 0;
  for (const auto & pt : source) {
    if (tree.nearestKSearch(pt, 1, index, sq_dist) == 1 && sq_dist[0] < max_dist * max_dist) {
      ++inliers;
    }
  }
  return static_cast<double>(inliers) / static_cast<double>(source.size());
}

std::pair<Eigen::Isometry3d, std::size_t> GlobalRegistrationNode::consensus() const
{
  const double max_angle = params_.consensus_rotation * M_PI / 180.0;
  auto agree = [&](const Eigen::Isometry3d & a, const Eigen::Isometry3d & b) {
      return (a.translation() - b.translation()).norm() < params_.consensus_translation &&
             rotation_angle(a, b) < max_angle;
    };

  // Pick the estimate that agrees with the most others (O(n^2), n is small) ...
  std::size_t best = 0, best_count = 0;
  for (std::size_t i = 0; i < estimates_.size(); ++i) {
    std::size_t count = 0;
    for (const auto & other : estimates_) {count += agree(estimates_[i], other);}
    if (count > best_count) {best = i; best_count = count;}
  }

  // ... and average its group. Rotations are close to each other, so a
  // sign-aligned normalized quaternion sum is an accurate mean.
  Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond q_ref(estimates_[best].rotation());
  for (const auto & e : estimates_) {
    if (!agree(estimates_[best], e)) {continue;}
    Eigen::Quaterniond q(e.rotation());
    if (q.dot(q_ref) < 0.0) {q.coeffs() = -q.coeffs();}
    t_sum += e.translation();
    q_sum += q.coeffs();
  }
  Eigen::Isometry3d mean = Eigen::Isometry3d::Identity();
  mean.linear() = Eigen::Quaterniond(q_sum.normalized()).toRotationMatrix();
  mean.translation() = t_sum / static_cast<double>(best_count);
  return {mean, best_count};
}

std::array<double, 6> GlobalRegistrationNode::to_xyzrpy(const Eigen::Isometry3d & pose)
{
  // static_transform_publisher (used by ouster_zed.launch.py) builds its
  // rotation with tf2 setRPY, so decompose with the matching tf2 getRPY.
  const Eigen::Matrix3d r = pose.rotation();
  const tf2::Matrix3x3 m(r(0, 0), r(0, 1), r(0, 2),
                         r(1, 0), r(1, 1), r(1, 2),
                         r(2, 0), r(2, 1), r(2, 2));
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  const Eigen::Vector3d t = pose.translation();
  return {t.x(), t.y(), t.z(), roll, pitch, yaw};
}

std::string GlobalRegistrationNode::pose_as_launch_args(const std::string & prefix,
                                                        const Eigen::Isometry3d & pose)
{
  static constexpr const char * keys[] = {"x", "y", "z", "roll", "pitch", "yaw"};
  const auto values = to_xyzrpy(pose);
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  for (std::size_t i = 0; i < values.size(); ++i) {
    ss << (i ? " " : "") << prefix << '_' << keys[i] << ":=" << values[i];
  }
  return ss.str();
}

bool GlobalRegistrationNode::save_calibration(const Eigen::Isometry3d & base_T_ouster,
                                              const Eigen::Isometry3d & base_T_zed,
                                              std::size_t agreeing_runs) const
{
  namespace fs = std::filesystem;
  static constexpr const char * keys[] = {"x", "y", "z", "roll", "pitch", "yaw"};

  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);

  std::ostringstream yaml;
  yaml << "# ouster_zed sensor calibration, written by the global_registration node.\n"
       << "# Poses of each sensor frame relative to base_frame: meters, and radians\n"
       << "# using the roll/pitch/yaw convention of tf2 static_transform_publisher.\n"
       << "# Used by: ros2 launch ouster_zed ouster_zed.launch.py calibration_file:=<this file>\n"
       << "stamp: \"" << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ") << "\"\n"
       << "base_frame: " << params_.base_frame << "\n"
       << "reference_sensor: " << params_.reference_sensor << "\n"
       << "agreeing_runs: " << agreeing_runs << "\n"
       << "total_runs: " << estimates_.size() << "\n"
       << std::fixed << std::setprecision(6);
  const std::pair<const char *, const Eigen::Isometry3d *> sensors[] = {
    {"ouster", &base_T_ouster}, {"zed", &base_T_zed}};
  for (const auto & [name, pose] : sensors) {
    const auto & frame = std::string(name) == "ouster" ? params_.ouster_frame : params_.zed_frame;
    yaml << name << ":\n  frame: " << frame << "\n";
    const auto values = to_xyzrpy(*pose);
    for (std::size_t i = 0; i < values.size(); ++i) {
      yaml << "  " << keys[i] << ": " << values[i] << "\n";
    }
  }

  // Write to a temporary file and rename it over the target, so a launch that
  // reads the file concurrently never sees a half-written calibration.
  try {
    const fs::path path(params_.calibration_file);
    if (path.has_parent_path()) {fs::create_directories(path.parent_path());}
    const fs::path tmp = path.string() + ".tmp";
    {
      std::ofstream f(tmp);
      f << yaml.str();
      if (!f) {throw std::runtime_error("write failed");}
    }
    fs::rename(tmp, path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not save calibration to '%s': %s",
                 params_.calibration_file.c_str(), e.what());
    return false;
  }
  return true;
}

void GlobalRegistrationNode::on_clouds(const CloudMsg::ConstSharedPtr & ouster_msg,
                                       const CloudMsg::ConstSharedPtr & zed_msg)
{
  if (done_ || (params_.max_runs > 0 && runs_ >= params_.max_runs)) {return;}
  if (last_run_ && seconds_since(*last_run_) < params_.period) {return;}

  // Poses currently published by the launch file, plus the sensors' internal
  // static transforms down to the frames the clouds are stamped in.
  const auto base_T_ouster = lookup(params_.ouster_frame);
  const auto base_T_zed = lookup(params_.zed_frame);
  const auto base_T_ouster_cloud = lookup(ouster_msg->header.frame_id);
  const auto base_T_zed_cloud = lookup(zed_msg->header.frame_id);
  if (!base_T_ouster || !base_T_zed || !base_T_ouster_cloud || !base_T_zed_cloud) {return;}

  last_run_ = Clock::now();
  ++runs_;
  const auto t_start = Clock::now();

  // ---- Convert + crop + express both clouds in base_frame -------------------
  const auto ouster_cloud = to_base_frame(*ouster_msg, *base_T_ouster_cloud,
                                          params_.ouster_min_range, params_.ouster_max_range);
  const auto zed_cloud = to_base_frame(*zed_msg, *base_T_zed_cloud,
                                       params_.zed_min_range, params_.zed_max_range);

  // ---- Voxel grid, normals, FPFH --------------------------------------------
  const auto target = prepare(ouster_cloud, base_T_ouster_cloud->translation());
  const auto source = prepare(zed_cloud, base_T_zed_cloud->translation());
  const double t_features = seconds_since(t_start);

  if (static_cast<int>(source.points->size()) < params_.sac_num_samples ||
    static_cast<int>(target.points->size()) < params_.sac_num_samples)
  {
    RCLCPP_WARN(get_logger(), "Not enough points with valid features (zed %zu, ouster %zu); "
                "check the range limits and voxel/feature radii",
                source.points->size(), target.points->size());
    return;
  }

  // ---- 4. Global alignment: SAC-IA ------------------------------------------
  // Repeatedly picks `num_samples` ZED points at least `min_sample_distance`
  // apart, matches each to one of its `correspondence_randomness` most similar
  // Ouster FPFH descriptors, solves the rigid transform and keeps the best.
  const auto t_sac = Clock::now();
  pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac;
  sac.setInputSource(source.points);
  sac.setSourceFeatures(source.features);
  sac.setInputTarget(target.points);
  sac.setTargetFeatures(target.features);
  sac.setMaximumIterations(params_.sac_max_iterations);
  sac.setNumberOfSamples(params_.sac_num_samples);
  sac.setCorrespondenceRandomness(params_.sac_correspondence_randomness);
  sac.setMinSampleDistance(static_cast<float>(params_.sac_min_sample_distance));
  sac.setMaxCorrespondenceDistance(params_.sac_max_correspondence_distance);
  Cloud sac_aligned;
  sac.align(sac_aligned);
  if (!sac.hasConverged()) {
    RCLCPP_WARN(get_logger(), "SAC-IA did not converge, retrying on the next pair");
    return;
  }
  // correction: maps ZED points (in base_frame, current poses) onto the Ouster ones
  Eigen::Isometry3d correction = to_isometry(sac.getFinalTransformation());
  const double sac_fitness = sac.getFitnessScore(params_.sac_max_correspondence_distance);
  const double sac_inliers = inlier_ratio(sac_aligned, target.points, params_.inlier_distance);
  const double t_sac_s = seconds_since(t_sac);

  // ---- Optional local refinement: ICP seeded with the SAC-IA result ---------
  std::ostringstream icp_line;
  double final_inliers = sac_inliers;
  if (params_.icp_refine) {
    const auto t_icp = Clock::now();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source.points);
    icp.setInputTarget(target.points);
    icp.setMaxCorrespondenceDistance(params_.icp_max_correspondence_distance);
    icp.setMaximumIterations(params_.icp_max_iterations);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);
    Cloud icp_aligned;
    icp.align(icp_aligned, correction.matrix().cast<float>());
    if (icp.hasConverged()) {
      correction = to_isometry(icp.getFinalTransformation());
      final_inliers = inlier_ratio(icp_aligned, target.points, params_.inlier_distance);
      icp_line << std::fixed << std::setprecision(4) << "converged, fitness "
               << icp.getFitnessScore(params_.icp_max_correspondence_distance) << " m^2, "
               << std::setprecision(1) << 100.0 * final_inliers << "% inliers, "
               << std::setprecision(2) << seconds_since(t_icp) << " s";
    } else {
      icp_line << "did not converge, keeping the SAC-IA result";
    }
  } else {
    icp_line << "disabled";
  }

  // ---- Corrected sensor poses -----------------------------------------------
  // After correction:  zed_in_base ~= ouster_in_base. Either move the ZED by the
  // correction, or keep the ZED and move the Ouster by its inverse. The sensors'
  // internal transforms (e.g. zed_camera_link -> zed_left_camera_frame) are
  // static, so correcting the cloud frame corrects the launch frame equally.
  const bool zed_moves = params_.reference_sensor == "ouster";
  const Eigen::Isometry3d estimate = zed_moves ?
    correction * *base_T_zed : correction.inverse() * *base_T_ouster;
  estimates_.push_back(estimate);
  const auto [agreed_pose, agreed_count] = consensus();
  const Eigen::Isometry3d & new_base_T_zed = zed_moves ? agreed_pose : *base_T_zed;
  const Eigen::Isometry3d & new_base_T_ouster = zed_moves ? *base_T_ouster : agreed_pose;
  const std::string moved = zed_moves ? "zed" : "ouster";

  const double stamp_gap = std::abs(
    (rclcpp::Time(ouster_msg->header.stamp) - rclcpp::Time(zed_msg->header.stamp)).seconds());
  const Eigen::AngleAxisd delta_rot(correction.rotation());

  std::ostringstream out;
  out << std::fixed
      << "\n========== Ouster <-> ZED global registration, run " << runs_ << " ==========\n"
      << std::setprecision(3) << "  pair stamp gap : " << stamp_gap << " s\n"
      << "  points         : ouster " << ouster_msg->width * ouster_msg->height << " raw -> "
      << ouster_cloud->size() << " cropped -> " << target.points->size() << " with features\n"
      << "                   zed    " << zed_msg->width * zed_msg->height << " raw -> "
      << zed_cloud->size() << " cropped -> " << source.points->size() << " with features\n"
      << std::setprecision(2) << "  features       : " << t_features << " s\n"
      << std::setprecision(4) << "  SAC-IA         : fitness " << sac_fitness << " m^2, "
      << std::setprecision(1) << 100.0 * sac_inliers << "% inliers, "
      << std::setprecision(2) << t_sac_s << " s\n"
      << "  ICP            : " << icp_line.str() << "\n"
      << std::setprecision(3) << "  correction     : " << correction.translation().norm()
      << " m, " << std::setprecision(2) << delta_rot.angle() * 180.0 / M_PI << " deg"
      << "  (~0 once the launch poses are right)\n"
      << "  inliers = share of ZED points within " << std::setprecision(2)
      << params_.inlier_distance << " m of an Ouster point\n"
      << "  this run       : " << pose_as_launch_args(moved, estimate) << "\n"
      << "  consensus      : " << agreed_count << " of " << estimates_.size()
      << " runs agree (within " << std::setprecision(2) << params_.consensus_translation
      << " m / " << std::setprecision(1) << params_.consensus_rotation << " deg); "
      << params_.reference_sensor << " pose kept fixed\n";
  const bool trusted = agreed_count >= static_cast<std::size_t>(params_.min_agreeing_runs);
  if (!trusted) {
    out << "  (not reliable yet: waiting for at least " << params_.min_agreeing_runs
        << " agreeing runs)\n";
  } else if (!params_.calibration_file.empty() &&
    save_calibration(new_base_T_ouster, new_base_T_zed, agreed_count))
  {
    out << "  saved to       : " << params_.calibration_file << "\n";
  }
  out << "  Relaunch with the consensus:\n"
      << "    ros2 launch ouster_zed ouster_zed.launch.py \\\n"
      << "      " << pose_as_launch_args("ouster", new_base_T_ouster) << " \\\n"
      << "      " << pose_as_launch_args("zed", new_base_T_zed) << "\n";
  if (final_inliers < 0.3) {
    out << "  WARNING: low overlap, this run is likely wrong. Make sure both sensors see\n"
        << "  the same structured scene, or tune voxel_leaf_size / feature_radius.\n";
  }
  RCLCPP_INFO(get_logger(), "%s", out.str().c_str());

  if (params_.target_agreeing_runs > 0 &&
    agreed_count >= static_cast<std::size_t>(params_.target_agreeing_runs))
  {
    done_ = true;
    if (params_.calibration_file.empty()) {
      RCLCPP_INFO(get_logger(), "Calibration complete (%zu of %zu runs agree); "
                  "no further registrations", agreed_count, estimates_.size());
    } else {
      RCLCPP_INFO(get_logger(), "Calibration complete (%zu of %zu runs agree), saved to '%s'. "
                  "Relaunch without register:=true to use it.", agreed_count, estimates_.size(),
                  params_.calibration_file.c_str());
    }
  } else if (params_.max_runs > 0 && runs_ >= params_.max_runs) {
    RCLCPP_INFO(get_logger(), "Reached max_runs=%d, no further registrations", params_.max_runs);
  }
}

}  // namespace ouster_zed

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_zed::GlobalRegistrationNode)
