/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "node.hpp"
#include "rko_lio/core/process_timestamps.hpp"
#include "rko_lio/core/profiler.hpp"
#include "rko_lio/ros_utils/ros_utils.hpp"
// other
#include <rclcpp/serialization.hpp>
#include <stdexcept>

namespace {
using namespace std::literals;

rko_lio::core::ImuControl imu_msg_to_imu_data(const sensor_msgs::msg::Imu& imu_msg) {
  rko_lio::core::ImuControl imu_data;
  imu_data.time = rko_lio::ros_utils::ros_time_to_seconds(imu_msg.header.stamp);
  imu_data.angular_velocity = rko_lio::ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.angular_velocity);
  imu_data.acceleration = rko_lio::ros_utils::ros_xyz_to_eigen_vector3d(imu_msg.linear_acceleration);
  return imu_data;
}

} // namespace

namespace rko_lio::ros {

Node::Node(const std::string& node_name, const rclcpp::NodeOptions& options) {
  node = rclcpp::Node::make_shared(node_name, options);
  imu_topic = node->declare_parameter<std::string>("imu_topic");     // required
  lidar_topic = node->declare_parameter<std::string>("lidar_topic"); // required
  base_frame = node->declare_parameter<std::string>("base_frame");   // required
  imu_frame = node->declare_parameter<std::string>("imu_frame", imu_frame);
  lidar_frame = node->declare_parameter<std::string>("lidar_frame", lidar_frame);
  odom_frame = node->declare_parameter<std::string>("odom_frame", odom_frame);
  odom_topic = node->declare_parameter<std::string>("odom_topic", odom_topic);
  results_dir = node->declare_parameter<std::string>("results_dir", results_dir);
  run_name = node->declare_parameter<std::string>("run_name", run_name);

  // tf
  invert_odom_tf = node->declare_parameter<bool>("invert_odom_tf", invert_odom_tf);
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  // publishing
  const rclcpp::QoS publisher_qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, publisher_qos);

  publish_lidar_acceleration = node->declare_parameter<bool>("publish_lidar_acceleration", publish_lidar_acceleration);
  if (publish_lidar_acceleration) {
    lidar_accel_publisher =
        node->create_publisher<geometry_msgs::msg::AccelStamped>("/rko_lio/lidar_acceleration", publisher_qos);
  }

  publish_deskewed_scan = node->declare_parameter<bool>("publish_deskewed_scan", publish_deskewed_scan);
  if (publish_deskewed_scan) {
    frame_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/rko_lio/frame", publisher_qos);
  }

  publish_local_map = node->declare_parameter<bool>("publish_local_map", publish_local_map);
  if (publish_local_map) {
    map_topic = node->declare_parameter<std::string>("map_topic", "/rko_lio/local_map");
    publish_map_after = core::Secondsd(node->declare_parameter<double>("publish_map_after", publish_map_after.count()));
    map_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, publisher_qos);
    map_publish_thead = std::jthread([this]() { publish_map_loop(); });
  }

  // lio params
  core::LIO::Config lio_config{};
  lio_config.deskew = node->declare_parameter<bool>("deskew", lio_config.deskew);
  lio_config.max_iterations =
      static_cast<size_t>(node->declare_parameter<int>("max_iterations", static_cast<int>(lio_config.max_iterations)));
  lio_config.voxel_size = node->declare_parameter<double>("voxel_size", lio_config.voxel_size);
  lio_config.max_points_per_voxel =
      static_cast<int>(node->declare_parameter<int>("max_points_per_voxel", lio_config.max_points_per_voxel));
  lio_config.max_range = node->declare_parameter<double>("max_range", lio_config.max_range);
  lio_config.min_range = node->declare_parameter<double>("min_range", lio_config.min_range);
  lio_config.convergence_criterion =
      node->declare_parameter<double>("convergence_criterion", lio_config.convergence_criterion);
  lio_config.max_correspondance_distance =
      node->declare_parameter<double>("max_correspondance_distance", lio_config.max_correspondance_distance);
  lio_config.max_num_threads =
      static_cast<int>(node->declare_parameter<int>("max_num_threads", lio_config.max_num_threads));
  lio_config.initialization_phase =
      node->declare_parameter<bool>("initialization_phase", lio_config.initialization_phase);
  lio_config.max_expected_jerk = node->declare_parameter<double>("max_expected_jerk", lio_config.max_expected_jerk);
  lio_config.double_downsample = node->declare_parameter<bool>("double_downsample", lio_config.double_downsample);
  lio_config.min_beta = node->declare_parameter<double>("min_beta", lio_config.min_beta);
  lio = std::make_unique<core::LIO>(lio_config);

  // manually, if, define extrinsics
  parse_cli_extrinsics();

  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Subscribed to IMU: "
                         << imu_topic << (!imu_frame.empty() ? " (frame " + imu_frame + ")" : "") << " and LiDAR: "
                         << lidar_topic << (!lidar_frame.empty() ? " (frame " + lidar_frame + ")" : "")
                         << ". Max number of threads: " << lio_config.max_num_threads << ". Publishing odometry to "
                         << odom_topic << " ( " << odom_frame
                         << " ) and acceleration "
                            "estimates to /rko_lio/lidar_acceleration. Deskewing is "
                         << (lio->config.deskew ? "enabled" : "disabled") << "."
                         << (publish_deskewed_scan ? " Publishing deskewed_cloud to /rko_lio/frame." : ""));

  registration_thread = std::jthread([this]() { registration_loop(); });

  RCLCPP_INFO(node->get_logger(), "RKO LIO Node is up!");
}

void Node::parse_cli_extrinsics() {
  auto parse_extrinsic = [this](const std::string& name, Sophus::SE3d& extrinsic) {
    const std::string param_name = "extrinsic_" + name + "2base_quat_xyzw_xyz";
    const std::vector<double> vec = node->declare_parameter<std::vector<double>>(param_name, std::vector<double>{});

    if (vec.size() != 7) {
      if (!vec.empty()) {
        RCLCPP_WARN_STREAM(node->get_logger(),
                           "Parameter 'extrinsic_"
                               << name << "2base_quat_xyzw_xyz' is set but has wrong size: " << vec.size()
                               << ". Expected 7 (qx, qy, qz, qw, x, y, z). check the value: "
                               << Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size()).transpose());
      }
      return false;
    }
    Eigen::Quaterniond q(vec[3], vec[0], vec[1], vec[2]); // qw, qx, qy, qz
    if (q.norm() < 1e-6) {
      throw std::runtime_error(name + " extrinsic quaternion has zero norm");
    }
    extrinsic = Sophus::SE3d(q, Eigen::Vector3d(vec[4], vec[5], vec[6]));
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed " << name << " extrinsic as: " << extrinsic.log().transpose());
    return true;
  };
  const bool imu_ok = parse_extrinsic("imu", extrinsic_imu2base);
  const bool lidar_ok = parse_extrinsic("lidar", extrinsic_lidar2base);
  extrinsics_set = imu_ok && lidar_ok;
}

bool Node::check_and_set_extrinsics() {
  if (extrinsics_set) {
    return true;
  }
  const std::optional<Sophus::SE3d> imu_transform = ros_utils::get_transform(tf_buffer, imu_frame, base_frame, 0s);
  if (!imu_transform) {
    return false;
  }
  const std::optional<Sophus::SE3d> lidar_transform = ros_utils::get_transform(tf_buffer, lidar_frame, base_frame, 0s);
  if (!lidar_transform) {
    return false;
  }
  extrinsic_imu2base = imu_transform.value();
  extrinsic_lidar2base = lidar_transform.value();
  extrinsics_set = true;
  return true;
}

void Node::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) {
  if (imu_frame.empty()) {
    if (imu_msg->header.frame_id.empty() && !extrinsics_set) {
      throw std::runtime_error("IMU message header has no frame id and we need it to query TF for the extrinsics. "
                               "Either specify the frame id or the extrinsic manually.");
    }
    imu_frame = imu_msg->header.frame_id;
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed the imu frame id as: " << imu_frame);
  }
  if (!check_and_set_extrinsics()) {
    // we assume that extrinsics are static. if they change, its better to query the tf directly in the registration
    // loop for each message being processed asynchronously.
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    imu_buffer.emplace(imu_msg_to_imu_data(*imu_msg));
    atomic_can_process = !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
  }
  if (atomic_can_process) {
    sync_condition_variable.notify_one();
  }
}

void Node::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg) {
  if (lidar_frame.empty()) {
    if (lidar_msg->header.frame_id.empty() && !extrinsics_set) {
      throw std::runtime_error("LiDAR message header has no frame id and we need it to query TF for the extrinsics. "
                               "Either specify the frame id or the extrinsic manually.");
    }
    lidar_frame = lidar_msg->header.frame_id;
    RCLCPP_INFO_STREAM(node->get_logger(), "Parsed the lidar frame id as: " << lidar_frame);
  }
  if (!check_and_set_extrinsics()) {
    return;
  }
  {
    std::lock_guard lock(buffer_mutex);
    if (lidar_buffer.size() >= max_lidar_buffer_size) {
      RCLCPP_WARN_STREAM(node->get_logger(), "Registration lidar buffer limit reached. Dropping frame.");
      sync_condition_variable.notify_one();
      return;
    }
  }
  try {
    const auto& [start_stamp, end_stamp, timestamps, scan] =
        std::invoke([&]() -> std::tuple<core::Secondsd, core::Secondsd, core::TimestampVector, core::Vector3dVector> {
          const auto& header_stamp = ros_utils::ros_time_to_seconds(lidar_msg->header.stamp);
          if (lio->config.deskew) {
            const auto& [scan, raw_timestamps] = ros_utils::point_cloud2_to_eigen_with_timestamps(lidar_msg);
            const auto& [start_stamp, end_stamp, timestamp_vector] =
                core::process_timestamps(raw_timestamps, header_stamp);
            return {start_stamp, end_stamp, timestamp_vector, scan};
          } else {
            RCLCPP_WARN_STREAM_ONCE(node->get_logger(),
                                    "Deskewing is disabled. Populating timestamps with static header time.");
            const core::Vector3dVector scan = ros_utils::point_cloud2_to_eigen(lidar_msg);
            return {header_stamp, header_stamp, core::TimestampVector(scan.size(), header_stamp), scan};
          }
        });

    {
      std::lock_guard lock(buffer_mutex);
      lidar_buffer.emplace(start_stamp, end_stamp, timestamps, scan);
      atomic_can_process = !imu_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
    }
    if (atomic_can_process) {
      sync_condition_variable.notify_one();
    }
  } catch (const std::invalid_argument& ex) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Encountered error, dropping frame: Error. " << ex.what());
  }
}

void Node::registration_loop() {
  while (rclcpp::ok() && atomic_node_running) {
    SCOPED_PROFILER("ROS Registration Loop");
    std::unique_lock buffer_lock(buffer_mutex);
    sync_condition_variable.wait(buffer_lock, [this]() { return !atomic_node_running || atomic_can_process; });
    if (!atomic_node_running) {
      // node could have been killed after waiting on the cv
      break;
    }
    core::LidarFrame frame = std::move(lidar_buffer.front());
    lidar_buffer.pop();
    const auto& [start_stamp, end_stamp, timestamps, scan] = frame;
    for (; !imu_buffer.empty() && imu_buffer.front().time < end_stamp; imu_buffer.pop()) {
      const core::ImuControl& imu_data = imu_buffer.front();
      lio->add_imu_measurement(extrinsic_imu2base, imu_data);
    }
    // check if there are more messages buffered already
    atomic_can_process =
        !imu_buffer.empty() && !lidar_buffer.empty() && imu_buffer.back().time > lidar_buffer.front().end;
    buffer_lock.unlock(); // we dont touch the buffers anymore

    try {
      const core::Vector3dVector deskewed_frame = std::invoke([&]() {
        if (publish_local_map) {
          std::lock_guard lock(local_map_mutex); // publish_map thread might access simultaneously
          return lio->register_scan(extrinsic_lidar2base, scan, timestamps);
        } else {
          return lio->register_scan(extrinsic_lidar2base, scan, timestamps);
        }
      });

      if (!deskewed_frame.empty()) {
        // TODO: first frame is skipped and an empty frame is returned. improve how we handle this
        if (publish_deskewed_scan) {
          std_msgs::msg::Header header;
          header.frame_id = lidar_frame;
          header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(end_stamp).count());
          frame_publisher->publish(ros_utils::eigen_to_point_cloud2(deskewed_frame, header));
        }
        publish_odometry(lio->lidar_state, end_stamp);
        if (publish_lidar_acceleration) {
          publish_lidar_accel(lio->lidar_state.linear_acceleration, end_stamp);
        }
      }
    } catch (const std::invalid_argument& ex) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Encountered error, dropping frame. Error: " << ex.what());
    }
  }
  atomic_node_running = false;
}

void Node::publish_odometry(const core::State& state, const core::Secondsd& stamp) const {
  const std::string_view from_frame = base_frame;
  const std::string_view to_frame = odom_frame;
  // tf message
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  if (invert_odom_tf) {
    transform_msg.header.frame_id = from_frame;
    transform_msg.child_frame_id = to_frame;
    transform_msg.transform = ros_utils::sophus_to_transform(state.pose.inverse());
  } else {
    transform_msg.header.frame_id = to_frame;
    transform_msg.child_frame_id = from_frame;
    transform_msg.transform = ros_utils::sophus_to_transform(state.pose);
  }
  tf_broadcaster->sendTransform(transform_msg);

  // odometry msg
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  odom_msg.header.frame_id = to_frame;
  odom_msg.child_frame_id = from_frame;
  odom_msg.pose.pose = ros_utils::sophus_to_pose(state.pose);
  ros_utils::eigen_vector3d_to_ros_xyz(state.velocity, odom_msg.twist.twist.linear);
  ros_utils::eigen_vector3d_to_ros_xyz(state.angular_velocity, odom_msg.twist.twist.angular);
  odom_publisher->publish(odom_msg);
}

void Node::publish_lidar_accel(const Eigen::Vector3d& acceleration, const core::Secondsd& stamp) const {
  auto accel_msg = geometry_msgs::msg::AccelStamped();
  accel_msg.header.stamp = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
  accel_msg.header.frame_id = base_frame;
  ros_utils::eigen_vector3d_to_ros_xyz(acceleration, accel_msg.accel.linear);
  lidar_accel_publisher->publish(accel_msg);
}

void Node::publish_map_loop() {
  while (atomic_node_running) {
    std::this_thread::sleep_for(publish_map_after);
    std::unique_lock lock(local_map_mutex);
    if (lio->map.Empty()) {
      RCLCPP_WARN_ONCE(node->get_logger(), "Local map publish thread: Local map is empty.");
      continue;
    }
    const core::Vector3dVector map_points = lio->map.Pointcloud();
    lock.unlock(); // we don't access the local map anymore
    std_msgs::msg::Header map_header;
    map_header.stamp = node->now();
    map_header.frame_id = odom_frame;
    map_publisher->publish(ros_utils::eigen_to_point_cloud2(map_points, map_header));
  }
}

Node::~Node() {
  atomic_node_running = false;
  sync_condition_variable.notify_all();
}
} // namespace rko_lio::ros
