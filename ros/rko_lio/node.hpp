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

#pragma once
#include "rko_lio/core/lio.hpp"
// stl
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
// ros
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rko_lio::ros {
class Node {
public:
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<core::LIO> lio;

  std::string imu_topic;
  std::string imu_frame = ""; // default: get from the first imu message
  std::string lidar_topic;
  std::string lidar_frame = ""; // default: get from the first lidar message
  std::string base_frame;
  std::string odom_frame = "odom";
  std::string odom_topic = "/rko_lio/odometry";
  std::string map_topic = "/rko_lio/local_map";
  std::string results_dir = "results";
  std::string run_name = "rko_lio_run";

  bool invert_odom_tf = false;
  bool publish_lidar_acceleration = false;
  bool publish_deskewed_scan = false;
  bool publish_local_map = false;

  Sophus::SE3d extrinsic_imu2base;
  Sophus::SE3d extrinsic_lidar2base;
  bool extrinsics_set = false;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr lidar_accel_publisher;

  // multithreading
  std::jthread map_publish_thead;
  core::Secondsd publish_map_after = std::chrono::seconds(1);
  std::mutex local_map_mutex;

  std::jthread registration_thread;
  std::mutex buffer_mutex;
  std::condition_variable sync_condition_variable;
  std::atomic<bool> atomic_node_running = true;
  std::atomic<bool> atomic_can_process = false;
  std::queue<core::ImuControl> imu_buffer;
  std::queue<core::LidarFrame> lidar_buffer;
  size_t max_lidar_buffer_size = 50;

  Node() = delete;
  Node(const std::string& node_name, const rclcpp::NodeOptions& options);

  void parse_cli_extrinsics();
  bool check_and_set_extrinsics();
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg);
  void registration_loop();
  void publish_odometry(const core::State& state, const core::Secondsd& stamp) const;
  void publish_lidar_accel(const Eigen::Vector3d& acceleration, const core::Secondsd& stamp) const;
  void publish_map_loop();

  ~Node();
  Node(const Node&) = delete;
  Node(Node&&) = delete;
  Node& operator=(const Node&) = delete;
  Node& operator=(Node&&) = delete;
};
} // namespace rko_lio::ros
