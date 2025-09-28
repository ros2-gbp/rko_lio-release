// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "point_cloud_write.hpp"
#include <regex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;
using StampType = builtin_interfaces::msg::Time;

std::string FixFrameId(const std::string& frame_id) { return std::regex_replace(frame_id, std::regex("^/"), ""); }

PointCloud2::UniquePtr create_point_cloud2_msg(const size_t n_points, const Header& header, bool timestamp = false) {
  PointCloud2::UniquePtr cloud_msg = std::make_unique<PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
  cloud_msg->fields.clear();
  int offset = 0;
  offset = addPointField(*cloud_msg, "x", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "y", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "z", 1, PointField::FLOAT32, offset);
  offset += sizeOfPointField(PointField::FLOAT32);
  if (timestamp) {
    // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
    offset = addPointField(*cloud_msg, "time", 1, PointField::FLOAT64, offset);
    offset += sizeOfPointField(PointField::FLOAT64);
  }

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(static_cast<size_t>(cloud_msg->height) * cloud_msg->row_step);
  modifier.resize(n_points);
  return cloud_msg;
}

void fill_point_cloud2_xyz(const std::vector<Eigen::Vector3d>& points, PointCloud2& msg) {
  sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
  for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
    const Eigen::Vector3d& point = points[i];
    *msg_x = static_cast<float>(point.x());
    *msg_y = static_cast<float>(point.y());
    *msg_z = static_cast<float>(point.z());
  }
}
} // namespace
namespace rko_lio::ros_utils {

PointCloud2::UniquePtr eigen_to_point_cloud2(const std::vector<Eigen::Vector3d>& points, const Header& header) {
  PointCloud2::UniquePtr msg = create_point_cloud2_msg(points.size(), header);
  fill_point_cloud2_xyz(points, *msg);
  return msg;
}

} // namespace rko_lio::ros_utils
