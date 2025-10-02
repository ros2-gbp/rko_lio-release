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
#include <Eigen/Core>
#include <chrono>
#include <sophus/se3.hpp>

namespace Eigen {
using Matrix3_6d = Matrix<double, 3, 6>;
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
using Matrix12d = Matrix<double, 12, 12>;
} // namespace Eigen

namespace rko_lio::core {
// aliases
using Vector3dVector = std::vector<Eigen::Vector3d>;
using Secondsd = std::chrono::duration<double>;
using TimestampVector = std::vector<Secondsd>;

// constants and util funcs
constexpr double square(double x) { return x * x; }
constexpr double GRAVITY_MAG = 9.8107;
inline Eigen::Vector3d gravity() { return {0, 0, -GRAVITY_MAG}; }

// data structs
struct State {
  Secondsd time{0};
  Sophus::SE3d pose;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

struct ImuBias {
  Eigen::Vector3d accelerometer = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();
};

struct ImuControl {
  Secondsd time{0};
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct LidarFrame {
  Secondsd start;
  Secondsd end;
  TimestampVector timestamps;
  Vector3dVector points;
};
}; // namespace rko_lio::core
