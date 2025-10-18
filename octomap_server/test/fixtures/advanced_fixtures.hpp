// Copyright 2024, OctoMap-ROS2. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef OCTOMAP_SERVER__TEST__FIXTURES__ADVANCED_FIXTURES_HPP_
#define OCTOMAP_SERVER__TEST__FIXTURES__ADVANCED_FIXTURES_HPP_

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace octomap_server
{
namespace test
{

/**
 * @brief RAII-style temporary file/directory fixture for file I/O testing
 */
class TempFileFixture
{
public:
  TempFileFixture() : temp_dir_(std::filesystem::temp_directory_path() / generate_unique_name())
  {
    std::filesystem::create_directories(temp_dir_);
  }

  ~TempFileFixture()
  {
    if (std::filesystem::exists(temp_dir_)) {
      std::filesystem::remove_all(temp_dir_);
    }
  }

  // Delete copy/move
  TempFileFixture(const TempFileFixture &) = delete;
  TempFileFixture & operator=(const TempFileFixture &) = delete;
  TempFileFixture(TempFileFixture &&) = delete;
  TempFileFixture & operator=(TempFileFixture &&) = delete;

  /**
   * @brief Get the temporary directory path
   */
  std::filesystem::path get_temp_dir() const
  {
    return temp_dir_;
  }

  /**
   * @brief Create a temporary file with given content
   * @param filename Filename (not path)
   * @param content File content
   * @return Full path to created file
   */
  std::filesystem::path create_file(const std::string & filename, const std::string & content = "")
  {
    auto filepath = temp_dir_ / filename;
    std::ofstream file(filepath);
    if (!content.empty()) {
      file << content;
    }
    file.close();
    return filepath;
  }

  /**
   * @brief Create a subdirectory
   * @param dirname Directory name
   * @return Path to created directory
   */
  std::filesystem::path create_subdir(const std::string & dirname)
  {
    auto dirpath = temp_dir_ / dirname;
    std::filesystem::create_directories(dirpath);
    return dirpath;
  }

  /**
   * @brief Check if a file exists in temp directory
   */
  bool file_exists(const std::string & filename) const
  {
    return std::filesystem::exists(temp_dir_ / filename);
  }

  /**
   * @brief Read file content
   */
  std::string read_file(const std::string & filename) const
  {
    std::ifstream file(temp_dir_ / filename);
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  }

private:
  std::filesystem::path temp_dir_;

  static std::string generate_unique_name()
  {
    auto now = std::chrono::system_clock::now();
    auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return "octomap_test_" + std::to_string(timestamp);
  }
};

/**
 * @brief Advanced PointCloud generator for complex scenarios
 */
class ComplexPointCloudGenerator
{
public:
  /**
   * @brief Generate a staircase point cloud
   * @param num_steps Number of steps
   * @param step_height Height of each step (meters)
   * @param step_depth Depth of each step (meters)
   * @param points_per_step Points to generate per step
   * @param frame_id Frame ID for the cloud
   * @return Generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createStaircase(
    size_t num_steps = 5, double step_height = 0.2, double step_depth = 0.3,
    size_t points_per_step = 100, const std::string & frame_id = "base_link")
  {
    size_t total_points = num_steps * points_per_step;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = total_points;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    for (size_t step = 0; step < num_steps; ++step) {
      double base_z = step * step_height;
      double base_x = step * step_depth;

      for (size_t i = 0; i < points_per_step; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = base_x + (rand() / static_cast<double>(RAND_MAX)) * step_depth;
        *iter_y = -0.5 + (rand() / static_cast<double>(RAND_MAX));
        *iter_z = base_z;
      }
    }

    return cloud;
  }

  /**
   * @brief Generate a sloped ground plane
   * @param width Width of the plane
   * @param length Length of the plane
   * @param angle Slope angle in degrees
   * @param num_points Number of points
   * @param frame_id Frame ID
   * @return Generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createSlope(
    double width = 2.0, double length = 3.0, double angle_degrees = 15.0, size_t num_points = 500,
    const std::string & frame_id = "base_link")
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = num_points;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    double angle_rad = angle_degrees * M_PI / 180.0;

    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      double x = (rand() / static_cast<double>(RAND_MAX)) * length;
      double y = -width / 2.0 + (rand() / static_cast<double>(RAND_MAX)) * width;
      double z = x * std::tan(angle_rad);

      *iter_x = x;
      *iter_y = y;
      *iter_z = z;
    }

    return cloud;
  }

  /**
   * @brief Generate a point cloud with obstacles
   * @param num_obstacles Number of box obstacles
   * @param points_per_obstacle Points per obstacle
   * @param min_size Minimum obstacle size
   * @param max_size Maximum obstacle size
   * @param arena_size Size of the arena
   * @param frame_id Frame ID
   * @return Generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createObstacles(
    size_t num_obstacles = 5, size_t points_per_obstacle = 200, double min_size = 0.2,
    double max_size = 0.8, double arena_size = 5.0, const std::string & frame_id = "base_link")
  {
    size_t total_points = num_obstacles * points_per_obstacle;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = total_points;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    for (size_t obs = 0; obs < num_obstacles; ++obs) {
      // Random obstacle position and size
      double cx = (rand() / static_cast<double>(RAND_MAX)) * arena_size;
      double cy = -arena_size / 2.0 + (rand() / static_cast<double>(RAND_MAX)) * arena_size;
      double cz = (rand() / static_cast<double>(RAND_MAX)) * 2.0;
      double size = min_size + (rand() / static_cast<double>(RAND_MAX)) * (max_size - min_size);

      for (size_t i = 0; i < points_per_obstacle; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = cx + ((rand() / static_cast<double>(RAND_MAX)) - 0.5) * size;
        *iter_y = cy + ((rand() / static_cast<double>(RAND_MAX)) - 0.5) * size;
        *iter_z = cz + ((rand() / static_cast<double>(RAND_MAX)) - 0.5) * size;
      }
    }

    return cloud;
  }

  /**
   * @brief Generate a cylindrical object
   * @param radius Cylinder radius
   * @param height Cylinder height
   * @param center_x X position
   * @param center_y Y position
   * @param num_points Number of points
   * @param frame_id Frame ID
   * @return Generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createCylinder(
    double radius = 0.5, double height = 2.0, double center_x = 0.0, double center_y = 0.0,
    size_t num_points = 500, const std::string & frame_id = "base_link")
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = num_points;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      double angle = (rand() / static_cast<double>(RAND_MAX)) * 2.0 * M_PI;
      double r = radius * std::sqrt(rand() / static_cast<double>(RAND_MAX));
      double z = (rand() / static_cast<double>(RAND_MAX)) * height;

      *iter_x = center_x + r * std::cos(angle);
      *iter_y = center_y + r * std::sin(angle);
      *iter_z = z;
    }

    return cloud;
  }
};

}  // namespace test
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__TEST__FIXTURES__ADVANCED_FIXTURES_HPP_
