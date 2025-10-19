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

#ifndef OCTOMAP_SERVER__TEST__FIXTURES__MOCK_POINTCLOUD_HPP_
#define OCTOMAP_SERVER__TEST__FIXTURES__MOCK_POINTCLOUD_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdlib>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace octomap_server
{
namespace test
{

/**
 * @brief Helper class for creating mock PointCloud2 messages for testing
 */
class MockPointCloud
{
public:
  /**
   * @brief Create a simple PointCloud2 with XYZ points in a grid pattern
   * @param num_points Number of points to generate
   * @param frame_id Frame ID for the cloud
   * @param spacing Spacing between points (meters)
   * @return Shared pointer to generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createSimpleCloud(
    size_t num_points = 100, const std::string & frame_id = "base_link", double spacing = 0.1)
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = num_points;
    cloud->is_dense = true;
    cloud->is_bigendian = false;

    // Define fields
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    // Populate data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    size_t grid_size = static_cast<size_t>(std::ceil(std::sqrt(num_points)));
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = (i % grid_size) * spacing;
      *iter_y = (i / grid_size) * spacing;
      *iter_z = 0.5;  // All points at 0.5m height
    }

    return cloud;
  }

  /**
   * @brief Create a PointCloud2 with RGB color information
   * @param num_points Number of points to generate
   * @param frame_id Frame ID for the cloud
   * @return Shared pointer to generated colored PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createColoredCloud(
    size_t num_points = 100, const std::string & frame_id = "base_link")
  {
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl_cloud.width = num_points;
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = true;

    size_t grid_size = static_cast<size_t>(std::ceil(std::sqrt(num_points)));
    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZRGB point;
      point.x = (i % grid_size) * 0.1;
      point.y = (i / grid_size) * 0.1;
      point.z = 0.5;
      point.r = (i * 10) % 256;
      point.g = (i * 20) % 256;
      point.b = (i * 30) % 256;
      pcl_cloud.points.push_back(point);
    }

    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl_cloud, *cloud);
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();

    return cloud;
  }

  /**
   * @brief Create a PointCloud2 with points distributed in a 3D box
   * @param min_point Minimum corner of the box
   * @param max_point Maximum corner of the box
   * @param num_points Number of points to generate
   * @param frame_id Frame ID for the cloud
   * @return Shared pointer to generated PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createBoxCloud(
    const std::array<double, 3> & min_point, const std::array<double, 3> & max_point,
    size_t num_points = 1000, const std::string & frame_id = "base_link")
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

    // Distribute points uniformly in 3D box
    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = min_point[0] + (max_point[0] - min_point[0]) * (rand() / (RAND_MAX + 1.0));
      *iter_y = min_point[1] + (max_point[1] - min_point[1]) * (rand() / (RAND_MAX + 1.0));
      *iter_z = min_point[2] + (max_point[2] - min_point[2]) * (rand() / (RAND_MAX + 1.0));
    }

    return cloud;
  }

  /**
   * @brief Create an empty PointCloud2
   * @param frame_id Frame ID for the cloud
   * @return Shared pointer to empty PointCloud2
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr createEmptyCloud(
    const std::string & frame_id = "base_link")
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = rclcpp::Clock().now();
    cloud->height = 1;
    cloud->width = 0;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    return cloud;
  }
};

}  // namespace test
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__TEST__FIXTURES__MOCK_POINTCLOUD_HPP_
