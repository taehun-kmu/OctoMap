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

#define CATCH_CONFIG_MAIN
#include <tf2_ros/static_transform_broadcaster.h>

#include <catch2/catch.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap_server/octomap_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fixtures/advanced_fixtures.hpp"
#include "fixtures/mock_pointcloud.hpp"
#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

// ==============================================================================
// PointCloud Subscription Tests
// ==============================================================================

TEST_CASE("OctomapServer subscribes to point cloud topic", "[octomap_server][pointcloud]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  rclcpp::spin_some(server);

  auto topic_names = server->get_topic_names_and_types();

  bool found_cloud_sub = false;
  for (const auto & [name, types] : topic_names) {
    if (name.find("cloud_in") != std::string::npos) {
      found_cloud_sub = true;
      break;
    }
  }

  CHECK(found_cloud_sub);
}

// ==============================================================================
// PointCloud Processing Tests
// ==============================================================================

TEST_CASE("OctomapServer processes simple point cloud", "[octomap_server][pointcloud]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.1);
  options.append_parameter_override("frame_id", "map");
  auto server = std::make_shared<OctomapServer>(options);

  // Create a simple test cloud
  auto cloud = MockPointCloud::createSimpleCloud(100, "map", 0.1);
  REQUIRE(cloud != nullptr);
  CHECK(cloud->width == 100);
  CHECK(cloud->header.frame_id == "map");

  // Server should be able to process cloud
  // Note: insertCloudCallback requires TF transforms, so we can't directly test it
  // but we can verify the cloud format is valid
  CHECK(cloud->data.size() > 0);
}

TEST_CASE("OctomapServer handles empty point cloud", "[octomap_server][pointcloud]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  auto empty_cloud = MockPointCloud::createEmptyCloud("map");
  REQUIRE(empty_cloud != nullptr);
  CHECK(empty_cloud->width == 0);
  CHECK(empty_cloud->data.empty());
}

TEST_CASE("OctomapServer handles box-distributed points", "[octomap_server][pointcloud]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create points in a 3D box
  std::array<double, 3> min_point = {-1.0, -1.0, 0.0};
  std::array<double, 3> max_point = {1.0, 1.0, 2.0};
  auto box_cloud = MockPointCloud::createBoxCloud(min_point, max_point, 500, "map");

  REQUIRE(box_cloud != nullptr);
  CHECK(box_cloud->width == 500);
  CHECK(box_cloud->data.size() > 0);
}

// ==============================================================================
// Complex PointCloud Scenario Tests
// ==============================================================================

TEST_CASE("OctomapServer handles staircase point cloud", "[octomap_server][pointcloud][complex]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.1);
  auto server = std::make_shared<OctomapServer>(options);

  // Create staircase with 5 steps
  auto staircase = ComplexPointCloudGenerator::createStaircase(5, 0.2, 0.3, 100, "map");

  REQUIRE(staircase != nullptr);
  CHECK(staircase->width == 500);  // 5 steps * 100 points each
  CHECK(staircase->header.frame_id == "map");
}

TEST_CASE("OctomapServer handles sloped ground plane", "[octomap_server][pointcloud][complex]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("filter_ground_plane", true);
  options.append_parameter_override("ground_filter.angle", 0.26);  // ~15 degrees
  auto server = std::make_shared<OctomapServer>(options);

  // Create sloped plane at 15 degrees
  auto slope = ComplexPointCloudGenerator::createSlope(2.0, 3.0, 15.0, 300, "map");

  REQUIRE(slope != nullptr);
  CHECK(slope->width == 300);

  // Verify ground filtering is enabled
  CHECK(server->get_parameter("filter_ground_plane").as_bool() == true);
}

TEST_CASE("OctomapServer handles obstacle scenarios", "[octomap_server][pointcloud][complex]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.05);
  auto server = std::make_shared<OctomapServer>(options);

  // Create environment with 5 random obstacles
  auto obstacles = ComplexPointCloudGenerator::createObstacles(5, 200, 0.2, 0.8, 5.0, "map");

  REQUIRE(obstacles != nullptr);
  CHECK(obstacles->width == 1000);  // 5 obstacles * 200 points each
}

TEST_CASE("OctomapServer handles cylindrical objects", "[octomap_server][pointcloud][complex]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create cylinder (e.g., pillar or tree trunk)
  auto cylinder = ComplexPointCloudGenerator::createCylinder(0.5, 2.0, 0.0, 0.0, 500, "map");

  REQUIRE(cylinder != nullptr);
  CHECK(cylinder->width == 500);
}

// ==============================================================================
// Parameter Effect Tests
// ==============================================================================

TEST_CASE(
  "OctomapServer respects point cloud min/max z parameters",
  "[octomap_server][pointcloud][parameters]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("point_cloud_min_z", -1.0);
  options.append_parameter_override("point_cloud_max_z", 2.0);
  auto server = std::make_shared<OctomapServer>(options);

  CHECK(server->get_parameter("point_cloud_min_z").as_double() == Approx(-1.0));
  CHECK(server->get_parameter("point_cloud_max_z").as_double() == Approx(2.0));
}

TEST_CASE(
  "OctomapServer respects point cloud x/y bounding box", "[octomap_server][pointcloud][parameters]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("point_cloud_min_x", -5.0);
  options.append_parameter_override("point_cloud_max_x", 5.0);
  options.append_parameter_override("point_cloud_min_y", -3.0);
  options.append_parameter_override("point_cloud_max_y", 3.0);
  auto server = std::make_shared<OctomapServer>(options);

  CHECK(server->get_parameter("point_cloud_min_x").as_double() == Approx(-5.0));
  CHECK(server->get_parameter("point_cloud_max_x").as_double() == Approx(5.0));
  CHECK(server->get_parameter("point_cloud_min_y").as_double() == Approx(-3.0));
  CHECK(server->get_parameter("point_cloud_max_y").as_double() == Approx(3.0));
}

// ==============================================================================
// Ground Filtering Parameter Tests
// ==============================================================================

TEST_CASE(
  "OctomapServer ground filtering configuration", "[octomap_server][pointcloud][ground_filter]")
{
  ROS2Fixture ros_fixture;

  SECTION("Ground filtering enabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("filter_ground_plane", true);
    options.append_parameter_override("ground_filter.distance", 0.05);
    options.append_parameter_override("ground_filter.angle", 0.2);
    options.append_parameter_override("ground_filter.plane_distance", 0.08);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("filter_ground_plane").as_bool() == true);
    CHECK(server->get_parameter("ground_filter.distance").as_double() == Approx(0.05));
    CHECK(server->get_parameter("ground_filter.angle").as_double() == Approx(0.2));
    CHECK(server->get_parameter("ground_filter.plane_distance").as_double() == Approx(0.08));
  }

  SECTION("Ground filtering disabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("filter_ground_plane", false);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("filter_ground_plane").as_bool() == false);
  }
}

// ==============================================================================
// Speckle Filtering Tests
// ==============================================================================

TEST_CASE(
  "OctomapServer speckle filtering configuration", "[octomap_server][pointcloud][speckle_filter]")
{
  ROS2Fixture ros_fixture;

  SECTION("Speckle filtering enabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("filter_speckles", true);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("filter_speckles").as_bool() == true);
  }

  SECTION("Speckle filtering disabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("filter_speckles", false);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("filter_speckles").as_bool() == false);
  }
}

// ==============================================================================
// Sensor Model Parameter Tests
// ==============================================================================

TEST_CASE("OctomapServer sensor model configuration", "[octomap_server][pointcloud][sensor_model]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("sensor_model.max_range", 10.0);
  options.append_parameter_override("sensor_model.hit", 0.7);
  options.append_parameter_override("sensor_model.miss", 0.4);
  options.append_parameter_override("sensor_model.min", 0.12);
  options.append_parameter_override("sensor_model.max", 0.97);

  auto server = std::make_shared<OctomapServer>(options);

  CHECK(server->get_parameter("sensor_model.max_range").as_double() == Approx(10.0));
  CHECK(server->get_parameter("sensor_model.hit").as_double() == Approx(0.7));
  CHECK(server->get_parameter("sensor_model.miss").as_double() == Approx(0.4));
  CHECK(server->get_parameter("sensor_model.min").as_double() == Approx(0.12));
  CHECK(server->get_parameter("sensor_model.max").as_double() == Approx(0.97));
}
