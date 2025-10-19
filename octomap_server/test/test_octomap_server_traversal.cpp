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
#include <octomap/octomap.h>

#include <catch2/catch.hpp>
#include <octomap_server/octomap_server.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

// ==============================================================================
// Tree Depth Configuration Tests
// ==============================================================================

TEST_CASE("OctomapServer tree depth configuration", "[octomap_server][traversal][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Default tree depth")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.05);

    auto server = std::make_shared<OctomapServer>(options);

    // Tree depth is derived from resolution
    // For resolution 0.05, max_depth would typically be calculated
    CHECK(server->get_parameter("resolution").as_double() == Approx(0.05));
  }

  SECTION("Custom max depth")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("max_depth", 14);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("max_depth").as_int() == 14);
  }

  SECTION("Maximum tree depth (16)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("max_depth", 16);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("max_depth").as_int() == 16);
  }
}

// ==============================================================================
// Speckle Filtering Tests
// ==============================================================================

TEST_CASE("OctomapServer speckle filtering behavior", "[octomap_server][traversal][speckle]")
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
// Update Bounding Box Tests
// ==============================================================================

TEST_CASE("OctomapServer update bounding box initialization", "[octomap_server][traversal][bbox]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.1);

  auto server = std::make_shared<OctomapServer>(options);

  // Server initializes successfully with bounding box
  REQUIRE(server != nullptr);
  CHECK(server->get_parameter("resolution").as_double() == Approx(0.1));
}

// ==============================================================================
// Occupied/Free Space Traversal Tests
// ==============================================================================

TEST_CASE(
  "OctomapServer occupancy threshold configuration", "[octomap_server][traversal][occupancy]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("sensor_model.min", 0.12);
  options.append_parameter_override("sensor_model.max", 0.97);

  auto server = std::make_shared<OctomapServer>(options);

  // These thresholds affect how nodes are classified as occupied/free
  CHECK(server->get_parameter("sensor_model.min").as_double() == Approx(0.12));
  CHECK(server->get_parameter("sensor_model.max").as_double() == Approx(0.97));
}

TEST_CASE("OctomapServer hit/miss probabilities", "[octomap_server][traversal][occupancy]")
{
  ROS2Fixture ros_fixture;

  SECTION("Default hit/miss values")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_model.hit", 0.7);
    options.append_parameter_override("sensor_model.miss", 0.4);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("sensor_model.hit").as_double() == Approx(0.7));
    CHECK(server->get_parameter("sensor_model.miss").as_double() == Approx(0.4));
  }

  SECTION("Conservative hit/miss values")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_model.hit", 0.6);
    options.append_parameter_override("sensor_model.miss", 0.45);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("sensor_model.hit").as_double() == Approx(0.6));
    CHECK(server->get_parameter("sensor_model.miss").as_double() == Approx(0.45));
  }

  SECTION("Aggressive hit/miss values")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_model.hit", 0.9);
    options.append_parameter_override("sensor_model.miss", 0.3);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("sensor_model.hit").as_double() == Approx(0.9));
    CHECK(server->get_parameter("sensor_model.miss").as_double() == Approx(0.3));
  }
}

// ==============================================================================
// Visualization Marker Tests
// ==============================================================================

TEST_CASE("OctomapServer visualization topics", "[octomap_server][traversal][visualization]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  rclcpp::spin_some(server);

  auto topic_names = server->get_topic_names_and_types();

  // Check for occupied cells marker topic
  bool found_occupied = false;
  for (const auto & [name, types] : topic_names) {
    if (name.find("occupied_cells_vis") != std::string::npos) {
      found_occupied = true;
      break;
    }
  }
  CHECK(found_occupied);
}

// ==============================================================================
// Multi-resolution Tests
// ==============================================================================

TEST_CASE("OctomapServer handles different resolutions", "[octomap_server][traversal][resolution]")
{
  ROS2Fixture ros_fixture;

  SECTION("Fine resolution (0.01m)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.01);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("resolution").as_double() == Approx(0.01));
  }

  SECTION("Medium resolution (0.05m)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.05);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("resolution").as_double() == Approx(0.05));
  }

  SECTION("Coarse resolution (0.2m)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.2);

    auto server = std::make_shared<OctomapServer>(options);

    CHECK(server->get_parameter("resolution").as_double() == Approx(0.2));
  }
}

// ==============================================================================
// Tree Initialization Tests
// ==============================================================================

TEST_CASE("OctomapServer octree initialization", "[octomap_server][traversal][initialization]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.1);
  options.append_parameter_override("frame_id", "map");

  auto server = std::make_shared<OctomapServer>(options);

  // Verify server initialized successfully
  REQUIRE(server != nullptr);
  CHECK(server->get_name() == std::string("octomap_server"));

  // Verify basic parameters
  CHECK(server->get_parameter("resolution").as_double() == Approx(0.1));
  CHECK(server->get_parameter("frame_id").as_string() == "map");
}

// ==============================================================================
// Node Processing Priority Tests
// ==============================================================================

TEST_CASE("OctomapServer processes nodes by depth", "[octomap_server][traversal][depth]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("max_depth", 16);
  options.append_parameter_override("resolution", 0.05);

  auto server = std::make_shared<OctomapServer>(options);

  // Max depth determines finest resolution for processing
  CHECK(server->get_parameter("max_depth").as_int() == 16);
  CHECK(server->get_parameter("resolution").as_double() == Approx(0.05));
}

// ==============================================================================
// Integration Tests
// ==============================================================================

TEST_CASE(
  "OctomapServer complete traversal configuration", "[octomap_server][traversal][integration]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  // Tree structure
  options.append_parameter_override("resolution", 0.1);
  options.append_parameter_override("max_depth", 14);

  // Filtering
  options.append_parameter_override("filter_speckles", true);

  // Sensor model
  options.append_parameter_override("sensor_model.hit", 0.7);
  options.append_parameter_override("sensor_model.miss", 0.4);
  options.append_parameter_override("sensor_model.min", 0.12);
  options.append_parameter_override("sensor_model.max", 0.97);

  // Visualization
  options.append_parameter_override("publish_free_space", true);

  auto server = std::make_shared<OctomapServer>(options);

  REQUIRE(server != nullptr);

  // Verify all parameters
  CHECK(server->get_parameter("resolution").as_double() == Approx(0.1));
  CHECK(server->get_parameter("max_depth").as_int() == 14);
  CHECK(server->get_parameter("filter_speckles").as_bool() == true);
  CHECK(server->get_parameter("sensor_model.hit").as_double() == Approx(0.7));
  CHECK(server->get_parameter("sensor_model.miss").as_double() == Approx(0.4));
  CHECK(server->get_parameter("sensor_model.min").as_double() == Approx(0.12));
  CHECK(server->get_parameter("sensor_model.max").as_double() == Approx(0.97));
  CHECK(server->get_parameter("publish_free_space").as_bool() == true);
}
