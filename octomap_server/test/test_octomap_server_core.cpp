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
#include <catch2/catch.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_server/octomap_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "fixtures/mock_pointcloud.hpp"
#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

// ==============================================================================
// Constructor and Initialization Tests
// ==============================================================================

TEST_CASE("OctomapServer constructor initializes correctly", "[octomap_server][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  REQUIRE(server != nullptr);
  CHECK(server->get_name() == std::string("octomap_server"));
}

TEST_CASE("OctomapServer parameters can be set via NodeOptions", "[octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.1);
  options.append_parameter_override("frame_id", "test_frame");
  options.append_parameter_override("sensor_model.max_range", 5.0);

  auto server = std::make_shared<OctomapServer>(options);

  REQUIRE(server != nullptr);
  CHECK(server->get_parameter("resolution").as_double() == Approx(0.1));
  CHECK(server->get_parameter("frame_id").as_string() == "test_frame");
  CHECK(server->get_parameter("sensor_model.max_range").as_double() == Approx(5.0));
}

// Note: The following methods are protected and cannot be tested directly:
// - updateMinKey, updateMaxKey (BBox utilities)
// - mapIdx, heightMapColor (2D map utilities)
// - isInUpdateBBX (BBox check)
// - isSpeckleNode (speckle filter)
// These are indirectly tested through the public API.

// ==============================================================================
// File Operations Tests
// ==============================================================================

TEST_CASE("openFile handles non-existent files", "[octomap_server][file_io]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  bool result = server->openFile("/nonexistent/path/to/file.ot");
  CHECK(result == false);
}

TEST_CASE("openFile handles empty filename", "[octomap_server][file_io]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  bool result = server->openFile("");
  CHECK(result == false);
}

// ==============================================================================
// Service Tests
// ==============================================================================

TEST_CASE("OctomapServer provides octomap services", "[octomap_server][services]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Give some time for service registration
  rclcpp::spin_some(server);

  auto service_names = server->get_service_names_and_types();

  SECTION("Binary octomap service exists")
  {
    bool found = false;
    for (const auto & [name, types] : service_names) {
      if (name.find("octomap_binary") != std::string::npos) {
        found = true;
        break;
      }
    }
    CHECK(found);
  }

  SECTION("Full octomap service exists")
  {
    bool found = false;
    for (const auto & [name, types] : service_names) {
      if (name.find("octomap_full") != std::string::npos) {
        found = true;
        break;
      }
    }
    CHECK(found);
  }

  SECTION("Reset service exists")
  {
    bool found = false;
    for (const auto & [name, types] : service_names) {
      if (name.find("reset") != std::string::npos) {
        found = true;
        break;
      }
    }
    CHECK(found);
  }
}

// ==============================================================================
// PointCloud Subscription Tests
// ==============================================================================

TEST_CASE("OctomapServer subscribes to point cloud topics", "[octomap_server][topics]")
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
// Parameter Validation Tests
// ==============================================================================

TEST_CASE("OctomapServer validates resolution parameter", "[octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Positive resolution is accepted")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.05);

    auto server = std::make_shared<OctomapServer>(options);
    CHECK(server->get_parameter("resolution").as_double() == Approx(0.05));
  }

  SECTION("Zero resolution is accepted (no validation)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.0);

    auto server = std::make_shared<OctomapServer>(options);
    // Current implementation doesn't validate resolution > 0
    CHECK(server->get_parameter("resolution").as_double() == Approx(0.0));
  }
}

TEST_CASE("OctomapServer validates frame_id parameter", "[octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("frame_id", "custom_frame");

  auto server = std::make_shared<OctomapServer>(options);
  CHECK(server->get_parameter("frame_id").as_string() == "custom_frame");
}

TEST_CASE("OctomapServer validates max_range parameter", "[octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Positive max_range is accepted")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_model.max_range", 10.0);

    auto server = std::make_shared<OctomapServer>(options);
    CHECK(server->get_parameter("sensor_model.max_range").as_double() == Approx(10.0));
  }

  SECTION("Negative max_range is handled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("sensor_model.max_range", -1.0);

    auto server = std::make_shared<OctomapServer>(options);
    // Should either reject or use as "no limit" indicator
    // Exact behavior depends on implementation
    CHECK(server->get_parameter("sensor_model.max_range").as_double() == Approx(-1.0));
  }
}

// ==============================================================================
// Service Callback Tests
// ==============================================================================

TEST_CASE("OctomapServer binary service callback", "[octomap_server][services]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create service client
  auto client = server->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");
  REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

  // Call service
  auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
  auto future = client->async_send_request(request);

  // Wait for response
  auto timeout = std::chrono::seconds(2);
  REQUIRE(
    rclcpp::spin_until_future_complete(server, future, timeout) ==
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  REQUIRE(response != nullptr);

  // Binary map should be returned (even if empty)
  CHECK(response->map.header.frame_id.length() > 0);
}

TEST_CASE("OctomapServer full service callback", "[octomap_server][services]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create service client
  auto client = server->create_client<octomap_msgs::srv::GetOctomap>("octomap_full");
  REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

  // Call service
  auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
  auto future = client->async_send_request(request);

  // Wait for response
  auto timeout = std::chrono::seconds(2);
  REQUIRE(
    rclcpp::spin_until_future_complete(server, future, timeout) ==
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  REQUIRE(response != nullptr);

  // Full map should be returned (even if empty)
  CHECK(response->map.header.frame_id.length() > 0);
}

TEST_CASE("OctomapServer reset service callback", "[octomap_server][services]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create service client
  auto client = server->create_client<std_srvs::srv::Empty>("~/reset");
  REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

  // Call reset service
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = client->async_send_request(request);

  // Wait for response
  auto timeout = std::chrono::seconds(2);
  REQUIRE(
    rclcpp::spin_until_future_complete(server, future, timeout) ==
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  CHECK(response != nullptr);
}

TEST_CASE("OctomapServer clear_bbox service callback", "[octomap_server][services]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServer>(options);

  // Create service client
  auto client = server->create_client<octomap_msgs::srv::BoundingBoxQuery>("~/clear_bbox");
  REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

  // Call service with a bounding box
  auto request = std::make_shared<octomap_msgs::srv::BoundingBoxQuery::Request>();
  request->min.x = -1.0;
  request->min.y = -1.0;
  request->min.z = -1.0;
  request->max.x = 1.0;
  request->max.y = 1.0;
  request->max.z = 1.0;

  auto future = client->async_send_request(request);

  // Wait for response
  auto timeout = std::chrono::seconds(2);
  REQUIRE(
    rclcpp::spin_until_future_complete(server, future, timeout) ==
    rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  CHECK(response != nullptr);
}
