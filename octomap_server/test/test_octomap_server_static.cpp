// Copyright 2024, OctoMap. All rights reserved.
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
//    * Neither the name of the Willow Garage nor the names of its
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
#include <octomap_server/octomap_server_static.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fixtures/advanced_fixtures.hpp"
#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

TEST_CASE(
  "OctomapServerStatic constructor initializes correctly", "[octomap_server_static][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServerStatic>(options);

  REQUIRE(server != nullptr);
  CHECK(server->get_name() == std::string("octomap_server_static"));
}

TEST_CASE(
  "OctomapServerStatic loads static map from file parameter", "[octomap_server_static][file_io]")
{
  ROS2Fixture ros_fixture;

  SECTION("Non-existent file parameter")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("map_file", "/nonexistent/map.ot");

    auto server = std::make_shared<OctomapServerStatic>(options);
    REQUIRE(server != nullptr);
    // Server should initialize but log error about missing file
  }

  SECTION("Empty file parameter")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("map_file", "");

    auto server = std::make_shared<OctomapServerStatic>(options);
    REQUIRE(server != nullptr);
  }
}

TEST_CASE("OctomapServerStatic provides octomap services", "[octomap_server_static][services]")
{
  ROS2Fixture ros_fixture;
  TempFileFixture temp_fixture;

  // Create a simple octomap file
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0, 0, 0), true);
  auto map_file = temp_fixture.get_temp_dir() / "test_map.ot";
  tree.write(map_file.string());

  rclcpp::NodeOptions options;
  options.append_parameter_override("octomap_path", map_file.string());
  auto server = std::make_shared<OctomapServerStatic>(options);

  // Services are created in constructor, no need to spin
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
}

TEST_CASE(
  "OctomapServerStatic does not subscribe to point clouds", "[octomap_server_static][topics]")
{
  ROS2Fixture ros_fixture;
  TempFileFixture temp_fixture;

  // Create a simple octomap file
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0, 0, 0), true);
  auto map_file = temp_fixture.get_temp_dir() / "test_map.ot";
  tree.write(map_file.string());

  rclcpp::NodeOptions options;
  options.append_parameter_override("octomap_path", map_file.string());
  auto server = std::make_shared<OctomapServerStatic>(options);

  rclcpp::spin_some(server);

  auto topic_names = server->get_topic_names_and_types();

  // Static server should NOT subscribe to cloud_in (it's static!)
  bool found_cloud_sub = false;
  for (const auto & [name, types] : topic_names) {
    if (name.find("cloud_in") != std::string::npos) {
      found_cloud_sub = true;
      break;
    }
  }

  CHECK_FALSE(found_cloud_sub);
}
