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
#include <catch2/catch.hpp>
#include <octomap_server/octomap_server_multilayer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

TEST_CASE(
  "OctomapServerMultilayer constructor initializes correctly",
  "[octomap_server_multilayer][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServerMultilayer>(options);

  REQUIRE(server != nullptr);
  // OctomapServerMultilayer inherits from OctomapServer, so node name is "octomap_server"
  CHECK(server->get_name() == std::string("octomap_server"));
}

TEST_CASE(
  "OctomapServerMultilayer supports MoveIt2 integration", "[octomap_server_multilayer][moveit2]")
{
  ROS2Fixture ros_fixture;

  SECTION("MoveIt2 mode enabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_moveit_attached_objects", true);
    options.append_parameter_override("robot_description", "test_robot");

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("use_moveit_attached_objects").as_bool() == true);
  }

  SECTION("Legacy mode (MoveIt2 disabled)")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_moveit_attached_objects", false);

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("use_moveit_attached_objects").as_bool() == false);
  }
}

TEST_CASE(
  "OctomapServerMultilayer publishes multi-level maps", "[octomap_server_multilayer][topics]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServerMultilayer>(options);

  rclcpp::spin_some(server);

  auto topic_names = server->get_topic_names_and_types();

  // Multilayer server should publish multiple 2D maps
  int map_publisher_count = 0;
  for (const auto & [name, types] : topic_names) {
    if (name.find("map") != std::string::npos) {
      map_publisher_count++;
    }
  }

  // Should have at least one map publisher
  CHECK(map_publisher_count > 0);
}

TEST_CASE("OctomapServerMultilayer layer configuration", "[octomap_server_multilayer][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Custom layer heights")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("base_layer.min_z", 0.0);
    options.append_parameter_override("base_layer.max_z", 0.3);
    options.append_parameter_override("spine_layer.min_z", 0.25);
    options.append_parameter_override("spine_layer.max_z", 1.4);
    options.append_parameter_override("arm_layer.min_z", 0.7);
    options.append_parameter_override("arm_layer.max_z", 0.9);

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("base_layer.min_z").as_double() == Approx(0.0));
    CHECK(server->get_parameter("base_layer.max_z").as_double() == Approx(0.3));
    CHECK(server->get_parameter("spine_layer.min_z").as_double() == Approx(0.25));
    CHECK(server->get_parameter("spine_layer.max_z").as_double() == Approx(1.4));
    CHECK(server->get_parameter("arm_layer.min_z").as_double() == Approx(0.7));
    CHECK(server->get_parameter("arm_layer.max_z").as_double() == Approx(0.9));
  }
}
