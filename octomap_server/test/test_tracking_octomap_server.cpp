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
#include <octomap_server/tracking_octomap_server.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

TEST_CASE(
  "TrackingOctomapServer constructor initializes correctly",
  "[tracking_octomap_server][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<TrackingOctomapServer>(options);

  REQUIRE(server != nullptr);
  // TrackingOctomapServer inherits from OctomapServer, so node name is "octomap_server"
  CHECK(server->get_name() == std::string("octomap_server"));
}

TEST_CASE("TrackingOctomapServer tracking configuration", "[tracking_octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Track changes enabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("track_changes", true);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("track_changes").as_bool() == true);
  }

  SECTION("Track changes disabled")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("track_changes", false);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("track_changes").as_bool() == false);
  }

  SECTION("Listen to changes")
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("listen_changes", true);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("listen_changes").as_bool() == true);
  }
}

TEST_CASE("TrackingOctomapServer change detection topics", "[tracking_octomap_server][topics]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  options.append_parameter_override("track_changes", true);

  auto server = std::make_shared<TrackingOctomapServer>(options);

  rclcpp::spin_some(server);

  auto topic_names = server->get_topic_names_and_types();

  // Should publish change detection topic
  bool found_changes_pub = false;
  for (const auto & [name, types] : topic_names) {
    if (name.find("change") != std::string::npos) {
      found_changes_pub = true;
      break;
    }
  }

  CHECK(found_changes_pub);
}
