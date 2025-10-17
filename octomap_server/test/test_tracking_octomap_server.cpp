// Copyright 2024, OctoMap-ROS2. All rights reserved.

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap_server/tracking_octomap_server.hpp>

#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

TEST_CASE("TrackingOctomapServer constructor initializes correctly",
  "[tracking_octomap_server][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<TrackingOctomapServer>(options);

  REQUIRE(server != nullptr);
  CHECK(server->get_name() == std::string("tracking_octomap_server"));
}

TEST_CASE("TrackingOctomapServer tracking configuration",
  "[tracking_octomap_server][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Track changes enabled") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("track_changes", true);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("track_changes").as_bool() == true);
  }

  SECTION("Track changes disabled") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("track_changes", false);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("track_changes").as_bool() == false);
  }

  SECTION("Listen to changes") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("listen_changes", true);

    auto server = std::make_shared<TrackingOctomapServer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("listen_changes").as_bool() == true);
  }
}

TEST_CASE("TrackingOctomapServer change detection topics",
  "[tracking_octomap_server][topics]")
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
