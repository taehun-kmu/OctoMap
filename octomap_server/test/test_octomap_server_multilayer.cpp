// Copyright 2024, OctoMap-ROS2. All rights reserved.

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap_server/octomap_server_multilayer.hpp>

#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

TEST_CASE("OctomapServerMultilayer constructor initializes correctly",
  "[octomap_server_multilayer][constructor]")
{
  ROS2Fixture ros_fixture;

  rclcpp::NodeOptions options;
  auto server = std::make_shared<OctomapServerMultilayer>(options);

  REQUIRE(server != nullptr);
  CHECK(server->get_name() == std::string("octomap_server_multilayer"));
}

TEST_CASE("OctomapServerMultilayer supports MoveIt2 integration",
  "[octomap_server_multilayer][moveit2]")
{
  ROS2Fixture ros_fixture;

  SECTION("MoveIt2 mode enabled") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_moveit_attached_objects", true);
    options.append_parameter_override("robot_description", "test_robot");

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("use_moveit_attached_objects").as_bool() == true);
  }

  SECTION("Legacy mode (MoveIt2 disabled)") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_moveit_attached_objects", false);

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("use_moveit_attached_objects").as_bool() == false);
  }
}

TEST_CASE("OctomapServerMultilayer publishes multi-level maps",
  "[octomap_server_multilayer][topics]")
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

TEST_CASE("OctomapServerMultilayer layer configuration",
  "[octomap_server_multilayer][parameters]")
{
  ROS2Fixture ros_fixture;

  SECTION("Custom layer heights") {
    rclcpp::NodeOptions options;
    options.append_parameter_override("ground_layer.min_z", 0.0);
    options.append_parameter_override("ground_layer.max_z", 0.3);
    options.append_parameter_override("arm_layer.min_z", 0.7);
    options.append_parameter_override("arm_layer.max_z", 0.9);

    auto server = std::make_shared<OctomapServerMultilayer>(options);
    REQUIRE(server != nullptr);

    CHECK(server->get_parameter("ground_layer.min_z").as_double() == Approx(0.0));
    CHECK(server->get_parameter("ground_layer.max_z").as_double() == Approx(0.3));
    CHECK(server->get_parameter("arm_layer.min_z").as_double() == Approx(0.7));
    CHECK(server->get_parameter("arm_layer.max_z").as_double() == Approx(0.9));
  }
}
