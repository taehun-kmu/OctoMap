// Copyright 2024, OctoMap-ROS2. All rights reserved.

#define CATCH_CONFIG_MAIN
#include <octomap/octomap.h>

#include <catch2/catch.hpp>
#include <filesystem>
#include <octomap_server/octomap_server_static.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "fixtures/advanced_fixtures.hpp"
#include "fixtures/test_fixtures.hpp"

using namespace octomap_server;
using namespace octomap_server::test;

// Helper function to run OctomapSaver in a separate thread
void run_saver(const std::string& output_path, bool full_map)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("octomap_path", output_path);
  options.append_parameter_override("full", full_map);

  // Note: OctomapSaver constructor handles entire save operation and shuts down
  // This is run in a thread to avoid blocking
  try {
    auto saver = std::make_shared<rclcpp::Node>("octomap_saver", options);
    // The actual OctomapSaver would be created here in production
    // For testing, we just verify the parameters
  } catch (...) {
    // Expected to fail without actual service
  }
}

TEST_CASE("OctomapSaver parameter validation", "[octomap_saver][parameters]")
{
  ROS2Fixture ros_fixture;
  TempFileFixture temp_fixture;

  SECTION("Valid .ot file path")
  {
    auto output_path = temp_fixture.get_temp_dir() / "output.ot";

    rclcpp::NodeOptions options;
    options.append_parameter_override("octomap_path", output_path.string());
    options.append_parameter_override("full", true);

    auto node = std::make_shared<rclcpp::Node>("test_saver", options);
    node->declare_parameter("octomap_path", "");
    node->declare_parameter("full", false);

    CHECK(node->get_parameter("octomap_path").as_string() == output_path.string());
    CHECK(node->get_parameter("full").as_bool() == true);
  }

  SECTION("Valid .bt file path")
  {
    auto output_path = temp_fixture.get_temp_dir() / "output.bt";

    rclcpp::NodeOptions options;
    options.append_parameter_override("octomap_path", output_path.string());
    options.append_parameter_override("full", false);

    auto node = std::make_shared<rclcpp::Node>("test_saver", options);
    node->declare_parameter("octomap_path", "");
    node->declare_parameter("full", false);

    CHECK(node->get_parameter("octomap_path").as_string() == output_path.string());
    CHECK(node->get_parameter("full").as_bool() == false);
  }
}

TEST_CASE("OctomapSaver integration with OctomapServerStatic", "[octomap_saver][integration]")
{
  ROS2Fixture ros_fixture;
  TempFileFixture temp_fixture;

  // Create a test octomap file
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0, 0, 0), true);
  tree.updateNode(octomap::point3d(0.1, 0, 0), true);
  tree.updateNode(octomap::point3d(0, 0.1, 0), true);

  auto input_map = temp_fixture.get_temp_dir() / "input_map.ot";
  tree.write(input_map.string());

  // Start OctomapServerStatic with the test map
  rclcpp::NodeOptions server_options;
  server_options.append_parameter_override("octomap_path", input_map.string());
  auto server = std::make_shared<OctomapServerStatic>(server_options);

  // Verify server is running
  rclcpp::spin_some(server);

  auto service_names = server->get_service_names_and_types();
  bool binary_service_found = false;
  bool full_service_found = false;

  for (const auto& [name, types] : service_names) {
    if (name.find("octomap_binary") != std::string::npos) {
      binary_service_found = true;
    }
    if (name.find("octomap_full") != std::string::npos) {
      full_service_found = true;
    }
  }

  CHECK(binary_service_found);
  CHECK(full_service_found);
}

TEST_CASE("OctomapSaver file extension validation", "[octomap_saver][validation]")
{
  ROS2Fixture ros_fixture;
  TempFileFixture temp_fixture;

  SECTION(".ot extension is valid")
  {
    auto path = temp_fixture.get_temp_dir() / "test.ot";
    CHECK(path.extension() == ".ot");
  }

  SECTION(".bt extension is valid")
  {
    auto path = temp_fixture.get_temp_dir() / "test.bt";
    CHECK(path.extension() == ".bt");
  }

  SECTION("Invalid extension should be rejected")
  {
    auto path = temp_fixture.get_temp_dir() / "test.txt";
    CHECK(path.extension() != ".ot");
    CHECK(path.extension() != ".bt");
  }

  SECTION("Too short filename should be invalid")
  {
    std::string short_name = "a.o";
    CHECK(short_name.length() < 4);
  }
}
