cmake_minimum_required(VERSION 3.8)
project(octomap_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OCTOMAP REQUIRED)

# MoveIt2 dependencies for multilayer server
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

ament_auto_add_library(octomap_server_static SHARED
  src/octomap_server_static.cpp
)

target_link_libraries(octomap_server_static
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_include_directories(octomap_server_static
  PUBLIC
    ${OCTOMAP_INCLUDE_DIRS}
)

rclcpp_components_register_node(octomap_server_static
  PLUGIN "octomap_server::OctomapServerStatic"
  EXECUTABLE octomap_server_static_node
)

ament_auto_add_library(octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(octomap_server
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server_node
)

ament_auto_add_library(octomap_server_multilayer SHARED
  src/octomap_server.cpp
  src/octomap_server_multilayer.cpp
)

target_link_libraries(octomap_server_multilayer
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  moveit_core::moveit_planning_scene
  moveit_ros_planning::moveit_planning_scene_monitor
  tf2_geometry_msgs::tf2_geometry_msgs
)

rclcpp_components_register_node(octomap_server_multilayer
  PLUGIN "octomap_server::OctomapServerMultilayer"
  EXECUTABLE octomap_server_multilayer_node
)

ament_auto_add_library(tracking_octomap_server SHARED
  src/octomap_server.cpp
  src/tracking_octomap_server.cpp
)

target_link_libraries(tracking_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(tracking_octomap_server
  PLUGIN "octomap_server::TrackingOctomapServer"
  EXECUTABLE tracking_octomap_server_node
)

ament_auto_add_library(color_octomap_server SHARED
  src/octomap_server.cpp
)

target_link_libraries(color_octomap_server
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(color_octomap_server
  PLUGIN "octomap_server::ColorOctomapServer"
  EXECUTABLE color_octomap_server_node
)

target_compile_definitions(color_octomap_server PUBLIC COLOR_OCTOMAP_SERVER)

ament_auto_add_library(octomap_saver SHARED
  src/octomap_saver.cpp
)

target_link_libraries(octomap_saver
  ${OCTOMAP_LIBRARIES}
)

rclcpp_components_register_node(octomap_saver
  PLUGIN "octomap_server::OctomapSaver"
  EXECUTABLE octomap_saver_node
)

if(BUILD_TESTING)
  # Exclude auto-generated files from linting
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  # Code quality tools with custom config files
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)

  ament_clang_format(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format
    "include"
    "src"
    "test"
  )

  ament_clang_tidy(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-tidy
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  # Complementary C++ static analysis
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(
    EXCLUDE "${CMAKE_CURRENT_BINARY_DIR}/*"
  )

  # Python linting (for scripts)
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()

  # Standard linting tools
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(
    EXCLUDE "${CMAKE_CURRENT_BINARY_DIR}/*"
  )
  ament_lint_cmake()
  ament_xmllint()

  # Catch2 test framework
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(catch_ros2 REQUIRED)

  # Helper macro for creating Catch2 tests
  macro(octomap_add_catch2_test test_name test_source)
    ament_add_catch2(${test_name}
      ${test_source}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(${test_name}
      ${ARGN}
    )
    target_include_directories(${test_name}
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  endmacro()

  # Core OctomapServer tests
  octomap_add_catch2_test(test_octomap_server_core
    test/test_octomap_server_core.cpp
    octomap_server
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapServerStatic tests
  octomap_add_catch2_test(test_octomap_server_static
    test/test_octomap_server_static.cpp
    octomap_server_static
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapServerMultilayer tests
  octomap_add_catch2_test(test_octomap_server_multilayer
    test/test_octomap_server_multilayer.cpp
    octomap_server_multilayer
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    moveit_core::moveit_planning_scene
    moveit_ros_planning::moveit_planning_scene_monitor
    tf2_geometry_msgs::tf2_geometry_msgs
  )

  # TrackingOctomapServer tests
  octomap_add_catch2_test(test_tracking_octomap_server
    test/test_tracking_octomap_server.cpp
    tracking_octomap_server
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapSaver tests
  octomap_add_catch2_test(test_octomap_saver
    test/test_octomap_saver.cpp
    octomap_saver
    octomap_server_static
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapServer PointCloud processing tests
  octomap_add_catch2_test(test_octomap_server_pointcloud
    test/test_octomap_server_pointcloud.cpp
    octomap_server
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapServer 2D projection tests
  octomap_add_catch2_test(test_octomap_server_2d_projection
    test/test_octomap_server_2d_projection.cpp
    octomap_server
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  # OctomapServer tree traversal tests
  octomap_add_catch2_test(test_octomap_server_traversal
    test/test_octomap_server_traversal.cpp
    octomap_server
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)

install(PROGRAMS
  scripts/octomap_eraser_cli.py
  DESTINATION lib/${PROJECT_NAME}
)
