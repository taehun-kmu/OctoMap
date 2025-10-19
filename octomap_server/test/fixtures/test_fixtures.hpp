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

#ifndef OCTOMAP_SERVER__TEST__FIXTURES__TEST_FIXTURES_HPP_
#define OCTOMAP_SERVER__TEST__FIXTURES__TEST_FIXTURES_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace octomap_server
{
namespace test
{

/**
 * @brief RAII-style ROS2 initialization fixture for Catch2 tests
 */
class ROS2Fixture
{
public:
  ROS2Fixture()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      initialized_ = true;
    }
  }

  ~ROS2Fixture()
  {
    if (initialized_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Delete copy/move to enforce single initialization
  ROS2Fixture(const ROS2Fixture &) = delete;
  ROS2Fixture & operator=(const ROS2Fixture &) = delete;
  ROS2Fixture(ROS2Fixture &&) = delete;
  ROS2Fixture & operator=(ROS2Fixture &&) = delete;

private:
  bool initialized_{false};
};

}  // namespace test
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__TEST__FIXTURES__TEST_FIXTURES_HPP_
