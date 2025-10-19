// Copyright 2010-2013, A. Hornung, M. Philips. All rights reserved.
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

#ifndef OCTOMAP_SERVER__OCTOMAP_SERVER_MULTILAYER_HPP_
#define OCTOMAP_SERVER__OCTOMAP_SERVER_MULTILAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <octomap_server/octomap_server.hpp>

namespace octomap_server
{
class OctomapServerMultilayer : public OctomapServer
{
public:
  explicit OctomapServerMultilayer(const rclcpp::NodeOptions & node_options);

protected:
  struct ProjectedMap
  {
    double min_z;
    double max_z;
    double z;  // for visualization
    std::string name;
    OccupancyGrid map;
  };
  using MultilevelGrid = std::vector<ProjectedMap>;

  /// hook that is called after traversing all nodes
  virtual void handlePreNodeTraversal(const rclcpp::Time & rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator & it, bool occupied);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const rclcpp::Time & rostime);

  /// callback for attached collision objects from MoveIt2
  void attachedObjectCallback(const moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr msg);

  /// update arm links from attached objects in planning scene
  void updateArmLinksFromAttachedObjects();

private:
  /// Initialize MoveIt2 integration (called after construction via timer)
  void initializeMoveIt2();

  /// Initialize legacy hardcoded arm links (PR2 robot)
  void initializeLegacyArmLinks();

  std::vector<rclcpp::Publisher<OccupancyGrid>::SharedPtr> multi_map_pub_;

  std::vector<std::string> arm_links_;
  std::vector<double> arm_link_offsets_;

  MultilevelGrid multi_gridmap_;

  // MoveIt2 integration
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_object_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;  // One-shot timer for deferred MoveIt2 initialization

  // Parameters
  bool use_moveit_attached_objects_;
  std::string planning_scene_topic_;

  // Thread safety for arm links access
  mutable std::mutex arm_links_mutex_;
};
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__OCTOMAP_SERVER_MULTILAYER_HPP_
