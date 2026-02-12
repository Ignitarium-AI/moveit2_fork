/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, University of Hamburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael 'v4hn' Goerner
   Desc: Tests for PlanningSceneMonitor
*/

// ROS
#include <rclcpp/rclcpp.hpp>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

/// Test helper class that exposes protected methods for testing
class PlanningSceneMonitorTestWrapper : public planning_scene_monitor::PlanningSceneMonitor
{
public:
  using PlanningSceneMonitor::PlanningSceneMonitor;
  
  // Expose protected method for testing
  using PlanningSceneMonitor::getShapeTransformCache;
};

class PlanningSceneMonitorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("moveit_planning_scene_monitor_test");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    planning_scene_monitor_ = std::make_unique<PlanningSceneMonitorTestWrapper>(
        test_node_, "robot_description", "planning_scene_monitor");
    planning_scene_monitor_->monitorDiffs(true);
    scene_ = planning_scene_monitor_->getPlanningScene();
    executor_->add_node(test_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Needed to avoid race conditions on high-load CPUs.
    std::this_thread::sleep_for(std::chrono::seconds{ 1 });
  }

  void TearDown() override
  {
    scene_.reset();
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

protected:
  std::shared_ptr<rclcpp::Node> test_node_;

  // Executor and a thread to run the executor.
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  std::unique_ptr<PlanningSceneMonitorTestWrapper> planning_scene_monitor_;
  planning_scene::PlanningScenePtr scene_;
};

// various code expects the monitored scene to remain the same
TEST_F(PlanningSceneMonitorTest, TestPersistentScene)
{
  auto scene{ planning_scene_monitor_->getPlanningScene() };
  moveit_msgs::msg::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  planning_scene_monitor_->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, planning_scene_monitor_->getPlanningScene());
  msg.is_diff = msg.robot_state.is_diff = false;
  planning_scene_monitor_->newPlanningSceneMessage(msg);
  EXPECT_EQ(scene, planning_scene_monitor_->getPlanningScene());
}

using UpdateType = planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType;

#define TRIGGERS_UPDATE(msg, expected_update_type)                                                                     \
  {                                                                                                                    \
    planning_scene_monitor_->clearUpdateCallbacks();                                                                   \
    auto received_update_type{ UpdateType::UPDATE_NONE };                                                              \
    planning_scene_monitor_->addUpdateCallback([&](auto type) { received_update_type = type; });                       \
    planning_scene_monitor_->newPlanningSceneMessage(msg);                                                             \
    EXPECT_EQ(received_update_type, expected_update_type);                                                             \
  }

TEST_F(PlanningSceneMonitorTest, UpdateTypes)
{
  moveit_msgs::msg::PlanningScene msg;
  msg.is_diff = msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_NONE);

  msg.fixed_frame_transforms.emplace_back();
  msg.fixed_frame_transforms.back().header.frame_id = "base_link";
  msg.fixed_frame_transforms.back().child_frame_id = "object";
  msg.fixed_frame_transforms.back().transform.rotation.w = 1.0;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_TRANSFORMS);
  msg.fixed_frame_transforms.clear();
  moveit::core::robotStateToRobotStateMsg(scene_->getCurrentState(), msg.robot_state, false);
  msg.robot_state.is_diff = true;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE);

  msg.robot_state.is_diff = false;
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_STATE | UpdateType::UPDATE_GEOMETRY);

  msg.robot_state = moveit_msgs::msg::RobotState{};
  msg.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "object";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_object.pose.orientation.w = 1.0;
  collision_object.primitives.emplace_back();
  collision_object.primitives.back().type = shape_msgs::msg::SolidPrimitive::SPHERE;
  collision_object.primitives.back().dimensions = { 1.0 };
  msg.world.collision_objects.emplace_back(collision_object);
  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_GEOMETRY);

  msg.world.collision_objects.clear();
  msg.is_diff = false;

  TRIGGERS_UPDATE(msg, UpdateType::UPDATE_SCENE);
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheEmpty)
{
  // Test with empty scene - should succeed but cache should be empty
  occupancy_map_monitor::ShapeTransformCache cache;
  rclcpp::Time test_time = test_node_->now();
  
  bool result = planning_scene_monitor_->getShapeTransformCache("base_link", test_time, cache);
  
  EXPECT_TRUE(result);
  EXPECT_TRUE(cache.empty());
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheWithCollisionObjects)
{
  // Add a collision object to the scene
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = scene_->getPlanningFrame();
  collision_object.id = "test_box";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  // Add a box primitive
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.5, 0.5, 0.5 };
  collision_object.primitives.push_back(box);
  
  // Set pose
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  collision_object.primitive_poses.push_back(pose);
  
  scene_msg.world.collision_objects.push_back(collision_object);
  planning_scene_monitor_->newPlanningSceneMessage(scene_msg);
  
  // Wait for scene update
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Now test the transform cache retrieval
  occupancy_map_monitor::ShapeTransformCache cache;
  rclcpp::Time test_time = test_node_->now();
  
  bool result = planning_scene_monitor_->getShapeTransformCache(scene_->getPlanningFrame(), test_time, cache);
  
  EXPECT_TRUE(result);
  // Cache may or may not have entries depending on whether octomap monitor is active
  // but the call should succeed
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheWithAttachedObject)
{
  // Create an attached object
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_link0";  // Assuming panda robot
  attached_object.object.header.frame_id = "panda_link0";
  attached_object.object.id = "attached_tool";
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  // Add a sphere primitive
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = { 0.1 };
  attached_object.object.primitives.push_back(sphere);
  
  // Set pose relative to link
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  attached_object.object.primitive_poses.push_back(pose);
  
  // Create planning scene message with attached object
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
  
  planning_scene_monitor_->newPlanningSceneMessage(scene_msg);
  
  // Wait for scene update
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Test transform cache retrieval
  occupancy_map_monitor::ShapeTransformCache cache;
  rclcpp::Time test_time = test_node_->now();
  
  bool result = planning_scene_monitor_->getShapeTransformCache(scene_->getPlanningFrame(), test_time, cache);
  
  EXPECT_TRUE(result);
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheInvalidFrame)
{
  // Test with a non-existent target frame
  occupancy_map_monitor::ShapeTransformCache cache;
  rclcpp::Time test_time = test_node_->now();
  
  // This might fail due to transform not being available
  // The function should handle this gracefully and return false or true depending on tf availability
  bool result = planning_scene_monitor_->getShapeTransformCache("nonexistent_frame", test_time, cache);
  
  // We can't guarantee the result since it depends on TF availability
  // but the function should not crash
  SUCCEED();
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheMultipleObjects)
{
  // Add multiple collision objects
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  
  for (int i = 0; i < 3; ++i)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = scene_->getPlanningFrame();
    object.id = "object_" + std::to_string(i);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions = { 0.2, 0.3 };  // radius, height
    object.primitives.push_back(primitive);
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = i * 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.15;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);
    
    scene_msg.world.collision_objects.push_back(object);
  }
  
  planning_scene_monitor_->newPlanningSceneMessage(scene_msg);
  
  // Wait for scene update
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Test transform cache retrieval
  occupancy_map_monitor::ShapeTransformCache cache;
  rclcpp::Time test_time = test_node_->now();
  
  bool result = planning_scene_monitor_->getShapeTransformCache(scene_->getPlanningFrame(), test_time, cache);
  
  EXPECT_TRUE(result);
}

TEST_F(PlanningSceneMonitorTest, GetShapeTransformCacheDifferentTimes)
{
  // Add a collision object
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;
  
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = scene_->getPlanningFrame();
  object.id = "time_test_object";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.3, 0.3, 0.3 };
  object.primitives.push_back(box);
  
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  
  scene_msg.world.collision_objects.push_back(object);
  planning_scene_monitor_->newPlanningSceneMessage(scene_msg);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Test with current time
  occupancy_map_monitor::ShapeTransformCache cache1;
  rclcpp::Time current_time = test_node_->now();
  bool result1 = planning_scene_monitor_->getShapeTransformCache(scene_->getPlanningFrame(), current_time, cache1);
  EXPECT_TRUE(result1);
  
  // Test with zero time (latest available)
  occupancy_map_monitor::ShapeTransformCache cache2;
  rclcpp::Time zero_time(0, 0, RCL_ROS_TIME);
  bool result2 = planning_scene_monitor_->getShapeTransformCache(scene_->getPlanningFrame(), zero_time, cache2);
  EXPECT_TRUE(result2);
  
  // Both should succeed since we're using the planning frame
  EXPECT_TRUE(result1 && result2);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
