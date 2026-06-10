// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2024 Black Coffee Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
#define VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace vector_pursuit_controller
{

/**
 * @class vector_pursuit_controller::VectorPursuitController
 * @brief Vector Pursuit path-tracking controller plugin (Wit, 2006)
 */
class VectorPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for vector_pursuit_controller::VectorPursuitController
   */
  VectorPursuitController() = default;

  /**
   * @brief Destructor for vector_pursuit_controller::VectorPursuitController
   */
  ~VectorPursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best velocity command given the current pose and velocity.
   *
   * @param pose          Current robot pose (in odom frame)
   * @param velocity      Current measured robot velocity
   * @param goal_checker  Ptr to the goal checker for this task; used to read goal tolerances
   * @param transformed_global_plan The global plan after being processed by the path handler
   * @param global_goal   The last pose of the global plan
   * @return              Twist command (linear.x, angular.z)
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/,
    const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;

  /**
   * @brief nav2_core newPathReceived - Receives a new plan from the Planner Server
   * @param raw_global_path The global plan from the Planner Server
   */
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;

  /**
   * @brief Get the orientation of the line segment between two points
   * @param p1 The first point
   * @param p2 The second point
   * @return The angle between the two points
   */
  geometry_msgs::msg::Quaternion getOrientation(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  /**
   * @brief Calculate the turning radius of the robot given a target point
   * @param target_pose The target pose to calculate the turning radius
   * @return The turning radius of the robot
   */
  double calcTurningRadius(
    const geometry_msgs::msg::PoseStamped & target_pose);

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @param target_dist Distance to the lookahead point
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const double & linear_vel, const double & angular_vel,
    const double & target_dist);

  /**
    * @brief Get lookahead point
    * @param lookahead_dist Optimal lookahead distance
    * @param path Current global path
    * @return Lookahead point
    * @details The lookahead point could:
    * 1. Be discrete/interpolated
    * 2. Have a heading:
    *    a. Directly from path (discrete)
    *    b. Directly from path (interpolated) (if use_interpolation_ is true)
    *    c. Computed from path (if use_heading_from_path_ is false)
    *
    * Hence the flags use_interpolation_ and use_heading_from_path_
    * play important roles here.
    */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  double approachVelocityScalingFactor(
    const nav_msgs::msg::Path & path
  ) const;

  /**
   * @brief Apply approach velocity scaling to the system
   * @param path Transformed global path
   * @param linear_vel robot command linear velocity input
   */
  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path,
    double & linear_vel
  ) const;

  /**
   * @brief Apply cost and curvature constraints to the system
   * @param curvature curvature of path
   * @param curr_speed Speed of robot
   * @param pose_cost cost at this pose
   * @param linear_vel robot command linear velocity input
   * @param path Transformed global path
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost, double & linear_vel, const nav_msgs::msg::Path & path, double & sign);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param target_pose current lookahead point
   * @param angle_to_path Angle of robot output relative to lookahead point
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & target_pose,
    double & angle_to_path, double & sign);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param target_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & target_pose);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double getCuspDist(const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relative to lookahead point
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Limit an angular velocity command to a feasible angular
   * acceleration about the current angular speed
   * @param angular_vel desired angular velocity
   * @param curr_angular_vel the current angular speed
   * @return angular velocity clamped to [curr - a*dt, curr + a*dt]
   */
  double applyAngularAccelerationLimit(
    double angular_vel, double curr_angular_vel) const;

  /**
   * @brief Apply angular braking to stop at desired angle
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relative to lookahead point
   * @param curr_speed the current robot speed
   */
  void applyAngularBraking(
    double & angular_vel, const double & angle_to_path,
    const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  nav2::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("VectorPursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double k_;
  double desired_linear_vel_, base_desired_linear_vel_;
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double min_turning_radius_;
  double min_linear_velocity_;
  double max_lateral_accel_;
  double control_duration_;
  double max_allowed_time_to_collision_up_to_target_;
  bool use_collision_detection_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double max_linear_accel_;
  double rotate_to_heading_min_angle_;
  double goal_dist_tol_;
  bool use_interpolation_;
  bool allow_reversing_;
  bool use_heading_from_path_;

  geometry_msgs::msg::Twist last_cmd_vel_;

  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  nav2::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr target_arc_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace vector_pursuit_controller

#endif  // VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
