// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "path_utils/path_utils.hpp"
#include "vector_pursuit_controller/vector_pursuit_controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class Controller : public vector_pursuit_controller::VectorPursuitController
{
public:
  Controller()
  : vector_pursuit_controller::VectorPursuitController() {}

  double getSpeed() {return desired_linear_vel_;}

  double getMinTurningRadius() {return min_turning_radius_;}
  void setMinTurningRadius(double r) {min_turning_radius_ = r;}

  void setVelocityScaledLookAhead() {use_velocity_scaled_lookahead_dist_ = true;}
  void setCostRegulationScaling() {use_cost_regulated_linear_velocity_scaling_ = true;}

  double getLookAheadDistanceWrapper(const geometry_msgs::msg::Twist & twist)
  {
    return getLookAheadDistance(twist);
  }

  double calcTurningRadiusWrappper(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    return calcTurningRadius(target_pose);
  }

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
  }

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & target_pose, double & angle_to_path, double & sign)
  {
    return shouldRotateToPath(target_pose, angle_to_path, sign);
  }

  double applyAngularAccelerationLimitWrapper(double angular_vel, double curr_angular_vel)
  {
    return applyAngularAccelerationLimit(angular_vel, curr_angular_vel);
  }

  void rotateToHeadingWrapper(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
  {
    return rotateToHeading(linear_vel, angular_vel, angle_to_path, curr_speed);
  }

  void applyConstraintsWrapper(
    const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
    const double & pose_cost, double & linear_vel, nav_msgs::msg::Path & path)
  {
    double sign = 1.0;
    return applyConstraints(
      curvature, curr_speed, pose_cost,
      linear_vel, path, sign);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommandsWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker,
    const nav_msgs::msg::Path & transformed_plan,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    return computeVelocityCommands(pose, speed, goal_checker, transformed_plan, goal);
  }
};

TEST(VectorPursuitTest, basicAPI)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<Controller>();
  costmap->on_configure(rclcpp_lifecycle::State());
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->deactivate();
  ctrl->cleanup();

  // newPathReceived is a no-op now (path handling is done by the controller
  // server's path handler), but it should be safe to call.
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->newPathReceived(path);

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(30, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.3);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}

TEST(VectorPursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::Twist twist;

  // test getLookAheadDistance
  double rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);  // default lookahead_dist

  // shouldn't be a function of speed
  twist.linear.x = 10.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);

  // now it should be a function of velocity, max out
  ctrl->setVelocityScaledLookAhead();
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.9);  // 10 speed maxes out at max_lookahead_dist

  // check normal range
  twist.linear.x = 0.35;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_NEAR(rtn, 0.525, 0.0001);  // 1.5 * 0.35

  // check minimum range
  twist.linear.x = 0.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.3);

  // test getLookAheadPoint
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test getting next closest point without interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test end of path
  dist = 100.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 9.0);

  // Test without use heading from path
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);

  dist = 4.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(true)));
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_interpolation",
      rclcpp::ParameterValue(true)));
  ctrl->configure(node, name, tf, costmap);
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);

  // Test without use heading from path with interpolation
  node->set_parameter(
    rclcpp::Parameter(
      name + ".use_heading_from_path",
      rclcpp::ParameterValue(false)));
  ctrl->configure(node, name, tf, costmap);

  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);

  // Path not starting at the robot pose
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i + 1);
  }

  // lookahead is measured along the path from its first pose (matching
  // nav2_util::getLookAheadPoint), so a path starting away from the robot
  // interpolates within its first segment rather than snapping to pose 0
  dist = 0.7;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.7);

  // If no pose is far enough, take the last pose discretely
  dist = 11.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 10.0);
}

TEST(VectorPursuitTest, angularAccelerationLimit) {
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  // max_angular_accel default 3.2 rad/s^2; control_duration default 1/20 = 0.05 s
  // => max delta per cycle = 0.16 rad/s
  ctrl->configure(node, name, tf, costmap);

  // within one step's reach: passes through unchanged
  EXPECT_NEAR(ctrl->applyAngularAccelerationLimitWrapper(0.1, 0.0), 0.1, 1e-9);

  // large positive jump from rest: clamped to +max delta
  EXPECT_NEAR(ctrl->applyAngularAccelerationLimitWrapper(5.0, 0.0), 0.16, 1e-9);

  // large negative jump from rest: clamped to -max delta
  EXPECT_NEAR(ctrl->applyAngularAccelerationLimitWrapper(-5.0, 0.0), -0.16, 1e-9);

  // clamp is relative to the current angular speed, not zero
  EXPECT_NEAR(ctrl->applyAngularAccelerationLimitWrapper(5.0, 1.0), 1.16, 1e-9);
}

TEST(VectorPursuitTest, calcTurnRadius) {
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::PoseStamped carrot;

  // directly ahead
  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.0;

  EXPECT_EQ(ctrl->calcTurningRadiusWrappper(carrot), std::numeric_limits<double>::max());

  // directly behind
  carrot.pose.position.x = -0.5;
  carrot.pose.position.y = 0.0;

  EXPECT_EQ(ctrl->calcTurningRadiusWrappper(carrot), ctrl->getMinTurningRadius());

  // diagonal
  carrot.pose.position.x = 0.1;
  carrot.pose.position.y = 0.1;

  ctrl->setMinTurningRadius(0.0);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.11, 0.01);

  // beside
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.1;

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.05, 0.01);

  // beside, heading reversed: the semicircle through (0, 0.1) naturally ends
  // pointing backward, so this is pure-pursuit-consistent and must return
  // exactly the pure-pursuit radius d^2 / (2y) = 0.05
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.1;

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -M_PI);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 0.05, 1e-6);

  // pure-pursuit consistency: when the target heading equals the natural end
  // heading of the tangent arc (2 * atan2(y, x)), the screw blend must reduce
  // to pure pursuit exactly, for any k: radius = d^2 / (2y)
  carrot.pose.position.x = 2.0;
  carrot.pose.position.y = 2.0;
  tf2_quat.setRPY(0, 0, M_PI / 2);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 2.0, 1e-6);

  // mirror symmetry: the reflected geometry gives the same magnitude with the
  // opposite sign (turning_radius is signed: -ve => screw center on the -y
  // side, i.e. a right turn toward the carrot)
  carrot.pose.position.y = -2.0;
  tf2_quat.setRPY(0, 0, -M_PI / 2);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), -2.0, 1e-6);

  // heading term dominates: a carrot slightly to the LEFT (y > 0) with a
  // desired heading that pulls hard the other way yields a screw center on
  // the far (-y) side, so the signed radius is negative even though y > 0.
  // computeVelocityCommands detects this (radius * y < 0) and rotates to
  // heading instead of arcing away from the path.
  carrot.pose.position.x = 2.0;
  carrot.pose.position.y = 0.1;
  tf2_quat.setRPY(0, 0, -0.75);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_LT(ctrl->calcTurningRadiusWrappper(carrot), 0.0);

  // near-straight with a leftward target heading: the heading term must pull
  // the radius down to a finite turn (k * phi / (k * phi + residual) * d^2/2y
  // with k = 8, phi = 2 * atan2(0.01, 1): radius ~= 12.5), not blow up or
  // fight the heading as the previous formulation did
  carrot.pose.position.x = 1.0;
  carrot.pose.position.y = 0.01;
  tf2_quat.setRPY(0, 0, 0.5);
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_NEAR(ctrl->calcTurningRadiusWrappper(carrot), 12.5, 0.1);

  // zero-total-rotation pole (k * phi + residual == 0): a pure-translation
  // screw — must degrade to straight-line motion, not divide to inf/NaN
  carrot.pose.position.x = 1.0;
  carrot.pose.position.y = std::tan(0.1);  // phi = 0.2
  tf2_quat.setRPY(0, 0, 0.2 - 8.0 * 0.2);  // residual = -k * phi
  carrot.pose.orientation = tf2::toMsg(tf2_quat);

  EXPECT_GT(ctrl->calcTurningRadiusWrappper(carrot), 1.0e6);
}

TEST(VectorPursuitTest, rotateTests)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn;
  double sign = 1.0;

  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, sign), true);

  // rotateToHeading
  double lin_v = 10.0;
  double ang_v = 0.5;
  double angle_to_path = 0.4;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.angular.z = 1.75;

  // basic full speed at a speed
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(lin_v, 0.0);
  EXPECT_EQ(ang_v, 1.8);

  // negative direction
  angle_to_path = -0.4;
  curr_speed.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, -1.8);

  // kinematic clamping, no speed, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.16, 0.01);

  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 1.16, 0.01);

  angle_to_path = -0.4;
  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.84, 0.01);
}

TEST(VectorPursuitTest, applyConstraints)
{
  auto ctrl = std::make_shared<Controller>();
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  constexpr double desired_linear_vel = 1.0;
  nav2::declare_parameter_if_not_declared(
    node,
    name + ".desired_linear_vel",
    rclcpp::ParameterValue(desired_linear_vel));

  ctrl->configure(node, name, tf, costmap);

  auto no_approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.6 + 1.0)
  });

  double curvature = 0.5;
  geometry_msgs::msg::Twist curr_speed;
  double pose_cost = 0.0;
  double linear_vel = desired_linear_vel;

  // test curvature regulation
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.35);  // max linear acceleration contraint

  linear_vel = 1.0;
  curvature = 3.6;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_LT(linear_vel, 0.5);  // lower by curvature

  linear_vel = 1.0;
  curvature = 1000.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.05);  // min out by curvature

  // Approach velocity scaling on a path with no distance left
  auto approach_path = path_utils::generate_path(
    geometry_msgs::msg::PoseStamped(), 0.1, {
    std::make_unique<path_utils::Straight>(0.0)
  });

  linear_vel = 1.0;
  curvature = 0.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(
    curvature, curr_speed, pose_cost, linear_vel, approach_path);
  EXPECT_NEAR(linear_vel, 0.05, 0.01);  // min out on min approach velocity

  // now try with cost regulation
  ctrl->setCostRegulationScaling();
  curvature = 0.0;

  // min changable cost
  pose_cost = 1;
  linear_vel = 0.5;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.498, 0.01);

  // max changing cost
  pose_cost = 127;
  curr_speed.linear.x = 0.255;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.280, 0.01);

  // over max cost thresh
  pose_cost = 200;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_NEAR(linear_vel, 0.086, 0.01);

  // test kinematic clamping
  pose_cost = 200;
  curr_speed.linear.x = 1.0;
  ctrl->applyConstraintsWrapper(curvature, curr_speed, pose_cost, linear_vel, no_approach_path);
  EXPECT_EQ(linear_vel, 0.05);
}

TEST(VectorPursuitTest, testDynamicParameter)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testVP");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto ctrl =
    std::make_unique<vector_pursuit_controller::VectorPursuitController>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  ctrl->configure(node, "test", tf, costmap);
  ctrl->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.k", 1.5),
      rclcpp::Parameter("test.desired_linear_vel", 1.0),
      rclcpp::Parameter("test.transform_tolerance", 0.5),
      rclcpp::Parameter("test.lookahead_dist", 1.0),
      rclcpp::Parameter("test.min_lookahead_dist", 6.0),
      rclcpp::Parameter("test.max_lookahead_dist", 7.0),
      rclcpp::Parameter("test.lookahead_time", 1.8),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 18.0),
      rclcpp::Parameter("test.rotate_to_heading_min_angle", 0.7),
      rclcpp::Parameter("test.min_linear_velocity", 1.0),
      rclcpp::Parameter("test.min_turning_radius", 0.0),
      rclcpp::Parameter("test.max_angular_accel", 3.0),
      rclcpp::Parameter("test.max_lateral_accel", 2.0),
      rclcpp::Parameter("test.max_linear_accel", 2.5),
      rclcpp::Parameter("test.max_allowed_time_to_collision_up_to_target", 5.0),
      rclcpp::Parameter("test.approach_velocity_scaling_dist", 10.0),
      rclcpp::Parameter("test.min_approach_linear_velocity", 0.6),
      rclcpp::Parameter("test.cost_scaling_dist", 2.0),
      rclcpp::Parameter("test.cost_scaling_gain", 4.0),
      rclcpp::Parameter("test.inflation_cost_scaling_factor", 1.0),
      rclcpp::Parameter("test.use_collision_detection", true),
      rclcpp::Parameter("test.use_velocity_scaled_lookahead_dist", false),
      rclcpp::Parameter("test.use_cost_regulated_linear_velocity_scaling", false),
      rclcpp::Parameter("test.use_rotate_to_heading", false),
      rclcpp::Parameter("test.use_interpolation", true),
      rclcpp::Parameter("test.use_heading_from_path", false),
      rclcpp::Parameter("test.allow_reversing", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.k").as_double(), 1.5);
  EXPECT_EQ(node->get_parameter("test.desired_linear_vel").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("test.lookahead_dist").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.min_lookahead_dist").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead_dist").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.lookahead_time").as_double(), 1.8);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_min_angle").as_double(), 0.7);
  EXPECT_EQ(node->get_parameter("test.min_linear_velocity").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.min_turning_radius").as_double(), 0.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("test.max_lateral_accel").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.max_linear_accel").as_double(), 2.5);
  EXPECT_EQ(
    node->get_parameter(
      "test.max_allowed_time_to_collision_up_to_target").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("test.approach_velocity_scaling_dist").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.min_approach_linear_velocity").as_double(), 0.6);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.cost_scaling_gain").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.inflation_cost_scaling_factor").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.use_collision_detection").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_velocity_scaled_lookahead_dist").as_bool(), false);
  EXPECT_EQ(
    node->get_parameter(
      "test.use_cost_regulated_linear_velocity_scaling").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_rotate_to_heading").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_interpolation").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_heading_from_path").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.allow_reversing").as_bool(), true);

  // An invalid inflation_cost_scaling_factor is rejected outright (not
  // silently ignored), and the previous value is retained.
  auto rejected = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.inflation_cost_scaling_factor", -1.0)});
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    rejected);
  EXPECT_FALSE(rejected.get().successful);
  EXPECT_EQ(node->get_parameter("test.inflation_cost_scaling_factor").as_double(), 1.0);
}

class ComputeVelocityCommandsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("testVP");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    ctrl_ = std::make_shared<Controller>();
  }

  void configure_costmap(uint16_t width, double resolution)
  {
    // Inject the costmap configuration via NodeOptions parameter_overrides.
    // Current nav2 declares these parameters inside Costmap2DROS::on_configure
    // (declare_or_get_parameter), so they must be supplied at construction
    // rather than set afterwards (which throws "parameter ... was not
    // declared"). width/height are in metres and declared as int.
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      {"width", static_cast<int>(width)},
      {"height", static_cast<int>(width)},
      {"resolution", resolution}
    });
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "fake_costmap", "/", false, options);

    rclcpp_lifecycle::State state;
    costmap_->on_configure(state);
    checker_.initialize(node_, "fake_checker", costmap_);
  }

  void configure_controller(bool allow_reversing)
  {
    std::string plugin_name = "test_vp";
    nav2::declare_parameter_if_not_declared(
      node_, plugin_name + ".allow_reversing",
      rclcpp::ParameterValue(allow_reversing));
    // These are kinematic tests on a clear/unknown test costmap; disable
    // collision detection so an empty costmap (cells = NO_INFORMATION) does
    // not abort the command. This mirrors nav2's own controller
    // compute-velocity unit tests.
    nav2::declare_parameter_if_not_declared(
      node_, plugin_name + ".use_collision_detection",
      rclcpp::ParameterValue(false));
    ctrl_->configure(node_, plugin_name, tf_buffer_, costmap_);
  }

  // Robot pose at the centre of the costmap, in the costmap global frame and
  // heading +x. Centring keeps the costmap cost/collision lookups in bounds.
  geometry_msgs::msg::PoseStamped robot_at_costmap_centre()
  {
    auto * cm = costmap_->getCostmap();
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = costmap_->getGlobalFrameID();
    p.header.stamp = node_->get_clock()->now();
    p.pose.position.x = cm->getOriginX() + cm->getSizeInMetersX() / 2.0;
    p.pose.position.y = cm->getOriginY() + cm->getSizeInMetersY() / 2.0;
    p.pose.orientation.w = 1.0;
    return p;
  }

  // Inject a plan directly in the robot base frame, already pruned to start at
  // the robot — exactly what the controller server's path handler would hand
  // over. computeVelocityCommands transforms it base->base (a tf2 identity),
  // so no transforms or path-handler plumbing are needed: the data is the
  // input. Points are (x, y) in metres relative to the robot.
  nav_msgs::msg::Path base_frame_plan(
    const std::vector<std::pair<double, double>> & points)
  {
    nav_msgs::msg::Path plan;
    // Stamp with the costmap's actual base frame so the controller's
    // transformPathInTargetFrame short-circuits (input frame == target frame)
    // and treats the points as already robot-relative.
    plan.header.frame_id = costmap_->getBaseFrameID();
    plan.header.stamp = node_->get_clock()->now();
    for (const auto & [x, y] : points) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = plan.header;
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation.w = 1.0;
      plan.poses.push_back(ps);
    }
    return plan;
  }

  // Straight plan of `n` poses spaced `step` m along the robot's heading
  // (sign +1 ahead, -1 behind).
  nav_msgs::msg::Path straight_plan(int n, double step, double sign)
  {
    std::vector<std::pair<double, double>> pts;
    for (int i = 0; i < n; i++) {pts.emplace_back(sign * step * i, 0.0);}
    return base_frame_plan(pts);
  }

  std::shared_ptr<Controller> ctrl_;
  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav2_controller::SimpleGoalChecker checker_;
};

TEST_F(ComputeVelocityCommandsTest, straightLineForward)
{
  configure_costmap(50u, 0.1);
  configure_controller(false);
  ctrl_->activate();

  auto robot_pose = robot_at_costmap_centre();
  // Straight plan ahead of the robot, in the base frame.
  auto plan = straight_plan(10, 1.0, +1.0);
  auto goal = plan.poses.back();  // unused by the controller

  geometry_msgs::msg::Twist robot_velocity;  // at rest

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(
    robot_pose, robot_velocity, &checker_, plan, goal);
  // From rest, linear velocity is acceleration-limited to
  // max_linear_accel * control_duration = 2.0 * (1/20) = 0.1, going straight.
  EXPECT_NEAR(cmd_vel.twist.linear.x, 0.1, 1e-6);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.01);
}

TEST_F(ComputeVelocityCommandsTest, straightLineBackward)
{
  configure_costmap(50u, 0.1);
  configure_controller(true);  // allow_reversing
  ctrl_->activate();

  auto robot_pose = robot_at_costmap_centre();
  // Straight plan directly behind the robot, in the base frame.
  auto plan = straight_plan(10, 1.0, -1.0);
  auto goal = plan.poses.back();

  geometry_msgs::msg::Twist robot_velocity;  // at rest

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(
    robot_pose, robot_velocity, &checker_, plan, goal);
  // Lookahead is behind, so the controller reverses: same magnitude, negative.
  EXPECT_NEAR(cmd_vel.twist.linear.x, -0.1, 1e-6);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 0.01);
}

TEST_F(ComputeVelocityCommandsTest, rotateToHeading)
{
  configure_costmap(50u, 0.1);
  configure_controller(false);
  ctrl_->activate();

  auto robot_pose = robot_at_costmap_centre();
  // Plan rising steeply to the side: the lookahead is well beyond the
  // rotate-to-heading angle, so the robot should rotate in place.
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i < 10; i++) {
    pts.emplace_back(i, 5.0 + i);
                                                             }
  auto plan = base_frame_plan(pts);
  auto goal = plan.poses.back();

  geometry_msgs::msg::Twist robot_velocity;  // at rest

  auto cmd_vel = ctrl_->computeVelocityCommandsWrapper(
    robot_pose, robot_velocity, &checker_, plan, goal);
  // Rotating in place: no translation, angular velocity acceleration-limited
  // from rest to max_angular_accel * control_duration = 3.2 * (1/20) = 0.16.
  EXPECT_NEAR(cmd_vel.twist.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.16, 0.01);
}
