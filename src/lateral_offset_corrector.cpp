#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <algorithm>
#include <cmath>
#include <limits>

// Shifts every pose of the incoming path sideways by a small amount, so the
// robot can be nudged left/right to re-align with a physical crop row while
// FollowPath keeps running (this node is re-ticked every cycle from a
// ReactiveSequence, so the shift is re-applied continuously without ever
// stopping/canceling the FollowPath goal).
//
// Sign convention matches SwathFactory::generateOffsetSwaths in
// row_swath_generator.cpp: perpendicular = (-dy, dx)/len, so a positive
// offset shifts LEFT of the direction of travel, negative shifts RIGHT.
//
// Coverage paths here can be huge (100k+ poses over many km), and being
// inside a ReactiveSequence means this node is re-ticked at the full BT
// rate (10Hz) for as long as FollowPath runs. Deep-copying such a path on
// every tick (getInput + setOutput, each poses.size() PoseStamped incl. a
// per-pose Header string) is expensive enough on embedded hardware to blow
// the tick budget on its own. So: only touch the ports (read nav_path,
// recompute, write offset_path) when the offset value actually changed
// since the last tick; otherwise skip all of it and return immediately.
//
// IMPORTANT: nav_path is the *static* path ResumePathTrimmer computed once
// at the start of this resume episode -- it always starts from wherever the
// robot was back then, not from where it is now. Every time we recompute we
// republish that whole array to FollowPath, and FollowPath's controller
// calls setPlan() with it, which resets the controller's own path-pruning
// progress back to that stale start. If the offset changes long after the
// robot has driven well past that point, the controller's own search for
// "closest point to the robot" (bounded by max_robot_pose_search_dist,
// measured as arc-length from the array's start) can't reach the robot's
// actual position any more, and every control cycle fails with "Resulting
// plan has 0 poses in it" until controller_patience gives up on the goal.
// So on every recompute we first re-anchor: find the pose in nav_path
// closest to the robot's current TF position and drop everything before
// it, then apply the offset only to what's left ahead of the robot.
class LateralOffsetCorrector : public BT::ActionNodeBase
{
public:
  // Matches max_lateral_offset default in cmd_vel_override/lateral_correction_node.py
  static constexpr double kMaxLateralOffset = 0.5;

  LateralOffsetCorrector(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config), offset_(0.0),
    last_applied_offset_(std::numeric_limits<double>::quiet_NaN())
  {
    node_ = rclcpp::Node::make_shared("lateral_offset_corrector_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(10);
    qos.transient_local();

    offset_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/row_alignment/lateral_offset", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        double requested = msg->data;
        double clamped = std::clamp(requested, -kMaxLateralOffset, kMaxLateralOffset);
        if (clamped != requested) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Requested lateral_offset %.3f exceeds +/-%.2f m limit, clamping to %.3f",
            requested, kMaxLateralOffset, clamped);
        }
        offset_ = clamped;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path"),
      BT::OutputPort<nav_msgs::msg::Path>("offset_path")
    };
  }

  BT::NodeStatus tick() override
  {
    // Process any pending offset updates published since the last tick
    rclcpp::spin_some(node_);

    // Steady state (by far the common case): offset hasn't changed since we
    // last wrote offset_path, so the blackboard entry is already correct.
    // Skip touching the ports entirely -- no path copy, no re-publish.
    if (has_output_ && offset_ == last_applied_offset_) {
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path input_path;
    if (!getInput("nav_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_path]");
      return BT::NodeStatus::FAILURE;
    }

    if (input_path.poses.size() < 2) {
      setOutput("offset_path", input_path);
      last_applied_offset_ = offset_;
      has_output_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    // Re-anchor to the robot's current position so we hand FollowPath a
    // plan that actually starts near the robot, instead of the stale
    // resume-time start of nav_path (see class comment) -- but only from
    // the *second* recompute onward. On the very first tick of a resume
    // episode, nav_path already starts exactly at the robot's pose plus a
    // smooth interpolated lead-in built by ResumePathTrimmer
    // (generateSafePathToRow); re-anchoring here would race that: a fresh
    // TF lookup a few tens of ms later can find a point further along as
    // "nearest" (robot has moved a hair since ResumePathTrimmer's own TF
    // read), trimming away the lead-in and making FollowPath aim straight
    // at a row point that's off to the side -- i.e. the robot turns sharply
    // into the row instead of approaching it straight, risking crop damage.
    size_t start_idx = 0;
    if (has_output_) {
      geometry_msgs::msg::PoseStamped robot_pose;
      if (getCurrentRobotPose(robot_pose)) {
        start_idx = findNearestIndex(robot_pose, input_path);
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "Could not get current robot pose from TF, applying offset to the full "
          "(possibly stale) nav_path without re-anchoring");
      }
    }

    if (std::fabs(offset_) < 1e-4) {
      nav_msgs::msg::Path pass_through = input_path;
      if (start_idx > 0) {
        pass_through.poses.erase(
          pass_through.poses.begin(), pass_through.poses.begin() + start_idx);
      }
      setOutput("offset_path", pass_through);
      last_applied_offset_ = offset_;
      has_output_ = true;
      return BT::NodeStatus::SUCCESS;
    }

    nav_msgs::msg::Path offset_path;
    offset_path.header = input_path.header;
    offset_path.poses.assign(
      input_path.poses.begin() + start_idx, input_path.poses.end());
    const size_t n = offset_path.poses.size();

    for (size_t i = 0; i < n; ++i) {
      // Central difference for the local tangent, forward/backward at the ends
      size_t prev = (i == 0) ? 0 : i - 1;
      size_t next = (i == n - 1) ? n - 1 : i + 1;

      double dx = offset_path.poses[next].pose.position.x - offset_path.poses[prev].pose.position.x;
      double dy = offset_path.poses[next].pose.position.y - offset_path.poses[prev].pose.position.y;
      double len = std::sqrt(dx * dx + dy * dy);

      if (len < 1e-6) {
        continue;
      }

      double perp_x = -dy / len;
      double perp_y = dx / len;

      offset_path.poses[i].pose.position.x += perp_x * offset_;
      offset_path.poses[i].pose.position.y += perp_y * offset_;
    }

    setOutput("offset_path", offset_path);
    last_applied_offset_ = offset_;
    has_output_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // Called by bt_navigator's haltAllActions() after every goal completes
  // (success or failure) -- this tree/node instance is NOT recreated per
  // resume episode (loadBehaviorTree() only runs once, at on_activate(),
  // and is skipped on later goals since the XML filename is unchanged), so
  // has_output_/last_applied_offset_ would otherwise stay stuck from one
  // episode to the next. Reset them here so the *next* episode's first
  // tick is correctly treated as "first tick" again (see tick() comment on
  // why that matters -- skipping the risky re-anchor on it).
  void halt() override
  {
    RCLCPP_INFO(node_->get_logger(), "LateralOffsetCorrector::halt() -- resetting episode state");
    has_output_ = false;
    last_applied_offset_ = std::numeric_limits<double>::quiet_NaN();
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr offset_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double offset_;
  double last_applied_offset_;
  bool has_output_ = false;

  bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped & robot_pose) const
  {
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, std::chrono::milliseconds(200));

      robot_pose.header.stamp = node_->now();
      robot_pose.header.frame_id = "map";
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "TF exception: %s", ex.what());
      return false;
    }
  }

  static size_t findNearestIndex(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & path)
  {
    size_t nearest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - robot_pose.pose.position.x;
      double dy = path.poses[i].pose.position.y - robot_pose.pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        nearest_idx = i;
      }
    }
    return nearest_idx;
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<LateralOffsetCorrector>("LateralOffsetCorrector");
}
