/*****************************************************************************
 * ylr1d_arm_commander.cpp
 *
 * Sends FollowJointTrajectory action goals to the left/right arm trajectory
 * controllers.  Supports:
 *   – Predefined poses  (home, neutral, reach)
 *   – Custom joint-space goals from command-line or service
 *   – Synchronised dual-arm motion
 *
 * Topics:
 *   Action clients:
 *     /left_arm_controller/follow_joint_trajectory
 *     /right_arm_controller/follow_joint_trajectory
 *****************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ── Pose presets ──────────────────────────────────────────────────────────

struct Pose
{
  std::string name;
  std::vector<double> positions;  // 7 joints
};

// Left & right arm share the same joint numbering (shoulder → wrist roll)
const std::vector<Pose> LEFT_ARM_POSES = {
  { "home",    { 0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00}},
  { "neutral", { 0.00,  0.80, -1.20,  0.00,  0.00,  0.00,  0.00}},
  { "reach",   { 0.50,  0.50, -0.80, -0.50,  0.30,  0.00,  0.00}},
  { "fold",    { 1.50,  1.20, -1.50,  0.00,  0.00,  0.00,  0.00}},
};

const std::vector<Pose> RIGHT_ARM_POSES = {
  { "home",    { 0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00}},
  { "neutral", { 0.00,  0.80, -1.20,  0.00,  0.00,  0.00,  0.00}},
  { "reach",   {-0.50,  0.50, -0.80,  0.50, -0.30,  0.00,  0.00}},
  { "fold",    {-1.50,  1.20, -1.50,  0.00,  0.00,  0.00,  0.00}},
};

// ── Arm Commander Node ────────────────────────────────────────────────────

class Ylr1dArmCommander : public rclcpp::Node
{
public:
  Ylr1dArmCommander()
    : Node("ylr1d_arm_commander")
  {
    // Action clients
    left_ac_  = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/left_arm_controller/follow_joint_trajectory");
    right_ac_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/right_arm_controller/follow_joint_trajectory");

    // Service-like command topic (String) for simple pose commands
    sub_cmd_ = create_subscription<std_msgs::msg::String>(
      "/arm_commander/cmd", rclcpp::QoS(1),
      std::bind(&Ylr1dArmCommander::on_cmd, this, std::placeholders::_1));

    joint_names_ = {
      "Joint_Body2_to_LeftArm1",   // 0  shoulder
      "Joint_LeftArm1_to_LeftArm2",// 1  upper arm
      "Joint_LeftArm2_to_LeftArm3",// 2  elbow
      "Joint_LeftArm3_to_LeftArm4",// 3  forearm
      "Joint_LeftArm4_to_LeftArm5",// 4  wrist pitch
      "Joint_LeftArm5_to_LeftArm6",// 5  wrist yaw
      "Joint_LeftArm6_to_LeftArm7" // 6  wrist roll
    };
    right_joint_names_ = {
      "Joint_Body2_RightArm1",
      "Joint_RightArm1_to_RightArm2",
      "Joint_RightArm2_to_RightArm3",
      "Joint_RightArm3_to_RightArm4",
      "Joint_RightArm4_to_RightArm5",
      "Joint_RightArm5_to_RightArm6",
      "Joint_RightArm6_to_RightArm7"
    };

    RCLCPP_INFO(get_logger(), "Arm commander ready. "
                "Send to /arm_commander/cmd: 'L:home', 'R:reach', 'LR:home', etc.");
  }

private:
  using StringMsg = std_msgs::msg::String;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_ac_, right_ac_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_cmd_;
  std::vector<std::string> joint_names_, right_joint_names_;

  // ── Command callback ────────────────────────────────────
  void on_cmd(const StringMsg::SharedPtr msg)
  {
    const std::string &cmd = msg->data;
    RCLCPP_INFO(get_logger(), "Command: %s", cmd.c_str());

    // Parse: "L:pose_name", "R:pose_name", "LR:pose_name"
    if (cmd.rfind("L:", 0) == 0) {
      send_pose(true, cmd.substr(2));
    } else if (cmd.rfind("R:", 0) == 0) {
      send_pose(false, cmd.substr(2));
    } else if (cmd.rfind("LR:", 0) == 0) {
      send_pose(true,  cmd.substr(3));
      send_pose(false, cmd.substr(3));
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command. Use L:name, R:name, or LR:name");
    }
  }

  // ── Send pose ───────────────────────────────────────────
  void send_pose(bool is_left, const std::string &pose_name)
  {
    const auto &poses = is_left ? LEFT_ARM_POSES : RIGHT_ARM_POSES;
    const auto &names = is_left ? joint_names_ : right_joint_names_;
    auto &ac = is_left ? left_ac_ : right_ac_;

    // Find pose
    const Pose *pose = nullptr;
    for (const auto &p : poses) {
      if (p.name == pose_name) { pose = &p; break; }
    }
    if (!pose) {
      RCLCPP_WARN(get_logger(), "Unknown pose '%s' for %s arm",
                  pose_name.c_str(), is_left ? "left" : "right");
      return;
    }

    if (!ac->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Action server not available for %s arm",
                  is_left ? "left" : "right");
      return;
    }

    // Build goal
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = pose->positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);  // 2 s ramp
    goal.trajectory.points.push_back(point);

    // Send
    auto send_opt = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_opt.goal_response_callback =
      [this, is_left](const GoalHandle::SharedPtr &gh) {
        if (gh) RCLCPP_INFO(get_logger(), "%s arm goal accepted",
                            is_left ? "Left" : "Right");
        else    RCLCPP_WARN(get_logger(), "%s arm goal REJECTED",
                            is_left ? "Left" : "Right");
      };
    send_opt.result_callback =
      [this, is_left](const GoalHandle::WrappedResult &res) {
        RCLCPP_INFO(get_logger(), "%s arm goal %s",
                    is_left ? "Left" : "Right",
                    res.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded" : "failed");
      };

    ac->async_send_goal(goal, send_opt);
  }
};

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ylr1dArmCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
