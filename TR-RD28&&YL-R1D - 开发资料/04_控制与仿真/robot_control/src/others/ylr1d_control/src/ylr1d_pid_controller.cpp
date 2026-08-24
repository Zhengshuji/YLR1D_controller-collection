/*****************************************************************************
 * ylr1d_pid_controller.cpp
 *
 * Unified PID position controller for all 30 joints.
 *
 * Control law (per joint):
 *   τ = kp · (q_des − q_act) + ki · ∫(q_des − q_act)dt − kd · q̇_act
 *
 * Subscribes to:
 *   /desired_joint_positions  (Float64MultiArray)  — desired positions
 *   /joint_states             (JointState)          — feedback
 *
 * Publishes to:
 *   /pid_controller/commands  (Float64MultiArray)  — effort commands
 *
 * Default desired positions = 0 (holds robot at zero config).
 *****************************************************************************/

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ylr1d_control/pid_controller.hpp"

// ── Node ────────────────────────────────────────────────────────────────────

class Ylr1dPidControllerNode : public rclcpp::Node
{
public:
  Ylr1dPidControllerNode()
    : Node("ylr1d_pid_controller",
           rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    // ── Parameters (YAML pre-loaded, fallback to hardcoded defaults) ──
    if (has_parameter("joints") &&
        get_parameter("joints").get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      joint_names_ = get_parameter("joints").as_string_array();
      RCLCPP_INFO(get_logger(), "Loaded %zu joints from parameters.", joint_names_.size());
    } else {
      joint_names_ = kDefaultJoints_;
      RCLCPP_WARN(get_logger(),
        "No 'joints' parameter — using built-in default list (%zu joints).", joint_names_.size());
    }

    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_logger(), "No joints specified — nothing to control.");
      return;
    }

    const size_t N = joint_names_.size();

    // Gains — accept array of N values or single scalar (broadcast)
    kp_         = load_gain_param("kp", N, 500.0);
    ki_         = load_gain_param("ki", N, 0.0);
    kd_         = load_gain_param("kd", N, 20.0);
    max_effort_ = load_gain_param("max_effort", N, 100.0);
    i_max_      = load_gain_param("i_max", N, 0.0);

    declare_parameter("rate", 200);
    rate_ = get_parameter("rate").as_int();

    // ── Per-joint state ─────────────────────────────────────
    pids_.resize(N);
    desired_pos_.resize(N, 0.0);   // default: all joints at zero
    current_pos_.resize(N, 0.0);
    current_vel_.resize(N, 0.0);
    pos_received_.resize(N, false);

    for (size_t i = 0; i < N; ++i) {
      ylr1d_control::PidGains g;
      g.kp         = kp_[i];
      g.ki         = ki_[i];
      g.kd         = kd_[i];
      g.max_effort = max_effort_[i];
      g.i_max      = i_max_[i];
      pids_[i].set_gains(g);

      RCLCPP_INFO(get_logger(), "  [%2zu] %-40s  kp=%7.1f  ki=%6.2f  kd=%7.1f",
                  i, joint_names_[i].c_str(), kp_[i], ki_[i], kd_[i]);
    }

    // Name → index lookup
    for (size_t i = 0; i < N; ++i)
      name_to_idx_[joint_names_[i]] = i;

    // ── Subscribers ─────────────────────────────────────────
    sub_desired_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/desired_joint_positions", rclcpp::QoS(1),
      std::bind(&Ylr1dPidControllerNode::on_desired, this, std::placeholders::_1));

    sub_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1),
      std::bind(&Ylr1dPidControllerNode::on_joint_states, this, std::placeholders::_1));

    // ── Publisher ───────────────────────────────────────────
    pub_effort_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pid_controller/commands", rclcpp::QoS(1));

    // ── Control loop timer ──────────────────────────────────
    auto period_ns = std::chrono::nanoseconds(1000000000 / rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_)),
      std::bind(&Ylr1dPidControllerNode::update, this));

    RCLCPP_INFO(get_logger(),
      "\nylr1d PID controller ready — %zu joints @ %d Hz\n"
      "  Desired positions: /desired_joint_positions\n"
      "  Effort output:     /pid_controller/commands",
      N, rate_);
  }

private:
  // ── Hardcoded fallback joint list (used when --params-file is not available) ──
  static const std::vector<std::string> kDefaultJoints_;

  // ── Data ──────────────────────────────────────────────────
  std::vector<std::string> joint_names_;
  std::vector<double> kp_, ki_, kd_, max_effort_, i_max_;
  std::vector<double> desired_pos_, current_pos_, current_vel_;
  std::vector<bool> pos_received_;
  std::unordered_map<std::string, size_t> name_to_idx_;
  std::vector<ylr1d_control::PidController> pids_;
  int rate_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_effort_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_desired_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Helpers ───────────────────────────────────────────────

  std::vector<double> load_gain_param(
    const std::string &name, size_t N, double default_val)
  {
    if (!has_parameter(name)) {
      RCLCPP_WARN(get_logger(), "%s: not set, using default %.1f",
                  name.c_str(), default_val);
      return std::vector<double>(N, default_val);
    }

    auto param = get_parameter(name);

    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
        auto arr = param.as_double_array();
        if (arr.size() == N) return arr;
        if (arr.size() == 1) return std::vector<double>(N, arr[0]);
        break;
      }
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
        auto arr = param.as_integer_array();
        if (arr.size() == N)
          return std::vector<double>(arr.begin(), arr.end());
        if (arr.size() == 1)
          return std::vector<double>(N, static_cast<double>(arr[0]));
        break;
      }
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return std::vector<double>(N, param.as_double());
      default:
        break;
    }

    RCLCPP_WARN(get_logger(), "%s: unexpected type/size, using default %.1f",
                name.c_str(), default_val);
    return std::vector<double>(N, default_val);
  }

  // ── Callbacks ─────────────────────────────────────────────

  void on_desired(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const size_t N = std::min(msg->data.size(), desired_pos_.size());
    for (size_t i = 0; i < N; ++i)
      desired_pos_[i] = msg->data[i];
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = name_to_idx_.find(msg->name[i]);
      if (it == name_to_idx_.end())
        continue;
      size_t idx = it->second;
      if (i < msg->position.size()) {
        current_pos_[idx] = msg->position[i];
        pos_received_[idx] = true;
      }
      if (i < msg->velocity.size())
        current_vel_[idx] = msg->velocity[i];
    }
  }

  // ── Control loop ──────────────────────────────────────────

  void update()
  {
    // Wait until we have feedback for all joints
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!pos_received_[i])
        return;
    }

    // Time step
    rclcpp::Time now = this->now();
    static rclcpp::Time last_time = now;
    double dt = (now - last_time).seconds();
    if (dt <= 0.0 || dt > 1.0)
      dt = 1.0 / rate_;
    last_time = now;

    // Compute PID for each joint
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      msg.data[i] = pids_[i].compute(
        desired_pos_[i], current_pos_[i], current_vel_[i], dt);
    }

    pub_effort_->publish(msg);
  }
};

// ── Static default joint list ───────────────────────────────────────────────

const std::vector<std::string> Ylr1dPidControllerNode::kDefaultJoints_ = {
  // Steering (4)
  "Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
  "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF",
  // Drive (4)
  "Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
  "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel",
  // Body (4)
  "Joint_Base_to_Body1", "Joint_Body1_to_Body2",
  "Joint_Body2_to_Body3", "Joint_Body3_to_Body4",
  // Left arm (7)
  "Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
  "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
  "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
  "Joint_LeftArm6_to_LeftArm7",
  // Right arm (7)
  "Joint_Body2_RightArm1", "Joint_RightArm1_to_RightArm2",
  "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
  "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
  "Joint_RightArm6_to_RightArm7",
  // Left gripper (2)
  "Joint_LeftArm7_to_LeftFinger1", "Joint_LeftArm7_to_LeftFinger2",
  // Right gripper (2)
  "Joint_RightArm7_to_RightFinger1", "Joint_RightArm7_to_RightFinger2",
};

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ylr1dPidControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
