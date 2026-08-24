/*****************************************************************************
 * ylr1d_teleop_keyboard.cpp
 *
 * Keyboard teleoperation for ylr1d robot.
 *
 * Mode-based control:
 *   1 – Base      (steering + drive)
 *   2 – Body      (lift + waist)
 *   3 – Left Arm  (7-DOF joint-by-joint)
 *   4 – Right Arm (7-DOF joint-by-joint)
 *   5 – Grippers  (left / right)
 *
 * Publishes Float64MultiArray messages to forward_command_controller topics.
 *****************************************************************************/

#include <cstdio>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// ── Keyboard helpers ──────────────────────────────────────────────────────

static struct termios g_old_tio;

void reset_terminal()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
}

void set_raw_terminal()
{
  struct termios tio;
  tcgetattr(STDIN_FILENO, &g_old_tio);
  tio = g_old_tio;
  tio.c_lflag &= ~(ICANON | ECHO);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &tio);
  atexit(reset_terminal);
}

char read_key()
{
  char c = 0;
  if (read(STDIN_FILENO, &c, 1) > 0)
    return c;
  return 0;
}

// ── Teleop node ───────────────────────────────────────────────────────────

class Ylr1dTeleopKeyboard : public rclcpp::Node
{
public:
  Ylr1dTeleopKeyboard()
    : Node("ylr1d_teleop_keyboard"),
      mode_(1),
      speed_scale_(1.0),
      joint_idx_(0)
  {
    // Publishers for each forward_command_controller
    pub_steer_  = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/steering_controller/commands", rclcpp::QoS(1));
    pub_drive_  = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/drive_controller/commands", rclcpp::QoS(1));
    pub_body_   = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/body_controller/commands", rclcpp::QoS(1));
    pub_lgrip_  = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_gripper_controller/commands", rclcpp::QoS(1));
    pub_rgrip_  = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_gripper_controller/commands", rclcpp::QoS(1));

    // Initialise all command arrays to zero
    steer_cmd_.data.assign(4, 0.0);
    drive_cmd_.data.assign(4, 0.0);
    body_cmd_.data.assign(4, 0.0);
    lgrip_cmd_.data.assign(2, 0.0);
    rgrip_cmd_.data.assign(2, 0.0);

    // Left-arm joint names (for display)
    left_arm_joints_ = {
      "Joint_Body2_to_LeftArm1",
      "Joint_LeftArm1_to_LeftArm2",
      "Joint_LeftArm2_to_LeftArm3",
      "Joint_LeftArm3_to_LeftArm4",
      "Joint_LeftArm4_to_LeftArm5",
      "Joint_LeftArm5_to_LeftArm6",
      "Joint_LeftArm6_to_LeftArm7"
    };
    // Right-arm joint names
    right_arm_joints_ = {
      "Joint_Body2_RightArm1",
      "Joint_RightArm1_to_RightArm2",
      "Joint_RightArm2_to_RightArm3",
      "Joint_RightArm3_to_RightArm4",
      "Joint_RightArm4_to_RightArm5",
      "Joint_RightArm5_to_RightArm6",
      "Joint_RightArm6_to_RightArm7"
    };
    // Arm joint positions (tracked for incremental control)
    left_arm_pos_.assign(7, 0.0);
    right_arm_pos_.assign(7, 0.0);

    print_help();
  }

  void run()
  {
    char c;
    while (rclcpp::ok()) {
      c = read_key();
      if (c != 0)
        handle_key(c);
      rclcpp::spin_some(shared_from_this());
      usleep(20000);  // 50 Hz
    }
  }

private:
  // ── Modes ─────────────────────────────────────────────────
  enum Mode { MODE_BASE = 1, MODE_BODY, MODE_LEFT_ARM, MODE_RIGHT_ARM, MODE_GRIPPER };

  int mode_;
  double speed_scale_;
  int joint_idx_;           // selected joint in arm modes
  std::vector<std::string> left_arm_joints_;
  std::vector<std::string> right_arm_joints_;
  std::vector<double> left_arm_pos_;
  std::vector<double> right_arm_pos_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    pub_steer_, pub_drive_, pub_body_, pub_lgrip_, pub_rgrip_;

  std_msgs::msg::Float64MultiArray steer_cmd_, drive_cmd_, body_cmd_,
    lgrip_cmd_, rgrip_cmd_;

  // ── Publish helper ───────────────────────────────────────
  void publish_all()
  {
    pub_steer_->publish(steer_cmd_);
    pub_drive_->publish(drive_cmd_);
    pub_body_->publish(body_cmd_);
    pub_lgrip_->publish(lgrip_cmd_);
    pub_rgrip_->publish(rgrip_cmd_);
  }

  void stop_all()
  {
    std::fill(steer_cmd_.data.begin(), steer_cmd_.data.end(), 0.0);
    std::fill(drive_cmd_.data.begin(), drive_cmd_.data.end(), 0.0);
    std::fill(body_cmd_.data.begin(), body_cmd_.data.end(), 0.0);
    std::fill(lgrip_cmd_.data.begin(), lgrip_cmd_.data.end(), 0.0);
    std::fill(rgrip_cmd_.data.begin(), rgrip_cmd_.data.end(), 0.0);
    publish_all();
    RCLCPP_INFO(get_logger(), "STOP  all motors");
  }

  // ── Key handler ──────────────────────────────────────────
  void handle_key(char c)
  {
    switch (c) {
      // ── Mode switching ──
      case '1': mode_ = MODE_BASE;    RCLCPP_INFO(get_logger(), "▶ Mode: BASE");    break;
      case '2': mode_ = MODE_BODY;    RCLCPP_INFO(get_logger(), "▶ Mode: BODY");    break;
      case '3': mode_ = MODE_LEFT_ARM;  joint_idx_ = 0;
        RCLCPP_INFO(get_logger(), "▶ Mode: LEFT ARM");  break;
      case '4': mode_ = MODE_RIGHT_ARM; joint_idx_ = 0;
        RCLCPP_INFO(get_logger(), "▶ Mode: RIGHT ARM"); break;
      case '5': mode_ = MODE_GRIPPER;
        RCLCPP_INFO(get_logger(), "▶ Mode: GRIPPERS");  break;

      // ── Global commands ──
      case ' ':
      case 'x':
      case 'X': stop_all(); break;

      case '+':
      case '=': speed_scale_ = std::min(speed_scale_ + 0.25, 3.0);
        RCLCPP_INFO(get_logger(), "Speed scale: %.2f", speed_scale_); break;
      case '-':
      case '_': speed_scale_ = std::max(speed_scale_ - 0.25, 0.25);
        RCLCPP_INFO(get_logger(), "Speed scale: %.2f", speed_scale_); break;

      case 'h':
      case 'H': print_help(); break;

      default:
        if (mode_ == MODE_BASE)      handle_base(c);
        else if (mode_ == MODE_BODY) handle_body(c);
        else if (mode_ == MODE_LEFT_ARM)  handle_arm(c, left_arm_pos_, left_arm_joints_, true);
        else if (mode_ == MODE_RIGHT_ARM) handle_arm(c, right_arm_pos_, right_arm_joints_, false);
        else if (mode_ == MODE_GRIPPER)   handle_gripper(c);
        break;
    }
  }

  // ── BASE mode ────────────────────────────────────────────
  //   steering:  0=forward,  +∠=left,  -∠=right
  //   drive:     + = forward, - = backward
  //   WASD=translate,  Q/E=rotate
  void handle_base(char c)
  {
    const double v = 2.0 * speed_scale_;     // wheel velocity
    const double a = 0.6 * speed_scale_;     // steering angle (rad)

    // Reset
    std::fill(steer_cmd_.data.begin(), steer_cmd_.data.end(), 0.0);
    std::fill(drive_cmd_.data.begin(), drive_cmd_.data.end(), 0.0);

    switch (c) {
      case 'w':
      case 'W':
        // Forward: all steer 0°, all drive +
        steer_cmd_.data = {0, 0, 0, 0};
        drive_cmd_.data = {v, v, v, v};
        RCLCPP_INFO(get_logger(), "BASE: forward  v=%.1f", v);
        break;
      case 's':
      case 'S':
        steer_cmd_.data = {0, 0, 0, 0};
        drive_cmd_.data = {-v, -v, -v, -v};
        RCLCPP_INFO(get_logger(), "BASE: backward v=%.1f", v);
        break;
      case 'a':
      case 'A':
        // Strafe left: steer 90° left, all drive +
        steer_cmd_.data = {a, -a, a, -a};
        drive_cmd_.data = {v, v, v, v};
        RCLCPP_INFO(get_logger(), "BASE: strafe left");
        break;
      case 'd':
      case 'D':
        // Strafe right: steer 90° right, all drive +
        steer_cmd_.data = {-a, a, -a, a};
        drive_cmd_.data = {v, v, v, v};
        RCLCPP_INFO(get_logger(), "BASE: strafe right");
        break;
      case 'q':
      case 'Q':
        // Rotate CCW: steer diagonal, drive opposing
        steer_cmd_.data = {a, -a, -a, a};
        drive_cmd_.data = {v, v, v, v};
        RCLCPP_INFO(get_logger(), "BASE: rotate CCW");
        break;
      case 'e':
      case 'E':
        steer_cmd_.data = {-a, a, a, -a};
        drive_cmd_.data = {v, v, v, v};
        RCLCPP_INFO(get_logger(), "BASE: rotate CW");
        break;
      default:
        return;  // no publish
    }
    publish_all();
  }

  // ── BODY mode ────────────────────────────────────────────
  //   W/S  – lift up/down            (Joint_Base_to_Body1)
  //   A/D  – waist rotate            (Joint_Body1_to_Body2)
  //   Q/E  – Body3 tilt              (Joint_Body2_to_Body3)
  //   Z/C  – Body4 tilt              (Joint_Body3_to_Body4)
  void handle_body(char c)
  {
    const double step = 0.05 * speed_scale_;
    switch (c) {
      case 'w': case 'W': body_cmd_.data[0] += step; break;
      case 's': case 'S': body_cmd_.data[0] -= step; break;
      case 'a': case 'A': body_cmd_.data[1] += step; break;
      case 'd': case 'D': body_cmd_.data[1] -= step; break;
      case 'q': case 'Q': body_cmd_.data[2] += step; break;
      case 'e': case 'E': body_cmd_.data[2] -= step; break;
      case 'z': case 'Z': body_cmd_.data[3] += step; break;
      case 'c': case 'C': body_cmd_.data[3] -= step; break;
      default: return;
    }
    // Clamp lift to limit range (prismatic: ±0.3)
    body_cmd_.data[0] = std::clamp(body_cmd_.data[0], -0.3, 0.3);
    RCLCPP_INFO(get_logger(), "BODY: lift=%.2f  waist=%.2f  tilt3=%.2f  tilt4=%.2f",
                body_cmd_.data[0], body_cmd_.data[1], body_cmd_.data[2], body_cmd_.data[3]);
    pub_body_->publish(body_cmd_);
  }

  // ── ARM mode ─────────────────────────────────────────────
  //   Tab          – cycle through joints
  //   W/S          – increment/decrement selected joint
  //   R            – reset all to 0 (home)
  void handle_arm(char c, std::vector<double> &pos,
                  const std::vector<std::string> &joints, bool is_left)
  {
    const double step = 0.05 * speed_scale_;
    const std::string prefix = is_left ? "L-ARM" : "R-ARM";

    switch (c) {
      case '\t':  // Tab
        joint_idx_ = (joint_idx_ + 1) % 7;
        RCLCPP_INFO(get_logger(), "%s: joint %d/%d [%s] = %.2f",
                    prefix.c_str(), joint_idx_, 7,
                    joints[joint_idx_].c_str(), pos[joint_idx_]);
        return;
      case 'w': case 'W': pos[joint_idx_] += step; break;
      case 's': case 'S': pos[joint_idx_] -= step; break;
      case 'r': case 'R':
        std::fill(pos.begin(), pos.end(), 0.0);
        RCLCPP_INFO(get_logger(), "%s: RESET to home", prefix.c_str());
        break;
      default: return;
    }

    // Clamp each joint within reasonable limits
    // Uses approximate limits from the URDF
    const std::vector<std::pair<double,double>> limits = {
      {-2.62, 2.62},   // shoulder
      {-1.57, 1.83},   // arm2
      {-2.62, 2.62},   // arm3
      {-1.57, 1.57},   // arm4
      {-2.62, 2.62},   // arm5
      {-2.09, 2.09},   // arm6
      {-6.28, 6.28},   // arm7 (continuous-ish)
    };
    for (size_t i = 0; i < pos.size(); ++i)
      pos[i] = std::clamp(pos[i], limits[i].first, limits[i].second);

    RCLCPP_INFO(get_logger(), "%s joint[%d] %s = %.2f",
                prefix.c_str(), joint_idx_, joints[joint_idx_].c_str(), pos[joint_idx_]);

    // Publish position array — the trajectory controller expects FollowJointTrajectory,
    // but we'll use a separate mechanism. For incremental mode, publish to body-style
    // topic or use the forward_command_controller approach.
    // NOTE: For arms we use joint_trajectory_controller; this keyboard node provides
    //       individual position control as a convenience. For trajectory execution use
    //       the ylr1d_arm_commander node instead.
    // For now we just print — the arm commander node handles trajectory goals.
    (void)is_left; // unused
  }

  // ── GRIPPER mode ─────────────────────────────────────────
  //   W/S  – left  gripper open/close
  //   A/D  – right gripper open/close
  void handle_gripper(char c)
  {
    const double step = 0.01 * speed_scale_;
    switch (c) {
      case 'w': case 'W': lgrip_cmd_.data[0] += step;
                          lgrip_cmd_.data[1] += step; break;
      case 's': case 'S': lgrip_cmd_.data[0] -= step;
                          lgrip_cmd_.data[1] -= step; break;
      case 'a': case 'A': rgrip_cmd_.data[0] += step;
                          rgrip_cmd_.data[1] += step; break;
      case 'd': case 'D': rgrip_cmd_.data[0] -= step;
                          rgrip_cmd_.data[1] -= step; break;
      case 'r': case 'R':
        std::fill(lgrip_cmd_.data.begin(), lgrip_cmd_.data.end(), 0.0);
        std::fill(rgrip_cmd_.data.begin(), rgrip_cmd_.data.end(), 0.0);
        RCLCPP_INFO(get_logger(), "GRIPPERS: RESET");
        break;
      default: return;
    }
    // Clamp finger positions (±0.05 from URDF)
    for (auto &v : lgrip_cmd_.data) v = std::clamp(v, -0.05, 0.05);
    for (auto &v : rgrip_cmd_.data) v = std::clamp(v, -0.05, 0.05);

    RCLCPP_INFO(get_logger(), "GRIP: L=[%.3f, %.3f]  R=[%.3f, %.3f]",
                lgrip_cmd_.data[0], lgrip_cmd_.data[1],
                rgrip_cmd_.data[0], rgrip_cmd_.data[1]);
    pub_lgrip_->publish(lgrip_cmd_);
    pub_rgrip_->publish(rgrip_cmd_);
  }

  void print_help()
  {
    RCLCPP_INFO(get_logger(), R"(
╔══════════════════════════════════════════════════════════╗
║              ylr1d  Keyboard Teleop                     ║
╠══════════════════════════════════════════════════════════╣
║  ┌─ Mode ────────────────────────────────────────────┐  ║
║  │  1  Base       2  Body       3  Left Arm          │  ║
║  │  4  Right Arm  5  Grippers                        │  ║
║  └───────────────────────────────────────────────────┘  ║
║  ┌─ Base ────────────────────────────────────────────┐  ║
║  │  W/S  forward/back    A/D  strafe                 │  ║
║  │  Q/E  rotate                                       │  ║
║  └───────────────────────────────────────────────────┘  ║
║  ┌─ Body ────────────────────────────────────────────┐  ║
║  │  W/S  lift up/down     A/D  waist rotate          │  ║
║  │  Q/E  Body3 tilt       Z/C  Body4 tilt            │  ║
║  └───────────────────────────────────────────────────┘  ║
║  ┌─ Arm (3/4) ───────────────────────────────────────┐  ║
║  │  Tab  cycle joint        W/S  inc/dec position    │  ║
║  │  R    reset to home                               │  ║
║  └───────────────────────────────────────────────────┘  ║
║  ┌─ Gripper ─────────────────────────────────────────┐  ║
║  │  W/S  left gripper  A/D  right gripper            │  ║
║  │  R    reset grippers                               │  ║
║  └───────────────────────────────────────────────────┘  ║
║  ┌─ Global ──────────────────────────────────────────┐  ║
║  │  SPACE / X  stop all      +/-  speed              │  ║
║  │  H          this help                              │  ║
║  └───────────────────────────────────────────────────┘  ║
╚══════════════════════════════════════════════════════════╝
)");
  }
};

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  set_raw_terminal();

  auto node = std::make_shared<Ylr1dTeleopKeyboard>();
  RCLCPP_INFO(node->get_logger(), "ylr1d teleop keyboard started. Press H for help.");

  node->run();

  reset_terminal();
  rclcpp::shutdown();
  return 0;
}
