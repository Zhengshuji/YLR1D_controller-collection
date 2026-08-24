/*****************************************************************************
 * ylr1d_joint_effort_plugin.cpp
 *
 * Minimal Gazebo plugin that subscribes to /pid_controller/commands
 * (Float64MultiArray) and applies the received efforts to the robot's joints.
 *
 * Joint order must match the array layout in the topic:
 *   [steering×4, drive×4, body×4, Larm×7, Rarm×7, Lgrip×2, Rgrip×2]
 *****************************************************************************/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ylr1d_control
{

class Ylr1dJointEffortPlugin : public gazebo::ModelPlugin
{
public:
  Ylr1dJointEffortPlugin() = default;
  ~Ylr1dJointEffortPlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    // ── Collect joint names ────────────────────────────────
    // Order must match /pid_controller/commands array layout.
    std::vector<std::string> joint_names = {
      // Steering (4)
      "Joint_Base_to_RFWheelF",
      "Joint_Base_to_LFWheelF",
      "Joint_Base_to_RBWheelF",
      "Joint_Base_to_LBWheelF",
      // Drive (4)
      "Joint_RFWheelF_to_RFWheel",
      "Joint_LFWheelF_to_LFWheel",
      "Joint_RBWheelF_to_RBWheel",
      "Joint_LBWheelF_to_LBWheel",
      // Body (4)
      "Joint_Base_to_Body1",
      "Joint_Body1_to_Body2",
      "Joint_Body2_to_Body3",
      "Joint_Body3_to_Body4",
      // Left arm (7)
      "Joint_Body2_to_LeftArm1",
      "Joint_LeftArm1_to_LeftArm2",
      "Joint_LeftArm2_to_LeftArm3",
      "Joint_LeftArm3_to_LeftArm4",
      "Joint_LeftArm4_to_LeftArm5",
      "Joint_LeftArm5_to_LeftArm6",
      "Joint_LeftArm6_to_LeftArm7",
      // Right arm (7)
      "Joint_Body2_RightArm1",
      "Joint_RightArm1_to_RightArm2",
      "Joint_RightArm2_to_RightArm3",
      "Joint_RightArm3_to_RightArm4",
      "Joint_RightArm4_to_RightArm5",
      "Joint_RightArm5_to_RightArm6",
      "Joint_RightArm6_to_RightArm7",
      // Left gripper (2)
      "Joint_LeftArm7_to_LeftFinger1",
      "Joint_LeftArm7_to_LeftFinger2",
      // Right gripper (2)
      "Joint_RightArm7_to_RightFinger1",
      "Joint_RightArm7_to_RightFinger2",
    };

    // Resolve to Gazebo joint pointers
    for (const auto &name : joint_names) {
      auto joint = model_->GetJoint(name);
      if (!joint) {
        gzerr << "Joint [" << name << "] not found in model!\n";
        continue;
      }
      joints_.push_back(joint);
    }
    gzmsg << "Ylr1dJointEffortPlugin: configured " << joints_.size()
          << "/" << joint_names.size() << " joints\n";

    // ── ROS 2 node handle ──────────────────────────────────
    node_ = std::make_shared<rclcpp::Node>("ylr1d_joint_effort_plugin");

    // ── Subscribe to effort commands ───────────────────────
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pid_controller/commands", rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        const size_t N = std::min(msg->data.size(), joints_.size());
        efforts_.resize(N);
        for (size_t i = 0; i < N; ++i)
          efforts_[i] = msg->data[i];
        has_effort_ = true;
      });

    // ── Publish joint states (so PID node gets feedback) ───
    pub_joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(1));

    // ── Timer for publishing joint states ──────────────────
    js_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(5),  // 200 Hz, matching PID loop
      std::bind(&Ylr1dJointEffortPlugin::PublishJointStates, this));

    // ── Connect the world update event ─────────────────────
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&Ylr1dJointEffortPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!has_effort_ || efforts_.empty())
      return;

    // Apply cached efforts to joints
    for (size_t i = 0; i < efforts_.size() && i < joints_.size(); ++i) {
      joints_[i]->SetForce(0, efforts_[i]);
    }
  }

  void PublishJointStates()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "world";

    for (const auto &joint : joints_) {
      msg.name.push_back(joint->GetName());
      msg.position.push_back(joint->Position(0));
      msg.velocity.push_back(joint->GetVelocity(0));
      // effort is what we just applied, but we can include it too
      msg.effort.push_back(0.0);
    }
    pub_joint_state_->publish(msg);
  }

  // ── Gazebo handles ───────────────────────────────────────
  gazebo::physics::ModelPtr model_;
  std::vector<gazebo::physics::JointPtr> joints_;
  gazebo::event::ConnectionPtr update_conn_;

  // ── ROS 2 ────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::TimerBase::SharedPtr js_timer_;
  std::vector<double> efforts_;
  bool has_effort_ = false;
};

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(Ylr1dJointEffortPlugin)

}  // namespace ylr1d_control
