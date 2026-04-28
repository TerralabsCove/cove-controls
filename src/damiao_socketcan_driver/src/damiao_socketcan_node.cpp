#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "damiao_socketcan_driver/damiao_socketcan.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

struct MotorDesc
{
  std::string name;
  damiao_socketcan::DM_Motor_Type type;
  uint32_t slave_id;
  uint32_t master_id;
};

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\n\r");
  return value.substr(first, last - first + 1);
}

}  // namespace

class DamiaoSocketCanNode : public rclcpp::Node
{
public:
  DamiaoSocketCanNode()
  : Node("damiao_socketcan_driver")
  {
    declare_parameter<std::string>("can_interface", "can1");
    declare_parameter<double>("loop_rate", 100.0);
    declare_parameter<std::string>("control_mode", "status");
    declare_parameter<std::string>("motors", "");
    declare_parameter<bool>("auto_enable", false);
    declare_parameter<bool>("switch_mode_on_start", false);
    declare_parameter<bool>("disable_on_shutdown", true);

    can_interface_ = get_parameter("can_interface").as_string();
    control_mode_ = get_parameter("control_mode").as_string();
    auto_enable_ = get_parameter("auto_enable").as_bool();
    switch_mode_on_start_ = get_parameter("switch_mode_on_start").as_bool();
    disable_on_shutdown_ = get_parameter("disable_on_shutdown").as_bool();
    const double loop_rate = get_parameter("loop_rate").as_double();
    const auto motors_spec = get_parameter("motors").as_string();

    if (motors_spec.empty()) {
      RCLCPP_FATAL(get_logger(), "No motors configured. Set the 'motors' parameter.");
      throw std::runtime_error("No motors configured");
    }

    can_ = std::make_shared<damiao_socketcan::SocketCan>(can_interface_);
    mc_ = std::make_unique<damiao_socketcan::MotorControl>(can_);
    parse_motors(motors_spec);

    if (switch_mode_on_start_ || auto_enable_) {
      configure_motors();
    }

    const size_t n = motors_.size();
    cmd_vel_.assign(n, 0.0f);
    cmd_pos_.assign(n, 0.0f);
    cmd_mit_kp_.assign(n, 0.0f);
    cmd_mit_kd_.assign(n, 0.0f);
    cmd_mit_q_.assign(n, 0.0f);
    cmd_mit_dq_.assign(n, 0.0f);
    cmd_mit_tau_.assign(n, 0.0f);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    if (control_mode_ == "velocity") {
      vel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_vel_motors", 10,
        std::bind(&DamiaoSocketCanNode::vel_callback, this, std::placeholders::_1));
    } else if (control_mode_ == "pos_vel" || control_mode_ == "position") {
      pos_vel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_pos_vel", 10,
        std::bind(&DamiaoSocketCanNode::pos_vel_callback, this, std::placeholders::_1));
    } else if (control_mode_ == "mit") {
      mit_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_mit", 10,
        std::bind(&DamiaoSocketCanNode::mit_callback, this, std::placeholders::_1));
    }

    enable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/enable_motors",
      std::bind(
        &DamiaoSocketCanNode::enable_service, this, std::placeholders::_1,
        std::placeholders::_2));
    disable_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/disable_motors",
      std::bind(
        &DamiaoSocketCanNode::disable_service, this, std::placeholders::_1,
        std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / loop_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DamiaoSocketCanNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Damiao SocketCAN driver ready on %s with %zu motor(s), control_mode=%s, auto_enable=%s",
      can_interface_.c_str(), motors_.size(), control_mode_.c_str(), auto_enable_ ? "true" : "false");
  }

  ~DamiaoSocketCanNode() override
  {
    if (!disable_on_shutdown_ || !mc_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Shutting down: disabling configured motors.");
    for (auto & motor : motors_) {
      try {
        mc_->disable(motor);
      } catch (const std::exception & exc) {
        RCLCPP_WARN(get_logger(), "Disable failed for 0x%02X: %s", motor.GetSlaveId(), exc.what());
      }
    }
  }

private:
  void parse_motors(const std::string & spec)
  {
    std::vector<MotorDesc> descs;
    std::istringstream stream(spec);
    std::string token;
    while (std::getline(stream, token, ',')) {
      token = trim(token);
      if (token.empty()) {
        continue;
      }

      std::vector<std::string> parts;
      std::istringstream token_stream(token);
      std::string part;
      while (std::getline(token_stream, part, ':')) {
        parts.push_back(trim(part));
      }

      if (parts.size() != 4) {
        throw std::runtime_error(
          "Invalid motor spec '" + token + "'. Expected name:type:slave_id:master_id");
      }

      descs.push_back({
        parts[0],
        damiao_socketcan::motor_type_from_string(parts[1]),
        static_cast<uint32_t>(std::stoul(parts[2], nullptr, 0)),
        static_cast<uint32_t>(std::stoul(parts[3], nullptr, 0))});

      motor_names_.push_back(parts[0]);
      RCLCPP_INFO(
        get_logger(), "Configured motor '%s' %s slave=0x%02X master=0x%02X",
        parts[0].c_str(), parts[1].c_str(), descs.back().slave_id, descs.back().master_id);
    }

    motors_.reserve(descs.size());
    for (const auto & desc : descs) {
      motors_.emplace_back(desc.type, desc.slave_id, desc.master_id);
    }
    for (auto & motor : motors_) {
      mc_->addMotor(&motor);
    }
  }

  void configure_motors()
  {
    const auto dm_mode = control_mode_ == "status" ?
      damiao_socketcan::POS_VEL_MODE :
      damiao_socketcan::control_mode_from_string(control_mode_);

    if (switch_mode_on_start_) {
      for (auto & motor : motors_) {
        if (!mc_->switchControlMode(motor, dm_mode)) {
          RCLCPP_WARN(
            get_logger(), "switchControlMode failed for motor 0x%02X; continuing",
            motor.GetSlaveId());
        }
      }
    }

    if (auto_enable_) {
      for (auto & motor : motors_) {
        mc_->enable(motor);
      }
      RCLCPP_INFO(get_logger(), "Enabled %zu motor(s)", motors_.size());
    }
  }

  void timer_callback()
  {
    for (size_t i = 0; i < motors_.size(); ++i) {
      try {
        if (control_mode_ == "velocity") {
          mc_->control_vel(motors_[i], cmd_vel_[i]);
        } else if (control_mode_ == "pos_vel" || control_mode_ == "position") {
          mc_->control_pos_vel(motors_[i], cmd_pos_[i], cmd_vel_[i]);
        } else if (control_mode_ == "mit") {
          mc_->control_mit(
            motors_[i], cmd_mit_kp_[i], cmd_mit_kd_[i], cmd_mit_q_[i], cmd_mit_dq_[i],
            cmd_mit_tau_[i]);
        } else {
          mc_->refresh_motor_status(motors_[i]);
        }
      } catch (const std::exception & exc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "CAN update failed for motor 0x%02X: %s",
          motors_[i].GetSlaveId(), exc.what());
      }
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = motor_names_;
    msg.position.resize(motors_.size());
    msg.velocity.resize(motors_.size());
    msg.effort.resize(motors_.size());
    for (size_t i = 0; i < motors_.size(); ++i) {
      msg.position[i] = motors_[i].Get_Position();
      msg.velocity[i] = motors_[i].Get_Velocity();
      msg.effort[i] = motors_[i].Get_tau();
    }
    joint_state_pub_->publish(msg);
  }

  void vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    for (size_t i = 0; i < std::min(msg->data.size(), motors_.size()); ++i) {
      cmd_vel_[i] = static_cast<float>(msg->data[i]);
    }
  }

  void pos_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    for (size_t i = 0; i < motors_.size() && (i * 2 + 1) < msg->data.size(); ++i) {
      cmd_pos_[i] = static_cast<float>(msg->data[i * 2]);
      cmd_vel_[i] = static_cast<float>(msg->data[i * 2 + 1]);
    }
  }

  void mit_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    for (size_t i = 0; i < motors_.size() && (i * 5 + 4) < msg->data.size(); ++i) {
      const size_t base = i * 5;
      cmd_mit_kp_[i] = static_cast<float>(msg->data[base]);
      cmd_mit_kd_[i] = static_cast<float>(msg->data[base + 1]);
      cmd_mit_q_[i] = static_cast<float>(msg->data[base + 2]);
      cmd_mit_dq_[i] = static_cast<float>(msg->data[base + 3]);
      cmd_mit_tau_[i] = static_cast<float>(msg->data[base + 4]);
    }
  }

  void enable_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      for (auto & motor : motors_) {
        mc_->enable(motor);
      }
      response->success = true;
      response->message = "Enabled configured DM motors";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void disable_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      for (auto & motor : motors_) {
        mc_->disable(motor);
      }
      response->success = true;
      response->message = "Disabled configured DM motors";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  std::string can_interface_;
  std::string control_mode_;
  bool auto_enable_ = false;
  bool switch_mode_on_start_ = false;
  bool disable_on_shutdown_ = true;

  damiao_socketcan::SocketCan::SharedPtr can_;
  std::unique_ptr<damiao_socketcan::MotorControl> mc_;
  std::vector<damiao_socketcan::Motor> motors_;
  std::vector<std::string> motor_names_;

  std::vector<float> cmd_vel_;
  std::vector<float> cmd_pos_;
  std::vector<float> cmd_mit_kp_;
  std::vector<float> cmd_mit_kd_;
  std::vector<float> cmd_mit_q_;
  std::vector<float> cmd_mit_dq_;
  std::vector<float> cmd_mit_tau_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mit_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DamiaoSocketCanNode>());
  rclcpp::shutdown();
  return 0;
}
