#include "damiao_socketcan_driver/damiao_socketcan_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace damiao_socketcan_driver
{

hardware_interface::CallbackReturn DamiaoSocketCanHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("control_mode")) {
    control_mode_ = info_.hardware_parameters.at("control_mode");
  }
  if (info_.hardware_parameters.count("switch_mode_on_activate")) {
    switch_mode_on_activate_ = info_.hardware_parameters.at("switch_mode_on_activate") == "true" ||
      info_.hardware_parameters.at("switch_mode_on_activate") == "1";
  }

  can_ = std::make_shared<damiao_socketcan::SocketCan>(can_interface_);
  mc_ = std::make_unique<damiao_socketcan::MotorControl>(can_);

  const size_t n = info_.joints.size();
  motors_.clear();
  motors_.reserve(n);
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_cmd_positions_.assign(n, 0.0);
  hw_cmd_velocities_.assign(n, 0.0);
  zero_offsets_.assign(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];
    const auto motor_type_str = joint.parameters.at("motor_type");
    const auto slave_id = static_cast<uint32_t>(
      std::stoul(joint.parameters.at("slave_id"), nullptr, 0));
    const auto master_id = static_cast<uint32_t>(
      std::stoul(joint.parameters.at("master_id"), nullptr, 0));
    const auto motor_type = damiao_socketcan::motor_type_from_string(motor_type_str);
    motors_.emplace_back(motor_type, slave_id, master_id);

    RCLCPP_INFO(
      rclcpp::get_logger("DamiaoSocketCanHardwareInterface"),
      "Joint '%s': %s slave=0x%02X master=0x%02X on %s",
      joint.name.c_str(), motor_type_str.c_str(), slave_id, master_id, can_interface_.c_str());
  }

  for (auto & motor : motors_) {
    mc_->addMotor(&motor);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DamiaoSocketCanHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DamiaoSocketCanHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_cmd_positions_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocities_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn DamiaoSocketCanHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_ready_ = false;

  damiao_socketcan::Control_Mode dm_mode = damiao_socketcan::POS_VEL_MODE;
  if (control_mode_ != "position") {
    dm_mode = damiao_socketcan::control_mode_from_string(control_mode_);
  }

  init_thread_ = std::thread([this, dm_mode]() {
    auto logger = rclcpp::get_logger("DamiaoSocketCanHardwareInterface");
    RCLCPP_INFO(logger, "SocketCAN motor init thread started on %s.", can_interface_.c_str());

    for (size_t i = 0; i < motors_.size(); ++i) {
      RCLCPP_INFO(
        logger, "Disabling joint '%s' (slave=0x%02X)...",
        info_.joints[i].name.c_str(), motors_[i].GetSlaveId());
      mc_->disable(motors_[i]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (switch_mode_on_activate_) {
      for (size_t i = 0; i < motors_.size(); ++i) {
        const bool ok = mc_->switchControlMode(motors_[i], dm_mode);
        RCLCPP_INFO(
          logger, "switchControlMode joint '%s' (slave=0x%02X): %s",
          info_.joints[i].name.c_str(), motors_[i].GetSlaveId(), ok ? "OK" : "FAILED");
      }
    }

    for (size_t i = 0; i < motors_.size(); ++i) {
      RCLCPP_INFO(
        logger, "Enabling joint '%s' (slave=0x%02X)...",
        info_.joints[i].name.c_str(), motors_[i].GetSlaveId());
      mc_->enable(motors_[i]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const char * skip = std::getenv("DAMIAO_SKIP_ZERO_CAPTURE");
    if (skip && std::string(skip) != "0" && std::string(skip) != "") {
      RCLCPP_WARN(
        logger,
        "DAMIAO_SKIP_ZERO_CAPTURE set: zero offsets stay at 0 and raw encoder positions are reported.");
    } else {
      for (int attempt = 0; attempt < 5; ++attempt) {
        for (size_t i = 0; i < motors_.size(); ++i) {
          mc_->refresh_motor_status(motors_[i]);
          zero_offsets_[i] = static_cast<double>(motors_[i].Get_Position());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    std::string offsets;
    for (size_t i = 0; i < zero_offsets_.size(); ++i) {
      offsets += info_.joints[i].name + "=" + std::to_string(zero_offsets_[i]) + " ";
    }
    RCLCPP_INFO(logger, "Zero offsets: %s", offsets.c_str());

    motors_ready_ = true;
    RCLCPP_INFO(logger, "SocketCAN motors ready.");
  });

  RCLCPP_INFO(
    rclcpp::get_logger("DamiaoSocketCanHardwareInterface"),
    "on_activate returning: motor init is running in background.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSocketCanHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("DamiaoSocketCanHardwareInterface");
  RCLCPP_INFO(logger, "Deactivating SocketCAN motors...");
  if (init_thread_.joinable()) {
    init_thread_.join();
  }
  motors_ready_ = false;
  for (auto & motor : motors_) {
    try {
      mc_->disable(motor);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(logger, "Disable failed for 0x%02X: %s", motor.GetSlaveId(), exc.what());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DamiaoSocketCanHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motors_ready_) {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < motors_.size(); ++i) {
    mc_->refresh_motor_status(motors_[i]);
    hw_positions_[i] = static_cast<double>(motors_[i].Get_Position()) - zero_offsets_[i];
    hw_velocities_[i] = static_cast<double>(motors_[i].Get_Velocity());
    hw_efforts_[i] = static_cast<double>(motors_[i].Get_tau());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DamiaoSocketCanHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motors_ready_) {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < motors_.size(); ++i) {
    if (control_mode_ == "velocity") {
      mc_->control_vel(motors_[i], static_cast<float>(hw_cmd_velocities_[i]));
    } else {
      float cmd_vel = static_cast<float>(hw_cmd_velocities_[i]);
      if (std::abs(cmd_vel) < 0.01f) {
        cmd_vel = 5.0f;
      }
      const float motor_cmd = static_cast<float>(hw_cmd_positions_[i] + zero_offsets_[i]);
      mc_->control_pos_vel(motors_[i], motor_cmd, cmd_vel);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace damiao_socketcan_driver

PLUGINLIB_EXPORT_CLASS(
  damiao_socketcan_driver::DamiaoSocketCanHardwareInterface,
  hardware_interface::SystemInterface)
