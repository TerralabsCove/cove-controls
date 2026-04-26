#include "damiao_driver/damiao_hardware_interface.hpp"

#include <cstring>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace damiao_driver
{

static damiao::DM_Motor_Type motor_type_from_string(const std::string & s)
{
  if (s == "DM4310")     return damiao::DM4310;
  if (s == "DM4310_48V") return damiao::DM4310_48V;
  if (s == "DM4340")     return damiao::DM4340;
  if (s == "DM4340_48V") return damiao::DM4340_48V;
  if (s == "DM6006")     return damiao::DM6006;
  if (s == "DM8006")     return damiao::DM8006;
  if (s == "DM8009")     return damiao::DM8009;
  if (s == "DM10010L")   return damiao::DM10010L;
  if (s == "DM10010")    return damiao::DM10010;
  if (s == "DMH3510")    return damiao::DMH3510;
  if (s == "DMH6215")    return damiao::DMH6215;
  if (s == "DMG6220")    return damiao::DMG6220;
  throw std::runtime_error("Unknown motor type: " + s);
}

// ---------------------------------------------------------------------------
// on_init — called once at startup, read URDF hardware parameters
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DamiaoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware-level params from URDF <ros2_control> block
  std::string serial_port = "/dev/ttyACM0";
  int baud_rate = 921600;
  control_mode_ = "pos_vel";

  if (info_.hardware_parameters.count("serial_port"))
    serial_port = info_.hardware_parameters.at("serial_port");
  if (info_.hardware_parameters.count("baud_rate"))
    baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  if (info_.hardware_parameters.count("control_mode"))
    control_mode_ = info_.hardware_parameters.at("control_mode");

  // Open serial port
  speed_t baud_code = B921600;
  if (baud_rate == 115200) baud_code = B115200;
  else if (baud_rate == 460800) baud_code = B460800;
  else if (baud_rate == 921600) baud_code = B921600;

  serial_ = std::make_shared<SerialPort>(serial_port, baud_code);
  mc_ = std::make_unique<damiao::Motor_Control>(serial_);

  // Read per-joint motor params from URDF joint tags
  size_t n = info_.joints.size();
  motors_.clear();
  motors_.reserve(n);
  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_efforts_.resize(n, 0.0);
  hw_cmd_positions_.resize(n, 0.0);
  hw_cmd_velocities_.resize(n, 0.0);
  zero_offsets_.resize(n, 0.0);

  for (size_t i = 0; i < n; i++) {
    const auto & joint = info_.joints[i];

    std::string motor_type_str = joint.parameters.at("motor_type");
    uint32_t slave_id = static_cast<uint32_t>(
      std::stoul(joint.parameters.at("slave_id"), nullptr, 0));
    uint32_t master_id = static_cast<uint32_t>(
      std::stoul(joint.parameters.at("master_id"), nullptr, 0));

    auto mtype = motor_type_from_string(motor_type_str);
    motors_.emplace_back(mtype, slave_id, master_id);

    RCLCPP_INFO(
      rclcpp::get_logger("DamiaoHardwareInterface"),
      "Joint '%s': %s slave=0x%02X master=0x%02X",
      joint.name.c_str(), motor_type_str.c_str(), slave_id, master_id);
  }

  // Register motor pointers only after vector growth is complete.
  // This avoids pointer invalidation when motors_ reallocates.
  for (auto & motor : motors_) {
    mc_->addMotor(&motor);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Export state interfaces — what ros2_control can READ from us
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
DamiaoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Export command interfaces — what ros2_control can WRITE to us
// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
DamiaoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & name = info_.joints[i].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_cmd_positions_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocities_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// on_activate — enable motors
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DamiaoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_ready_ = false;

  damiao::Control_Mode dm_mode = damiao::POS_VEL_MODE;
  if (control_mode_ == "velocity")                     dm_mode = damiao::VEL_MODE;
  else if (control_mode_ == "pos_vel" ||
           control_mode_ == "position")                dm_mode = damiao::POS_VEL_MODE;
  else if (control_mode_ == "mit")                     dm_mode = damiao::MIT_MODE;
  else if (control_mode_ == "pos_force")               dm_mode = damiao::POS_FORCE_MODE;

  // Run blocking motor init in a background thread so the controller manager
  // doesn't crash waiting for on_activate to return.
  init_thread_ = std::thread([this, dm_mode]() {
    RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"), "Motor init thread started.");

    for (size_t i = 0; i < motors_.size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"),
        "Disabling joint '%s' (slave=0x%02X)...", info_.joints[i].name.c_str(), motors_[i].GetSlaveId());
      mc_->disable(motors_[i]);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (size_t i = 0; i < motors_.size(); i++) {
      bool ok = mc_->switchControlMode(motors_[i], dm_mode);
      RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"),
        "switchControlMode joint '%s' (slave=0x%02X): %s",
        info_.joints[i].name.c_str(), motors_[i].GetSlaveId(),
        ok ? "OK" : "FAILED (motor may be at wrong CAN ID)");
    }

    for (size_t i = 0; i < motors_.size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"),
        "Enabling joint '%s' (slave=0x%02X)...", info_.joints[i].name.c_str(), motors_[i].GetSlaveId());
      mc_->enable(motors_[i]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Capture the motor's current encoder reading as the zero offset.
    // The arm must be physically at the vertical (zero) position before starting.
    // After this, read() reports (motor_pos - offset) → 0.0 at startup,
    // and write() sends (logical_cmd + offset) → holds current physical position.
    // hw_positions_ and hw_cmd_positions_ stay 0.0 so the JTC initializes its
    // setpoint to zero and the first write() keeps the motor exactly where it is.
    for (int attempt = 0; attempt < 5; attempt++) {
      for (size_t i = 0; i < motors_.size(); i++) {
        mc_->refresh_motor_status(motors_[i]);
        zero_offsets_[i] = static_cast<double>(motors_[i].Get_Position());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::string offset_str;
    for (size_t i = 0; i < zero_offsets_.size(); i++) {
      offset_str += info_.joints[i].name + "=" + std::to_string(zero_offsets_[i]) + " ";
    }
    RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"),
      "Zero offsets captured: %s", offset_str.c_str());

    motors_ready_ = true;
    RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"), "Motors ready.");
  });

  RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"),
    "on_activate returning — motor init running in background.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate — disable motors
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DamiaoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DamiaoHardwareInterface"), "Deactivating motors...");
  if (init_thread_.joinable()) init_thread_.join();
  for (auto & m : motors_) {
    try { mc_->disable(m); } catch (...) {}
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read — get current state from hardware → update state interfaces
// ---------------------------------------------------------------------------
hardware_interface::return_type DamiaoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motors_ready_) return hardware_interface::return_type::OK;
  for (size_t i = 0; i < motors_.size(); i++) {
    mc_->refresh_motor_status(motors_[i]);
    hw_positions_[i]  = static_cast<double>(motors_[i].Get_Position()) - zero_offsets_[i];
    hw_velocities_[i] = static_cast<double>(motors_[i].Get_Velocity());
    hw_efforts_[i]    = static_cast<double>(motors_[i].Get_tau());
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write — take commands from command interfaces → send to hardware
// ---------------------------------------------------------------------------
hardware_interface::return_type DamiaoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!motors_ready_) return hardware_interface::return_type::OK;
  for (size_t i = 0; i < motors_.size(); i++) {
    if (control_mode_ == "velocity") {
      mc_->control_vel(motors_[i], static_cast<float>(hw_cmd_velocities_[i]));
    } else {
      float cmd_vel = static_cast<float>(hw_cmd_velocities_[i]);
      // If no velocity is commanded (JointTrajectoryController only sends position),
      // use a default speed so the motor can actually move to the target position.
      if (std::abs(cmd_vel) < 0.01f) cmd_vel = 5.0f;  // rad/s
      // Add zero offset: convert logical command back to motor encoder space.
      float motor_cmd = static_cast<float>(hw_cmd_positions_[i] + zero_offsets_[i]);
      mc_->control_pos_vel(motors_[i], motor_cmd, cmd_vel);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace damiao_driver

PLUGINLIB_EXPORT_CLASS(
  damiao_driver::DamiaoHardwareInterface,
  hardware_interface::SystemInterface)
