#ifndef DAMIAO_HARDWARE_INTERFACE_HPP
#define DAMIAO_HARDWARE_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "damiao_driver/damiao.h"

namespace damiao_driver
{

class DamiaoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DamiaoHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial + motor controller
  std::shared_ptr<SerialPort> serial_;
  std::unique_ptr<damiao::Motor_Control> mc_;
  std::vector<damiao::Motor> motors_;

  // Control mode
  std::string control_mode_;

  // State mirrors (what we last read from hardware)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Command mirrors (what ros2_control wants us to send)
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_velocities_;

  // Per-joint encoder offsets captured at startup.
  // Logical position = motor_pos - zero_offsets_[i]
  // Motor command    = logical_cmd + zero_offsets_[i]
  std::vector<double> zero_offsets_;

  // Background motor init — keeps on_activate non-blocking
  std::thread init_thread_;
  std::atomic<bool> motors_ready_{false};
};

}  // namespace damiao_driver

#endif  // DAMIAO_HARDWARE_INTERFACE_HPP
