#ifndef DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HARDWARE_INTERFACE_HPP_
#define DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "damiao_socketcan_driver/damiao_socketcan.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace damiao_socketcan_driver
{

class DamiaoSocketCanHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DamiaoSocketCanHardwareInterface)

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
  std::string can_interface_ = "can1";
  std::string control_mode_ = "pos_vel";
  bool switch_mode_on_activate_ = true;
  bool capture_zero_on_activate_ = true;

  damiao_socketcan::SocketCan::SharedPtr can_;
  std::unique_ptr<damiao_socketcan::MotorControl> mc_;
  std::vector<damiao_socketcan::Motor> motors_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_velocities_;
  std::vector<double> zero_offsets_;

  std::thread init_thread_;
  std::atomic<bool> motors_ready_{false};
};

}  // namespace damiao_socketcan_driver

#endif  // DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HARDWARE_INTERFACE_HPP_
