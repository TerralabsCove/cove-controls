#include "damiao_socketcan_driver/damiao_socketcan.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace damiao_socketcan
{
namespace
{

LimitParam limit_params[Num_Of_Motor] = {
  {12.5f, 30.0f, 10.0f},   // DM4310
  {12.5f, 50.0f, 10.0f},   // DM4310_48V
  {12.5f, 8.0f, 28.0f},    // DM4340
  {12.5f, 10.0f, 28.0f},   // DM4340_48V
  {12.5f, 45.0f, 20.0f},   // DM6006
  {12.5f, 45.0f, 40.0f},   // DM8006
  {12.5f, 45.0f, 54.0f},   // DM8009
  {12.5f, 25.0f, 200.0f},  // DM10010L
  {12.5f, 20.0f, 200.0f},  // DM10010
  {12.5f, 280.0f, 1.0f},   // DMH3510
  {12.5f, 45.0f, 10.0f},   // DMH6215
  {12.5f, 45.0f, 10.0f},   // DMG6220
};

std::runtime_error errno_error(const std::string & action)
{
  return std::runtime_error(action + ": " + std::strerror(errno));
}

}  // namespace

SocketCan::SocketCan(const std::string & interface_name)
: interface_name_(interface_name)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw errno_error("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed");
  }

  int recv_own_msgs = 0;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw errno_error("setsockopt(CAN_RAW_RECV_OWN_MSGS) failed");
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw errno_error("ioctl(SIOCGIFINDEX) failed for " + interface_name_);
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw errno_error("bind(" + interface_name_ + ") failed");
  }
}

SocketCan::~SocketCan()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void SocketCan::send(MotorId id, const std::array<uint8_t, 8> & data, uint8_t len)
{
  if (len > 8) {
    throw std::invalid_argument("SocketCan::send len must be <= 8");
  }

  can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = len;
  std::copy(data.begin(), data.begin() + len, frame.data);

  const auto written = ::write(fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    throw errno_error("write CAN frame failed on " + interface_name_);
  }
}

std::optional<CanFrame> SocketCan::receive(std::chrono::milliseconds timeout)
{
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ready < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    throw errno_error("poll CAN socket failed on " + interface_name_);
  }
  if (ready == 0 || !(pfd.revents & POLLIN)) {
    return std::nullopt;
  }

  can_frame frame{};
  const auto received = ::read(fd_, &frame, sizeof(frame));
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return std::nullopt;
    }
    throw errno_error("read CAN frame failed on " + interface_name_);
  }
  if (received != static_cast<ssize_t>(sizeof(frame))) {
    return std::nullopt;
  }

  CanFrame out{};
  out.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  out.is_remote = (frame.can_id & CAN_RTR_FLAG) != 0;
  out.is_error = (frame.can_id & CAN_ERR_FLAG) != 0;
  out.id = frame.can_id & (out.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  out.len = frame.can_dlc;
  std::copy(frame.data, frame.data + std::min<uint8_t>(out.len, 8), out.data.begin());
  return out;
}

void SocketCan::drain(std::chrono::milliseconds max_duration)
{
  const auto deadline = std::chrono::steady_clock::now() + max_duration;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!receive(std::chrono::milliseconds(1)).has_value()) {
      return;
    }
  }
}

Motor::Motor(DM_Motor_Type motor_type, MotorId slave_id, MotorId master_id)
: master_id_(master_id),
  slave_id_(slave_id),
  limit_param_(limit_params[motor_type]),
  motor_type_(motor_type)
{
}

Motor::Motor()
: Motor(DM4310, 0x11, 0x01)
{
}

void Motor::receive_data(float q, float dq, float tau)
{
  state_q_ = q;
  state_dq_ = dq;
  state_tau_ = tau;
}

void Motor::set_param(int key, float value)
{
  ValueType v{};
  v.value.float_value = value;
  v.is_float = true;
  param_map_[key] = v;
}

void Motor::set_param(int key, uint32_t value)
{
  ValueType v{};
  v.value.uint32_value = value;
  v.is_float = false;
  param_map_[key] = v;
}

void Motor::clear_param(int key)
{
  param_map_.erase(key);
}

bool Motor::is_have_param(int key) const
{
  return param_map_.find(key) != param_map_.end();
}

float Motor::get_param_as_float(int key) const
{
  const auto it = param_map_.find(key);
  if (it == param_map_.end() || !it->second.is_float) {
    return 0.0f;
  }
  return it->second.value.float_value;
}

uint32_t Motor::get_param_as_uint32(int key) const
{
  const auto it = param_map_.find(key);
  if (it == param_map_.end() || it->second.is_float) {
    return 0;
  }
  return it->second.value.uint32_value;
}

MotorControl::MotorControl(SocketCan::SharedPtr can)
: can_(std::move(can))
{
  if (!can_) {
    throw std::invalid_argument("MotorControl requires a SocketCan transport");
  }
}

void MotorControl::addMotor(Motor * motor)
{
  motors_.insert({motor->GetSlaveId(), motor});
  if (motor->GetMasterId() != 0) {
    motors_.insert({motor->GetMasterId(), motor});
  }
}

void MotorControl::enable(const Motor & motor)
{
  control_cmd(motor.GetSlaveId(), 0xFC);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  receive_for(std::chrono::milliseconds(10));
}

void MotorControl::enable_old(const Motor & motor, Control_Mode mode)
{
  const MotorId id = ((mode - 1) << 2) + motor.GetSlaveId();
  control_cmd(id, 0xFC);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  receive_for(std::chrono::milliseconds(10));
}

void MotorControl::disable(const Motor & motor)
{
  control_cmd(motor.GetSlaveId(), 0xFD);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  receive_for(std::chrono::milliseconds(10));
}

void MotorControl::set_zero_position(const Motor & motor)
{
  control_cmd(motor.GetSlaveId(), 0xFE);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  receive_for(std::chrono::milliseconds(10));
}

void MotorControl::refresh_motor_status(const Motor & motor)
{
  std::array<uint8_t, 8> data{
    static_cast<uint8_t>(motor.GetSlaveId() & 0xFF),
    static_cast<uint8_t>((motor.GetSlaveId() >> 8) & 0xFF),
    0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_frame(0x7FF, data);
  receive_for(std::chrono::milliseconds(3));
}

void MotorControl::control_mit(Motor & motor, float kp, float kd, float q, float dq, float tau)
{
  const MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("MIT control error: motor id not found");
  }

  const auto limits = motor.get_limit_param();
  const auto kp_uint = float_to_uint(kp, 0.0f, 500.0f, 12);
  const auto kd_uint = float_to_uint(kd, 0.0f, 5.0f, 12);
  const auto q_uint = float_to_uint(q, -limits.q_max, limits.q_max, 16);
  const auto dq_uint = float_to_uint(dq, -limits.dq_max, limits.dq_max, 12);
  const auto tau_uint = float_to_uint(tau, -limits.tau_max, limits.tau_max, 12);

  std::array<uint8_t, 8> data{};
  data[0] = (q_uint >> 8) & 0xFF;
  data[1] = q_uint & 0xFF;
  data[2] = dq_uint >> 4;
  data[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF);
  data[4] = kp_uint & 0xFF;
  data[5] = kd_uint >> 4;
  data[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF);
  data[7] = tau_uint & 0xFF;

  send_frame(id, data);
  receive_for(std::chrono::milliseconds(2));
}

void MotorControl::control_pos_vel(Motor & motor, float pos, float vel)
{
  const MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("POS_VEL control error: motor id not found");
  }

  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &pos, sizeof(float));
  std::memcpy(data.data() + 4, &vel, sizeof(float));
  send_frame(id + POS_MODE, data);
  receive_for(std::chrono::milliseconds(2));
}

void MotorControl::control_vel(Motor & motor, float vel)
{
  const MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("velocity control error: motor id not found");
  }

  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &vel, sizeof(float));
  send_frame(id + SPEED_MODE, data);
  receive_for(std::chrono::milliseconds(2));
}

void MotorControl::control_pos_force(Motor & motor, float pos, uint16_t vel, uint16_t current)
{
  const MotorId id = motor.GetSlaveId();
  if (motors_.find(id) == motors_.end()) {
    throw std::runtime_error("POS_FORCE control error: motor id not found");
  }

  std::array<uint8_t, 8> data{};
  std::memcpy(data.data(), &pos, sizeof(float));
  std::memcpy(data.data() + 4, &vel, sizeof(uint16_t));
  std::memcpy(data.data() + 6, &current, sizeof(uint16_t));
  send_frame(id + POSI_MODE, data);
  receive_for(std::chrono::milliseconds(2));
}

bool MotorControl::receive(std::chrono::milliseconds timeout)
{
  const auto frame = can_->receive(timeout);
  if (!frame.has_value() || frame->is_error || frame->is_remote || frame->len < 8) {
    return false;
  }
  if (decode_param(*frame)) {
    return true;
  }
  return decode_feedback(*frame);
}

void MotorControl::receive_for(std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    receive(std::chrono::milliseconds(1));
  }
}

float MotorControl::read_motor_param(Motor & motor, uint8_t rid)
{
  motor.clear_param(rid);
  std::array<uint8_t, 8> data{
    static_cast<uint8_t>(motor.GetSlaveId() & 0xFF),
    static_cast<uint8_t>((motor.GetSlaveId() >> 8) & 0xFF),
    0x33, rid, 0x00, 0x00, 0x00, 0x00};
  send_frame(0x7FF, data);

  for (int i = 0; i < MAX_RETRIES; ++i) {
    std::this_thread::sleep_for(RETRY_INTERVAL);
    receive_for(std::chrono::milliseconds(5));
    if (motor.is_have_param(rid)) {
      return is_in_ranges(rid) ? static_cast<float>(motor.get_param_as_uint32(rid)) :
        motor.get_param_as_float(rid);
    }
  }
  return 0.0f;
}

bool MotorControl::switchControlMode(Motor & motor, Control_Mode mode)
{
  const uint8_t write_data[4] = {static_cast<uint8_t>(mode), 0x00, 0x00, 0x00};
  const uint8_t rid = CTRL_MODE;
  motor.clear_param(rid);
  write_motor_param(motor, rid, write_data);

  for (int i = 0; i < MAX_RETRIES; ++i) {
    std::this_thread::sleep_for(RETRY_INTERVAL);
    receive_for(std::chrono::milliseconds(5));
    if (motor.is_have_param(rid)) {
      return motor.get_param_as_uint32(rid) == static_cast<uint32_t>(mode);
    }
  }
  return false;
}

bool MotorControl::change_motor_param(Motor & motor, uint8_t rid, float data)
{
  motor.clear_param(rid);
  uint32_t data_uint32 = float_to_uint32(data);
  uint8_t * bytes = nullptr;
  if (is_in_ranges(rid)) {
    bytes = reinterpret_cast<uint8_t *>(&data_uint32);
  } else {
    bytes = reinterpret_cast<uint8_t *>(&data);
  }
  write_motor_param(motor, rid, bytes);

  for (int i = 0; i < MAX_RETRIES; ++i) {
    std::this_thread::sleep_for(RETRY_INTERVAL);
    receive_for(std::chrono::milliseconds(5));
    if (motor.is_have_param(rid)) {
      if (is_in_ranges(rid)) {
        return motor.get_param_as_uint32(rid) == data_uint32;
      }
      return std::abs(motor.get_param_as_float(rid) - data) < 0.1f;
    }
  }
  return false;
}

void MotorControl::save_motor_param(Motor & motor)
{
  disable(motor);
  std::array<uint8_t, 8> data{
    static_cast<uint8_t>(motor.GetSlaveId() & 0xFF),
    static_cast<uint8_t>((motor.GetSlaveId() >> 8) & 0xFF),
    0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
  send_frame(0x7FF, data);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void MotorControl::changeMotorLimit(Motor & motor, float p_max, float q_max, float t_max)
{
  (void)motor;
  limit_params[motor.GetMotorType()] = {p_max, q_max, t_max};
}

void MotorControl::changeMotorPMAX(Motor & motor, float p_max)
{
  limit_params[motor.GetMotorType()].q_max = p_max;
}

void MotorControl::send_frame(MotorId id, const std::array<uint8_t, 8> & data)
{
  can_->send(id, data, 8);
}

void MotorControl::control_cmd(MotorId id, uint8_t cmd)
{
  std::array<uint8_t, 8> data{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
  send_frame(id, data);
}

void MotorControl::write_motor_param(Motor & motor, uint8_t rid, const uint8_t data[4])
{
  std::array<uint8_t, 8> frame_data{
    static_cast<uint8_t>(motor.GetSlaveId() & 0xFF),
    static_cast<uint8_t>((motor.GetSlaveId() >> 8) & 0xFF),
    0x55, rid, data[0], data[1], data[2], data[3]};
  send_frame(0x7FF, frame_data);
}

bool MotorControl::decode_feedback(const CanFrame & frame)
{
  auto it = motors_.find(frame.id);
  if (it == motors_.end() && frame.id == 0x00) {
    it = motors_.find(frame.data[0] & 0x0F);
  }
  if (it == motors_.end()) {
    return false;
  }

  auto * motor = it->second;
  const auto limits = motor->get_limit_param();
  const uint16_t q_uint = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
  const uint16_t dq_uint = (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const uint16_t tau_uint = (static_cast<uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];

  motor->receive_data(
    uint_to_float(q_uint, -limits.q_max, limits.q_max, 16),
    uint_to_float(dq_uint, -limits.dq_max, limits.dq_max, 12),
    uint_to_float(tau_uint, -limits.tau_max, limits.tau_max, 12));
  return true;
}

bool MotorControl::decode_param(const CanFrame & frame)
{
  if (!(frame.data[2] == 0x33 || frame.data[2] == 0x55)) {
    return false;
  }

  const uint32_t slave_id = (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[0];
  const auto it = motors_.find(slave_id);
  if (it == motors_.end()) {
    return false;
  }

  const uint8_t rid = frame.data[3];
  if (is_in_ranges(rid)) {
    const uint32_t data_uint32 =
      (static_cast<uint32_t>(frame.data[7]) << 24) |
      (static_cast<uint32_t>(frame.data[6]) << 16) |
      (static_cast<uint32_t>(frame.data[5]) << 8) |
      frame.data[4];
    it->second->set_param(rid, data_uint32);
  } else {
    it->second->set_param(rid, uint8_to_float(frame.data.data() + 4));
  }
  return true;
}

bool MotorControl::is_in_ranges(int number)
{
  return (7 <= number && number <= 10) ||
    (13 <= number && number <= 16) ||
    (35 <= number && number <= 36);
}

uint16_t MotorControl::float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
  const float clamped = std::max(xmin, std::min(x, xmax));
  const float span = xmax - xmin;
  const float data_norm = (clamped - xmin) / span;
  return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

float MotorControl::uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits)
{
  const float span = xmax - xmin;
  const float data_norm = static_cast<float>(x) / static_cast<float>((1 << bits) - 1);
  return data_norm * span + xmin;
}

uint32_t MotorControl::float_to_uint32(float value)
{
  return static_cast<uint32_t>(value);
}

float MotorControl::uint8_to_float(const uint8_t data[4])
{
  const uint32_t combined =
    (static_cast<uint32_t>(data[3]) << 24) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[1]) << 8) |
    data[0];
  float result = 0.0f;
  std::memcpy(&result, &combined, sizeof(result));
  return result;
}

DM_Motor_Type motor_type_from_string(const std::string & value)
{
  if (value == "DM4310") return DM4310;
  if (value == "DM4310_48V") return DM4310_48V;
  if (value == "DM4340") return DM4340;
  if (value == "DM4340_48V") return DM4340_48V;
  if (value == "DM6006") return DM6006;
  if (value == "DM8006") return DM8006;
  if (value == "DM8009") return DM8009;
  if (value == "DM10010L") return DM10010L;
  if (value == "DM10010") return DM10010;
  if (value == "DMH3510") return DMH3510;
  if (value == "DMH6215") return DMH6215;
  if (value == "DMG6220") return DMG6220;
  throw std::runtime_error("Unknown motor type: " + value);
}

std::string motor_type_to_string(DM_Motor_Type value)
{
  switch (value) {
    case DM4310: return "DM4310";
    case DM4310_48V: return "DM4310_48V";
    case DM4340: return "DM4340";
    case DM4340_48V: return "DM4340_48V";
    case DM6006: return "DM6006";
    case DM8006: return "DM8006";
    case DM8009: return "DM8009";
    case DM10010L: return "DM10010L";
    case DM10010: return "DM10010";
    case DMH3510: return "DMH3510";
    case DMH6215: return "DMH6215";
    case DMG6220: return "DMG6220";
    default: return "UNKNOWN";
  }
}

Control_Mode control_mode_from_string(const std::string & value)
{
  if (value == "mit") return MIT_MODE;
  if (value == "pos_vel" || value == "position") return POS_VEL_MODE;
  if (value == "velocity") return VEL_MODE;
  if (value == "pos_force") return POS_FORCE_MODE;
  throw std::runtime_error("Unknown control mode: " + value);
}

}  // namespace damiao_socketcan
