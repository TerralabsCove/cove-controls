#ifndef DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HPP_
#define DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace damiao_socketcan
{

using MotorId = uint32_t;

constexpr MotorId POS_MODE = 0x100;
constexpr MotorId SPEED_MODE = 0x200;
constexpr MotorId POSI_MODE = 0x300;
constexpr int MAX_RETRIES = 20;
constexpr auto RETRY_INTERVAL = std::chrono::microseconds(50000);

enum DM_Motor_Type
{
  DM4310,
  DM4310_48V,
  DM4340,
  DM4340_48V,
  DM6006,
  DM8006,
  DM8009,
  DM10010L,
  DM10010,
  DMH3510,
  DMH6215,
  DMG6220,
  Num_Of_Motor
};

enum Control_Mode
{
  MIT_MODE = 1,
  POS_VEL_MODE = 2,
  VEL_MODE = 3,
  POS_FORCE_MODE = 4,
};

enum DM_REG
{
  UV_Value = 0,
  KT_Value = 1,
  OT_Value = 2,
  OC_Value = 3,
  ACC = 4,
  DEC = 5,
  MAX_SPD = 6,
  MST_ID = 7,
  ESC_ID = 8,
  TIMEOUT = 9,
  CTRL_MODE = 10,
  Damp = 11,
  Inertia = 12,
  hw_ver = 13,
  sw_ver = 14,
  SN = 15,
  NPP = 16,
  Rs = 17,
  LS = 18,
  Flux = 19,
  Gr = 20,
  PMAX = 21,
  VMAX = 22,
  TMAX = 23,
  I_BW = 24,
  KP_ASR = 25,
  KI_ASR = 26,
  KP_APR = 27,
  KI_APR = 28,
  OV_Value = 29,
  GREF = 30,
  Deta = 31,
  V_BW = 32,
  IQ_c1 = 33,
  VL_c1 = 34,
  can_br = 35,
  sub_ver = 36,
  u_off = 50,
  v_off = 51,
  k1 = 52,
  k2 = 53,
  m_off = 54,
  dir = 55,
  p_m = 80,
  xout = 81,
};

struct LimitParam
{
  float q_max;
  float dq_max;
  float tau_max;
};

struct CanFrame
{
  MotorId id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};
  bool is_extended = false;
  bool is_remote = false;
  bool is_error = false;
};

class SocketCan
{
public:
  using SharedPtr = std::shared_ptr<SocketCan>;

  explicit SocketCan(const std::string & interface_name);
  ~SocketCan();

  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  void send(MotorId id, const std::array<uint8_t, 8> & data, uint8_t len = 8);
  std::optional<CanFrame> receive(std::chrono::milliseconds timeout);
  void drain(std::chrono::milliseconds max_duration = std::chrono::milliseconds(20));

  const std::string & interface_name() const { return interface_name_; }

private:
  int fd_ = -1;
  std::string interface_name_;
};

class Motor
{
public:
  Motor(DM_Motor_Type motor_type, MotorId slave_id, MotorId master_id);
  Motor();

  void receive_data(float q, float dq, float tau);

  DM_Motor_Type GetMotorType() const { return motor_type_; }
  MotorId GetMasterId() const { return master_id_; }
  MotorId GetSlaveId() const { return slave_id_; }
  float Get_Position() const { return state_q_; }
  float Get_Velocity() const { return state_dq_; }
  float Get_tau() const { return state_tau_; }
  LimitParam get_limit_param() const { return limit_param_; }

  void set_param(int key, float value);
  void set_param(int key, uint32_t value);
  void clear_param(int key);
  bool is_have_param(int key) const;
  float get_param_as_float(int key) const;
  uint32_t get_param_as_uint32(int key) const;

private:
  union ValueUnion
  {
    float float_value;
    uint32_t uint32_value;
  };

  struct ValueType
  {
    ValueUnion value;
    bool is_float;
  };

  MotorId master_id_;
  MotorId slave_id_;
  float state_q_ = 0.0f;
  float state_dq_ = 0.0f;
  float state_tau_ = 0.0f;
  LimitParam limit_param_{};
  DM_Motor_Type motor_type_;
  std::unordered_map<uint32_t, ValueType> param_map_;
};

class MotorControl
{
public:
  explicit MotorControl(SocketCan::SharedPtr can);

  void addMotor(Motor * motor);

  void enable(const Motor & motor);
  void enable_old(const Motor & motor, Control_Mode mode);
  void disable(const Motor & motor);
  void set_zero_position(const Motor & motor);
  void refresh_motor_status(const Motor & motor);

  void control_mit(Motor & motor, float kp, float kd, float q, float dq, float tau);
  void control_pos_vel(Motor & motor, float pos, float vel);
  void control_vel(Motor & motor, float vel);
  void control_pos_force(Motor & motor, float pos, uint16_t vel, uint16_t current);

  bool receive(std::chrono::milliseconds timeout = std::chrono::milliseconds(2));
  void receive_for(std::chrono::milliseconds duration);

  float read_motor_param(Motor & motor, uint8_t rid);
  bool switchControlMode(Motor & motor, Control_Mode mode);
  bool change_motor_param(Motor & motor, uint8_t rid, float data);
  void save_motor_param(Motor & motor);

  static void changeMotorLimit(Motor & motor, float p_max, float q_max, float t_max);
  static void changeMotorPMAX(Motor & motor, float p_max);

private:
  void send_frame(MotorId id, const std::array<uint8_t, 8> & data);
  void control_cmd(MotorId id, uint8_t cmd);
  void write_motor_param(Motor & motor, uint8_t rid, const uint8_t data[4]);
  bool decode_feedback(const CanFrame & frame);
  bool decode_param(const CanFrame & frame);

  static bool is_in_ranges(int number);
  static uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits);
  static float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits);
  static uint32_t float_to_uint32(float value);
  static float uint8_to_float(const uint8_t data[4]);

  std::unordered_map<MotorId, Motor *> motors_;
  SocketCan::SharedPtr can_;
};

DM_Motor_Type motor_type_from_string(const std::string & value);
std::string motor_type_to_string(DM_Motor_Type value);
Control_Mode control_mode_from_string(const std::string & value);

}  // namespace damiao_socketcan

#endif  // DAMIAO_SOCKETCAN_DRIVER_DAMIAO_SOCKETCAN_HPP_
