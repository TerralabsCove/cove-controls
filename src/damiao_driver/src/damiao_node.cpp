#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "damiao_driver/damiao.h"

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

// ---------------------------------------------------------------------------
// Motor descriptor loaded from ROS parameters
// ---------------------------------------------------------------------------
struct MotorDesc
{
    std::string name;
    damiao::DM_Motor_Type type;
    uint32_t slave_id;
    uint32_t master_id;
};

static damiao::DM_Motor_Type motor_type_from_string(const std::string& s)
{
    if (s == "DM4310")      return damiao::DM4310;
    if (s == "DM4310_48V")  return damiao::DM4310_48V;
    if (s == "DM4340")      return damiao::DM4340;
    if (s == "DM4340_48V")  return damiao::DM4340_48V;
    if (s == "DM6006")      return damiao::DM6006;
    if (s == "DM8006")      return damiao::DM8006;
    if (s == "DM8009")      return damiao::DM8009;
    if (s == "DM10010L")    return damiao::DM10010L;
    if (s == "DM10010")     return damiao::DM10010;
    if (s == "DMH3510")     return damiao::DMH3510;
    if (s == "DMH6215")     return damiao::DMH6215;
    if (s == "DMG6220")     return damiao::DMG6220;
    throw std::runtime_error("Unknown motor type: " + s);
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------
class DamiaoNode : public rclcpp::Node
{
public:
    DamiaoNode() : Node("damiao_driver")
    {
        // -- Parameters --
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 921600);
        this->declare_parameter<double>("loop_rate", 100.0);
        this->declare_parameter<std::string>("control_mode", "velocity");

        // Motor list: comma-separated "name:type:slave_id:master_id" entries
        // Example: "pan:DM4340:0x02:0x12,tilt:DM10010:0x01:0x11"
        this->declare_parameter<std::string>("motors", "");

        auto port = this->get_parameter("serial_port").as_string();
        auto baud = this->get_parameter("baud_rate").as_int();
        double loop_rate = this->get_parameter("loop_rate").as_double();
        control_mode_ = this->get_parameter("control_mode").as_string();

        // -- Serial & motor controller --
        speed_t baud_code = B921600;  // default
        if (baud == 115200) baud_code = B115200;
        else if (baud == 460800) baud_code = B460800;
        else if (baud == 921600) baud_code = B921600;

        serial_ = std::make_shared<SerialPort>(port, baud_code);
        mc_ = std::make_unique<damiao::Motor_Control>(serial_);

        // -- Parse motor descriptors --
        auto motors_str = this->get_parameter("motors").as_string();
        if (motors_str.empty()) {
            RCLCPP_FATAL(this->get_logger(), "No motors configured. Set the 'motors' parameter.");
            throw std::runtime_error("No motors configured");
        }
        parse_motors(motors_str);

        // -- Set control mode and enable --
        damiao::Control_Mode dm_mode = damiao::VEL_MODE;
        if (control_mode_ == "velocity")       dm_mode = damiao::VEL_MODE;
        else if (control_mode_ == "pos_vel")   dm_mode = damiao::POS_VEL_MODE;
        else if (control_mode_ == "mit")       dm_mode = damiao::MIT_MODE;
        else if (control_mode_ == "pos_force") dm_mode = damiao::POS_FORCE_MODE;

        for (auto& m : motors_) {
            mc_->disable(m);
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        for (auto& m : motors_) {
            if (!mc_->switchControlMode(m, dm_mode)) {
                RCLCPP_WARN(this->get_logger(), "switchControlMode failed for motor 0x%02X, continuing...",
                            m.GetSlaveId());
            }
        }

        for (auto& m : motors_) {
            mc_->enable(m);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(this->get_logger(), "Enabled %zu motor(s) in %s mode on %s",
                     motors_.size(), control_mode_.c_str(), port.c_str());

        // -- Initialize command storage (all zeros = stopped) --
        size_t n = motors_.size();
        cmd_vel_.resize(n, 0.0f);
        cmd_pos_.resize(n, 0.0f);
        cmd_mit_kp_.resize(n, 0.0f);
        cmd_mit_kd_.resize(n, 0.0f);
        cmd_mit_q_.resize(n, 0.0f);
        cmd_mit_dq_.resize(n, 0.0f);
        cmd_mit_tau_.resize(n, 0.0f);

        // -- Publishers --
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // -- Subscribers --
        if (control_mode_ == "velocity") {
            vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "cmd_vel_motors", 10,
                std::bind(&DamiaoNode::vel_callback, this, std::placeholders::_1));
        } else if (control_mode_ == "pos_vel") {
            // Expects pairs: [pos0, vel0, pos1, vel1, ...]
            pos_vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "cmd_pos_vel", 10,
                std::bind(&DamiaoNode::pos_vel_callback, this, std::placeholders::_1));
        } else if (control_mode_ == "mit") {
            // Expects quintuples: [kp0, kd0, q0, dq0, tau0, kp1, ...]
            mit_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "cmd_mit", 10,
                std::bind(&DamiaoNode::mit_callback, this, std::placeholders::_1));
        }

        // -- Timer (control loop) --
        auto period = std::chrono::duration<double>(1.0 / loop_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DamiaoNode::timer_callback, this));
    }

    ~DamiaoNode() override
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down — disabling motors...");
        for (auto& m : motors_) {
            try { mc_->disable(m); } catch (...) {}
        }
    }

private:
    // -- Parse "name:type:slave_id:master_id,..." --
    void parse_motors(const std::string& s)
    {
        std::istringstream stream(s);
        std::string token;
        while (std::getline(stream, token, ',')) {
            // trim whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (token.empty()) continue;

            // Split by ':'
            std::vector<std::string> parts;
            std::istringstream ts(token);
            std::string part;
            while (std::getline(ts, part, ':')) parts.push_back(part);

            if (parts.size() != 4) {
                RCLCPP_ERROR(this->get_logger(),
                    "Invalid motor spec '%s'. Expected name:type:slave_id:master_id", token.c_str());
                continue;
            }

            MotorDesc desc;
            desc.name = parts[0];
            desc.type = motor_type_from_string(parts[1]);
            desc.slave_id = static_cast<uint32_t>(std::stoul(parts[2], nullptr, 0));
            desc.master_id = static_cast<uint32_t>(std::stoul(parts[3], nullptr, 0));

            motor_descs_.push_back(desc);
            motor_names_.push_back(desc.name);
            motors_.emplace_back(desc.type, desc.slave_id, desc.master_id);
            mc_->addMotor(&motors_.back());

            RCLCPP_INFO(this->get_logger(), "Added motor '%s' (%s) slave=0x%02X master=0x%02X",
                        desc.name.c_str(), parts[1].c_str(), desc.slave_id, desc.master_id);
        }
    }

    // -- Timer: resend last command every tick and publish JointState --
    // Damiao motors have a communication timeout — they stop if they don't
    // receive continuous control commands. So we resend the last values each tick.
    void timer_callback()
    {
        for (size_t i = 0; i < motors_.size(); i++) {
            if (control_mode_ == "velocity") {
                mc_->control_vel(motors_[i], cmd_vel_[i]);
            } else if (control_mode_ == "pos_vel") {
                mc_->control_pos_vel(motors_[i], cmd_pos_[i], cmd_vel_[i]);
            } else if (control_mode_ == "mit") {
                mc_->control_mit(motors_[i],
                    cmd_mit_kp_[i], cmd_mit_kd_[i],
                    cmd_mit_q_[i], cmd_mit_dq_[i], cmd_mit_tau_[i]);
            } else {
                mc_->refresh_motor_status(motors_[i]);
            }
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = motor_names_;
        msg.position.resize(motors_.size());
        msg.velocity.resize(motors_.size());
        msg.effort.resize(motors_.size());

        for (size_t i = 0; i < motors_.size(); i++) {
            msg.position[i] = motors_[i].Get_Position();
            msg.velocity[i] = motors_[i].Get_Velocity();
            msg.effort[i]   = motors_[i].Get_tau();
        }

        joint_state_pub_->publish(msg);
    }

    // -- Callbacks just store the commanded values; timer resends them --
    void vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        for (size_t i = 0; i < std::min(msg->data.size(), motors_.size()); i++) {
            cmd_vel_[i] = static_cast<float>(msg->data[i]);
        }
    }

    void pos_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        for (size_t i = 0; i < motors_.size() && (i * 2 + 1) < msg->data.size(); i++) {
            cmd_pos_[i] = static_cast<float>(msg->data[i * 2]);
            cmd_vel_[i] = static_cast<float>(msg->data[i * 2 + 1]);
        }
    }

    void mit_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        for (size_t i = 0; i < motors_.size() && (i * 5 + 4) < msg->data.size(); i++) {
            size_t base = i * 5;
            cmd_mit_kp_[i]  = static_cast<float>(msg->data[base]);
            cmd_mit_kd_[i]  = static_cast<float>(msg->data[base + 1]);
            cmd_mit_q_[i]   = static_cast<float>(msg->data[base + 2]);
            cmd_mit_dq_[i]  = static_cast<float>(msg->data[base + 3]);
            cmd_mit_tau_[i] = static_cast<float>(msg->data[base + 4]);
        }
    }

    // -- Members --
    std::shared_ptr<SerialPort> serial_;
    std::unique_ptr<damiao::Motor_Control> mc_;
    std::vector<damiao::Motor> motors_;
    std::vector<MotorDesc> motor_descs_;
    std::vector<std::string> motor_names_;
    std::string control_mode_;

    // Last commanded values (resent every timer tick)
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
    rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DamiaoNode>());
    rclcpp::shutdown();
    return 0;
}
