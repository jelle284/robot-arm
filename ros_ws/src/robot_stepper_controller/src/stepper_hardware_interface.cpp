#include "robot_stepper_controller/stepper_hardware_interface.hpp"
#include <hardware_interface/introspection.hpp>
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::vector<double> gear_ratio = {140.0, 56.0, 56.0, 125.78125, 19.047619047619047, 19.047619047619047}; // TODO: Get from joint configuration
static const std::vector<double> home_position = {0.0, 0.0, 0.20, 0.0, 0.0, 0.0}; // TODO: Get from joint configuration
static const double steps_per_rev = 800.0; // TODO: Get from joint configuration

hardware_interface::CallbackReturn robot_stepper_controller::StepperHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    state_position_.resize(info_.joints.size(), 0);
    state_velocity_.resize(info_.joints.size(), 0);
    command_velocity_.resize(info_.joints.size(), 0);
    REGISTER_ROS2_CONTROL_INTROSPECTION("my_custom_introspection", &joint_introspection_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn robot_stepper_controller::StepperHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    for (auto & state : state_position_)
        state = 0;
    for (auto & state : state_velocity_)
        state = 0;
    for (auto & command : command_velocity_)
        command = 0;

    // Intialize publisher and subscriber
    node_ = rclcpp::Node::make_shared("stepper_hardware_interface_node");
    auto sensor_qos = rclcpp::SensorDataQoS();
    command_pub_ = node_->create_publisher<stepper_msgs::msg::StepperCommand>("stepper_command", sensor_qos);
    state_sub_ = node_->create_subscription<stepper_msgs::msg::StepperState>("stepper_state", sensor_qos, [this](stepper_msgs::msg::StepperState::SharedPtr msg){
        state_position_ = msg->position;
        state_velocity_ = msg->velocity;
    });
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn robot_stepper_controller::StepperHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type robot_stepper_controller::StepperHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    if (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
    }
    // Convert from steps to radians
    for (size_t i = 0; i < 4; i++)
    {   
        double position = (state_position_[i] / (steps_per_rev * gear_ratio[i])) * 2.0 * M_PI;
        position += home_position[i];
        set_state("joint_" + std::to_string(i) + "/position", position);

        double velocity = (state_velocity_[i] / (steps_per_rev * gear_ratio[i])) * 2.0 * M_PI;
        set_state("joint_" + std::to_string(i) + "/velocity", velocity);
    }
    // Differential drive joints
    double g1_position = (state_position_[4] / (steps_per_rev * gear_ratio[4])) * 2.0 * M_PI;
    double g2_position = (state_position_[5] / (steps_per_rev * gear_ratio[5])) * 2.0 * M_PI;
    set_state("joint_4/position", g1_position + g2_position + home_position[4]);
    set_state("joint_5/position", g1_position - g2_position + home_position[5]);
    double g1_velocity = (state_velocity_[4] / (steps_per_rev * gear_ratio[4])) * 2.0 * M_PI;
    double g2_velocity = (state_velocity_[5] / (steps_per_rev * gear_ratio[5])) * 2.0 * M_PI;
    set_state("joint_4/velocity", g1_velocity + g2_velocity);
    set_state("joint_5/velocity", g1_velocity - g2_velocity);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type robot_stepper_controller::StepperHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    joint_introspection_ = get_command("joint_0/velocity");
    for (size_t i = 0; i < 4; i++)
    {
        double velocity = get_command("joint_" + std::to_string(i) + "/velocity");
        command_velocity_[i] = static_cast<int32_t>((velocity / (2.0 * M_PI)) * steps_per_rev * gear_ratio[i]);
    }
    // Differential drive joints
    double joint4_velocity = get_command("joint_4/velocity");
    double joint5_velocity = get_command("joint_5/velocity");
    double g1_velocity = (joint4_velocity + joint5_velocity);
    double g2_velocity = (joint4_velocity - joint5_velocity);
    command_velocity_[4] = static_cast<int32_t>((g1_velocity / (2.0 * M_PI)) * steps_per_rev * gear_ratio[4]);
    command_velocity_[5] = static_cast<int32_t>((g2_velocity / (2.0 * M_PI)) * steps_per_rev * gear_ratio[5]);
    command_pub_->publish(stepper_msgs::msg::StepperCommand().set__velocity(command_velocity_));
    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_stepper_controller::StepperHardwareInterface, hardware_interface::SystemInterface)