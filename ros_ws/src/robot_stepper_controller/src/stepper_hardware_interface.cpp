#include "robot_stepper_controller/stepper_hardware_interface.hpp"
#include <hardware_interface/introspection.hpp>
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::vector<double> gear_ratio = {140.0, 56.0, 56.0, 125.78125, 19.047619047619047, 19.047619047619047}; // TODO: Get from joint configuration
static const std::vector<double> home_position = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0}; // TODO: Get from joint configuration
static const double steps_per_rev = 800.0; // TODO: Get from joint configuration

hardware_interface::CallbackReturn robot_stepper_controller::StepperHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn robot_stepper_controller::StepperHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    // Initialize states and commands
    for (size_t i = 0; i < 6; ++i) {
        const std::string joint_name = "joint_" + std::to_string(i);
        set_state(joint_name + "/position", home_position[i]);
        set_state(joint_name + "/velocity", 0.0);
        set_command(joint_name + "/position", home_position[i]);
    }

    // Intialize publisher and subscriber
    node_ = rclcpp::Node::make_shared("stepper_hardware_interface_node");
    auto sensor_qos = rclcpp::SensorDataQoS();
    command_pub_ = node_->create_publisher<stepper_msgs::msg::StepperCommand>("stepper_command", sensor_qos);
    state_sub_ = node_->create_subscription<stepper_msgs::msg::StepperState>("stepper_state", sensor_qos, [this](stepper_msgs::msg::StepperState::SharedPtr msg){
        auto joint_positions = steps_to_joints(msg->position);
        auto joint_velocities = steps_to_joints(msg->velocity);
        for (size_t i = 0; i < 6; i++)
        {   
            double pos = joint_positions[i] + home_position[i];
            set_state("joint_" + std::to_string(i) + "/position", pos);
            set_state("joint_" + std::to_string(i) + "/velocity", joint_velocities[i]);
        }
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
    // Reading is done in the subscription callback
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type robot_stepper_controller::StepperHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    std::vector<double> joint_commands;
    for (size_t i = 0; i < 6; i++)
    {
        double cmd = get_command("joint_" + std::to_string(i) + "/position") - home_position[i];
        joint_commands.push_back(cmd);
    }
    command_pub_->publish(stepper_msgs::msg::StepperCommand().set__position(joints_to_steps(joint_commands)));
    return hardware_interface::return_type::OK;
}

std::vector<int32_t> robot_stepper_controller::StepperHardwareInterface::joints_to_steps(const std::vector<double> &joints)
{
    std::vector<int32_t> steps;
    for (size_t i = 0; i < 4; i++)
    {
        steps.push_back(static_cast<int32_t>((joints[i] / (2.0 * M_PI)) * steps_per_rev * gear_ratio[i]));
    }
    // Differential drive joints
    double g1_position = (joints[4] + joints[5]);
    double g2_position = (joints[4] - joints[5]);
    steps.push_back(static_cast<int32_t>((g1_position / (2.0 * M_PI)) * steps_per_rev * gear_ratio[4]));
    steps.push_back(static_cast<int32_t>((g2_position / (2.0 * M_PI)) * steps_per_rev * gear_ratio[5]));
    return steps;
}

std::vector<double> robot_stepper_controller::StepperHardwareInterface::steps_to_joints(const std::vector<int32_t> &steps)
{
    std::vector<double> joints;
    // Convert from steps to radians
    for (size_t i = 0; i < 4; i++)
    {   
        joints.push_back((steps[i] / (steps_per_rev * gear_ratio[i])) * 2.0 * M_PI);
    }
    // Differential drive joints
    double g1_position = (steps[4] / (steps_per_rev * gear_ratio[4])) * 2.0 * M_PI;
    double g2_position = (steps[5] / (steps_per_rev * gear_ratio[5])) * 2.0 * M_PI;
    joints.push_back((g1_position + g2_position) / 2.0);
    joints.push_back((g1_position - g2_position) / 2.0);
    return joints;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_stepper_controller::StepperHardwareInterface, hardware_interface::SystemInterface)