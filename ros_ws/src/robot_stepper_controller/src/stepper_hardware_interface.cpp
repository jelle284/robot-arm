#include "stepper_hardware_interface.hpp"

hardware_interface::CallbackReturn stepper_hardware_interface::StepperHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn stepper_hardware_interface::StepperHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    // TODO:
    // * Declare parameter for starting position
    // * Try and load it from a yaml file
    // * Initialize state and command vectors
    // * Set initial position 0
    // * Set all commands to 0
    // * Intialize publisher and subscriber
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn stepper_hardware_interface::StepperHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    // TODO:
    // * Save new starting position for next time to a yaml file
    return hardware_interface::CallbackReturn();
}

hardware_interface::return_type stepper_hardware_interface::StepperHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // TODO:
    // * Read steps and step velocity from topic
    // * Figure out how to run it through transmission interface
    // * Add starting position to obtain current position
    // * Update state vector
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type stepper_hardware_interface::StepperHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // TODO:
    // * Run through transmission interface
    // * Publish step velocity to topic
    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stepper_hardware_interface::StepperHardwareInterface, hardware_interface::SystemInterface)