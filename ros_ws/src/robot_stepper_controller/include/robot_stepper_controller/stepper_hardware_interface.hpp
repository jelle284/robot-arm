#ifndef STEPPER_HARDWARE_INTERFACE_HPP
#define STEPPER_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace stepper_hardware_interface {
        class StepperHardwareInterface : public hardware_interface::SystemInterface {
        public:
            StepperHardwareInterface();
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };
}

#endif // STEPPER_HARDWARE_INTERFACE_HPP