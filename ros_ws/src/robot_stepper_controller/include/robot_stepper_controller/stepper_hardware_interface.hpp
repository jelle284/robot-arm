#ifndef STEPPER_HARDWARE_INTERFACE_HPP
#define STEPPER_HARDWARE_INTERFACE_HPP

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/macros.hpp"

#include "stepper_msgs/msg/stepper_command.hpp"
#include "stepper_msgs/msg/stepper_state.hpp"

namespace robot_stepper_controller {
        class StepperHardwareInterface : public hardware_interface::SystemInterface {
        public:
            StepperHardwareInterface() {}
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<stepper_msgs::msg::StepperCommand>::SharedPtr command_pub_;
            rclcpp::Publisher<stepper_msgs::msg::StepperCommand>::SharedPtr initial_position_pub_;
            rclcpp::Subscription<stepper_msgs::msg::StepperState>::SharedPtr state_sub_;

            std::vector<int32_t> joints_to_steps(const std::vector<double>& joints);
            std::vector<double> steps_to_joints(const std::vector<int32_t>& steps);

    };
}

#endif // STEPPER_HARDWARE_INTERFACE_HPP