#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/int8.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace control_pad
{
    class ControlPadController : public controller_interface::ControllerInterface
    {
    public:
        ControlPadController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
        rclcpp::Logger get_logger() const { return *logger_; }
    protected:
        std::shared_ptr<rclcpp::Logger> logger_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        int8_t default_mode_;
        int8_t position_mode_;
        int8_t velocity_mode_;
        int8_t current_mode_;
        bool new_mode{true};
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mode_command_subscriber_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr move_cmd_publisher_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_mode_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_cmd_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_mode_command_interface_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            m_CommandInterfaceMap = {
                {"velocity", &joint_velocity_cmd_interface_},
                {"mode", &joint_mode_command_interface_}
            };

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            m_StateInterfaceMap = {
                {"position", &joint_position_state_interface_},
                {"velocity", &joint_velocity_state_interface_},
                {"mode", &joint_mode_state_interface_}
            };
    };

}

