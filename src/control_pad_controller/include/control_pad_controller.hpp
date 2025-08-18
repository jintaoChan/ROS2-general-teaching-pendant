#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"

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
        virtual void switchMode(int8_t mode);
    protected:
        std::shared_ptr<rclcpp::Logger> logger_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        uint8_t default_mode_;
        uint8_t position_mode_;
        uint8_t velocity_mode_;
        uint8_t current_mode_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr move_command_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr move_state_publisher_;
        bool new_move_msg_ = false;
        sensor_msgs::msg::JointState move_msg_;

        rclcpp::Time m_StartTime;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_mode_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_mode_state_interface_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            m_CommandInterfaceMap = {
                {"position", &joint_position_command_interface_},
                {"velocity", &joint_velocity_command_interface_},
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

