#pragma once

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
        rclcpp::Logger get_logger() const { return *m_Logger; }
    protected:
        // Objects for logging
        std::shared_ptr<rclcpp::Logger> m_Logger;
        std::vector<std::string> m_JointNames;
        std::vector<std::string> m_CommandInterfaceTypes;
        std::vector<std::string> m_StateInterfaceTypes;

        // movement msg
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_MoveCommandSubscriber;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_MoveStatePublisher;
        bool m_NewMoveMsg = false;
        sensor_msgs::msg::JointState m_MoveMsg;

        // mode msg
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_ModeCommandSubscriber;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_ModeStatePublisher;
        bool m_NewModeMsg = false;
        rclcpp::Time m_StartTime;
        std_msgs::msg::Int8 m_ModeMsg;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_JointPositionCommandInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> m_JointPositionStateInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_JointVelocityCommandInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> m_JointVelocityStateInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_JointModeCommandInterface;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> m_JointModeStateInterface;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            m_CommandInterfaceMap = {
                {"position", &m_JointPositionCommandInterface},
                {"velocity", &m_JointVelocityCommandInterface},
                {"mode", &m_JointModeCommandInterface}
            };

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            m_StateInterfaceMap = {
                {"position", &m_JointPositionStateInterface},
                {"velocity", &m_JointVelocityStateInterface},
                {"mode", &m_JointModeStateInterface}
            };
    };

}

