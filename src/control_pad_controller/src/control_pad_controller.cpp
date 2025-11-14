
#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "control_pad_controller.hpp"


using config_type = controller_interface::interface_configuration_type;

namespace control_pad
{
    ControlPadController::ControlPadController() : controller_interface::ControllerInterface(){}

    controller_interface::CallbackReturn ControlPadController::on_init()
    {
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("ControlPadController"));
        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        default_mode_ = auto_declare<uint8_t>("mode.default", default_mode_);
        position_mode_ = auto_declare<uint8_t>("mode.position", position_mode_);
        velocity_mode_ = auto_declare<uint8_t>("mode.velocity", velocity_mode_);
        current_mode_ = default_mode_;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ControlPadController::command_interface_configuration()
        const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration ControlPadController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::CallbackReturn ControlPadController::on_configure(const rclcpp_lifecycle::State &)
    {
        auto moveCallback =
            [this](const std::shared_ptr<std_msgs::msg::Int8> msg) -> void
        {
            if(current_mode_ != msg->data) {
                new_mode = true;
            }
            current_mode_ = msg->data;
        };

        
        move_cmd_publisher_ = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_controller/joint_trajectory", 10);
        mode_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int8>("mode_controller/command", rclcpp::SystemDefaultsQoS(), moveCallback);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ControlPadController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_mode_state_interface_.clear();
        joint_velocity_cmd_interface_.clear();
        joint_mode_command_interface_.clear();
        // assign command interfaces
        for (auto &interface : command_interfaces_)
        {
            m_CommandInterfaceMap[interface.get_interface_name()]->push_back(interface);
        }

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            m_StateInterfaceMap[interface.get_interface_name()]->push_back(interface);
        }
        return CallbackReturn::SUCCESS;
    }


    controller_interface::return_type ControlPadController::update(
        const rclcpp::Time &time, const rclcpp::Duration & period)
    {
        trajectory_msgs::msg::JointTrajectory reset;
        trajectory_msgs::msg::JointTrajectoryPoint p;
        // Publish joint state
        for(const auto& state : joint_position_state_interface_){
            reset.joint_names.push_back(state.get().get_prefix_name());
            p.positions.push_back(state.get().get_optional().value());
            // RCLCPP_INFO(get_logger(), "Getting current position\t%s: %f", std::string(state.get().get_name()).c_str(), joint_state.position.back());
        }
        reset.header.stamp = get_node()->get_clock()->now();
        p.time_from_start = period;
        reset.points.push_back(p);
        if (new_mode)
        {
            new_mode = false;
            move_cmd_publisher_->publish(reset);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn ControlPadController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    control_pad::ControlPadController, controller_interface::ControllerInterface)
