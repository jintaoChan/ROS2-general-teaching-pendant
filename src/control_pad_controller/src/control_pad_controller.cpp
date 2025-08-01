
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
            [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
        {
            move_msg_ = *msg;
            new_move_msg_ = true;
        };
        
        move_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("control_pad_controller/control_pad_move_state", 10);
        move_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>("control_pad_controller/control_pad_move_cmd", rclcpp::SystemDefaultsQoS(), moveCallback);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ControlPadController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_mode_command_interface_.clear();
        joint_mode_state_interface_.clear();
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
        switchMode(default_mode_);
        return CallbackReturn::SUCCESS;
    }


    controller_interface::return_type ControlPadController::update(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {

        // Publish joint state
        sensor_msgs::msg::JointState joint_state;
        for(const auto& state : joint_position_state_interface_){
            joint_state.position.push_back(state.get().get_optional().value());
            // RCLCPP_INFO(get_logger(), "Getting current position\t%s: %f", std::string(state.get().get_name()).c_str(), joint_state.position.back());
        }
        for(const auto& state : joint_velocity_state_interface_){
            joint_state.velocity.push_back(state.get().get_optional().value());
            // RCLCPP_INFO(get_logger(), "Getting current velocity\t%s: %f", std::string(state.get().get_name()).c_str(), joint_state.velocity.back());
        }
        move_state_publisher_->publish(joint_state);


        if (new_move_msg_)
        {
            m_StartTime = time;
            new_move_msg_ = false;
            if(move_msg_.name.size() == move_msg_.position.size() && !joint_position_command_interface_.empty()) {
                switchMode(position_mode_);
                for(size_t i = 0 ; i < move_msg_.name.size() ; ++i) {
                    auto ifIdx = std::find(joint_names_.begin(), joint_names_.end(), move_msg_.name[i]);
                    if(ifIdx != joint_names_.end()){
                        joint_position_command_interface_[ifIdx - joint_names_.begin()].get().set_value(move_msg_.position[i]);
                    }
                }
            }
            else if (move_msg_.name.size() == move_msg_.velocity.size() && !joint_velocity_command_interface_.empty()){
                switchMode(velocity_mode_);
                for(size_t i = 0 ; i < move_msg_.name.size() ; ++i) {
                    auto ifIdx = std::find(joint_names_.begin(), joint_names_.end(), move_msg_.name[i]);
                    if(ifIdx != joint_names_.end()){
                        joint_velocity_command_interface_[ifIdx - joint_names_.begin()].get().set_value(move_msg_.velocity[i]);
                    }
                }
            }
            else {
                RCLCPP_WARN(get_logger(), "Invalid message! name.size(): %d   position.size(): %d   velocity.size(): %d", move_msg_.name.size(), move_msg_.position.size(), move_msg_.velocity.size());
            }
        }


        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn ControlPadController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    void ControlPadController::switchMode(int8_t mode)
    {
        switch(mode){
            case 0:
            case 8:
                for(size_t i = 0; i < joint_position_command_interface_.size(); ++i) {
                    joint_position_command_interface_[i].get().set_value(joint_position_state_interface_[i].get().get_optional().value());
                }
            case 9:
                for(size_t i = 0; i < joint_velocity_command_interface_.size(); ++i) {
                    joint_velocity_command_interface_[i].get().set_value(0.0);
                }
                for(size_t i = 0; i < joint_mode_command_interface_.size(); ++i) {
                    joint_mode_command_interface_[i].get().set_value((double)mode);
                }
                break;
            default:
                RCLCPP_INFO(get_logger(), "Setting a illegal mode! Abort!");
        }
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    control_pad::ControlPadController, controller_interface::ControllerInterface)
