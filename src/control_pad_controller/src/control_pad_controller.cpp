
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
    ControlPadController::ControlPadController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn ControlPadController::on_init()
    {
        m_Logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("ControlPadController"));
        // should have error handling
        m_JointNames = auto_declare<std::vector<std::string>>("joints", m_JointNames);
        m_CommandInterfaceTypes =
            auto_declare<std::vector<std::string>>("command_interfaces", m_CommandInterfaceTypes);
        m_StateInterfaceTypes =
            auto_declare<std::vector<std::string>>("state_interfaces", m_StateInterfaceTypes);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ControlPadController::command_interface_configuration()
        const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(m_JointNames.size() * m_CommandInterfaceTypes.size());
        for (const auto &joint_name : m_JointNames)
        {
            for (const auto &interface_type : m_CommandInterfaceTypes)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration ControlPadController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(m_JointNames.size() * m_StateInterfaceTypes.size());
        for (const auto &joint_name : m_JointNames)
        {
            for (const auto &interface_type : m_StateInterfaceTypes)
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
            m_MoveMsg = *msg;
            m_NewMoveMsg = true;
        };
        
        m_MoveStatePublisher = get_node()->create_publisher<sensor_msgs::msg::JointState>("control_pad_move_state", 10);
        m_MoveCommandSubscriber = get_node()->create_subscription<sensor_msgs::msg::JointState>("control_pad_move_cmd", rclcpp::SystemDefaultsQoS(), moveCallback);

        auto modeCallback =
            [this](const std::shared_ptr<std_msgs::msg::Int8> msg) -> void
        {
            m_ModeMsg = *msg;
            m_NewModeMsg = true;
        };

        m_ModeStatePublisher = get_node()->create_publisher<std_msgs::msg::Int8>("control_pad_mode_state", 10);
        m_ModeCommandSubscriber = get_node()->create_subscription<std_msgs::msg::Int8>("control_pad_mode_cmd", rclcpp::SystemDefaultsQoS(), modeCallback);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ControlPadController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        m_JointPositionCommandInterface.clear();
        m_JointPositionStateInterface.clear();
        m_JointVelocityCommandInterface.clear();
        m_JointVelocityStateInterface.clear();
        m_JointModeCommandInterface.clear();
        m_JointModeStateInterface.clear();
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
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {

        // Publish joint state
        {
            sensor_msgs::msg::JointState jointState;
            for(const auto& state : m_JointPositionStateInterface){
                jointState.position.push_back(state.get().get_optional().value());
                // RCLCPP_INFO(get_logger(), "Getting current position\t%s: %f", std::string(state.get().get_name()).c_str(), jointState.position.back());
            }
            for(const auto& state : m_JointVelocityStateInterface){
                jointState.velocity.push_back(state.get().get_optional().value());
                // RCLCPP_INFO(get_logger(), "Getting current velocity\t%s: %f", std::string(state.get().get_name()).c_str(), jointState.velocity.back());
            }
            m_MoveStatePublisher->publish(jointState);
        }

        {
            std_msgs::msg::Int8 modeStates;
            std::vector<double> modes;
            for(const auto& state : m_JointModeStateInterface){
                modes.push_back(state.get().get_value());
                // RCLCPP_INFO(get_logger(), "Getting current position\t%s: %f", std::string(state.get().get_name()).c_str(), jointState.position.back());
            }
            auto first = modes.front();
            if (std::all_of(modes.begin(), modes.end(), [first](auto val) { return val == first; })) {
                modeStates.data = first;
                m_ModeStatePublisher->publish(modeStates);
            }
            else {
                RCLCPP_ERROR(get_logger(), "Joint Modes differ from each other!");
            }
        }



        if (m_NewModeMsg)
        {
            m_StartTime = time;
            m_NewModeMsg = false;
            switch(m_ModeMsg.data){
                case 0:
                case 8:
                    for(size_t i = 0; i < m_JointPositionCommandInterface.size(); ++i) {
                        m_JointPositionCommandInterface[i].get().set_value(m_JointPositionStateInterface[i].get().get_optional().value());
                    }
                case 9:
                    for(size_t i = 0; i < m_JointVelocityCommandInterface.size(); ++i) {
                        m_JointVelocityCommandInterface[i].get().set_value(0.0);
                    }
                    for(size_t i = 0; i < m_JointModeCommandInterface.size(); ++i) {
                        m_JointModeCommandInterface[i].get().set_value((double)m_ModeMsg.data);
                    }
                    break;
                default:
                    RCLCPP_INFO(get_logger(), "Setting a illegal mode! Abort!");
            }
        }

        if (m_NewMoveMsg)
        {
            m_StartTime = time;
            m_NewMoveMsg = false;
            if(m_MoveMsg.name.size() == m_MoveMsg.position.size() && !m_JointPositionCommandInterface.empty()) {
                for(size_t i = 0 ; i < m_MoveMsg.name.size() ; ++i) {
                    auto ifIdx = std::find(m_JointNames.begin(), m_JointNames.end(), m_MoveMsg.name[i]);
                    if(ifIdx != m_JointNames.end()){
                        m_JointPositionCommandInterface[ifIdx - m_JointNames.begin()].get().set_value(m_MoveMsg.position[i]);
                    }
                }
            }
            else if (m_MoveMsg.name.size() == m_MoveMsg.velocity.size() && !m_JointVelocityCommandInterface.empty()){
                for(size_t i = 0 ; i < m_MoveMsg.name.size() ; ++i) {
                    auto ifIdx = std::find(m_JointNames.begin(), m_JointNames.end(), m_MoveMsg.name[i]);
                    if(ifIdx != m_JointNames.end()){
                        m_JointVelocityCommandInterface[ifIdx - m_JointNames.begin()].get().set_value(m_MoveMsg.velocity[i]);
                    }
                }
            }
            else {
                RCLCPP_WARN(get_logger(), "Invalid message! name.size(): %d   position.size(): %d   velocity.size(): %d", m_MoveMsg.name.size(), m_MoveMsg.position.size(), m_MoveMsg.velocity.size());
            }
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
