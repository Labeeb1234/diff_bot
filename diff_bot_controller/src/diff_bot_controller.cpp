#include <memory> 
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "diff_bot_controller/diff_bot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace
{
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/cmd_vel"; 
}  


namespace diff_bot_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    DiffBotController::DiffBotController() 
        : controller_interface::ControllerInterface()
        , velocity_command_subscriber_(nullptr)
        , velocity_command_ptr_(nullptr){}

    controller_interface::CallbackReturn DiffBotController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration DiffBotController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for(const auto &joint_name: wheel_joint_names_)
        {
            conf_names.push_back(joint_name+ "/" + HW_IF_VELOCITY);

        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

    }
    
    InterfaceConfiguration DiffBotController::state_interface_configuration() const 
    {
        std::vector<std::string> conf_names;

        for (const auto & joint_name : wheel_joint_names_) 
        {
        
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }

        for(const auto & joint_name: wheel_joint_names_)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }
    
    controller_interface::CallbackReturn DiffBotController::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configuring DiffBotController.....\n");

        num_wheels_ = get_node()->get_parameter("num_wheels").as_int();
        RCLCPP_INFO(get_node()->get_logger(), "setting number of wheels as: [%d]", num_wheels_);

        wheel_joint_names_ = get_node()->get_parameter("wheel_joint_names").as_string_array();
        if(wheel_joint_names_.size() != num_wheels_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "number wheels in params not matching the number of wheels on the model\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        if(wheel_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "no wheels!\n");
            return controller_interface::CallbackReturn::ERROR;
        }
        
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();

        is_reset = get_node()->get_parameter("is_reset").as_bool();
        publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0/publish_rate_);
        previous_publish_timestamp_ = get_node()->get_clock()->now();


        // creating a subcriber to subcriber to Twist(unstamped) topic
        velocity_command_subscriber_ = get_node()->create_subscription<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS(),[this](Twist::SharedPtr msg)
            {
                if(!subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),"Can't accept new msgs subscriber is not active\n");
                    return;
                }
                
                velocity_command_ptr_.writeFromNonRT(msg);                
            });

        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn DiffBotController::on_activate(const rclcpp_lifecycle::State &)
    {

        if(wheel_joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "wheel joint name are empty\n");
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_wheel_handles_.reserve(wheel_joint_names_.size());
        {
            for(const auto &wheel_joint_name: wheel_joint_names_)
            {
                const auto pos_interface_name = HW_IF_POSITION;
                const auto pos_state = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name, &pos_interface_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == pos_interface_name;
                    });
                if(pos_state == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get position state handle\n");
                    return controller_interface::CallbackReturn::ERROR;
                }
                const auto interface_name = HW_IF_VELOCITY;
                const auto state_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name, &interface_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == interface_name;

                    });

                if(state_handle == state_interfaces_.cend())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint state handle for %s", wheel_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                const auto command_handle = std::find_if(
                    command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const auto &interface)
                    {
                        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                    });
                
                if(command_handle == command_interfaces_.end())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "Unable to get joint command handle for %s", wheel_joint_name.c_str());
                    return controller_interface::CallbackReturn::ERROR;
                }

                registered_wheel_handles_.emplace_back(WheelHandle{std::ref(*pos_state),std::ref(*state_handle), std::ref(*command_handle)});

                RCLCPP_INFO(get_node()->get_logger(),"Got command interface: %s", command_handle->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(), "Got positon state interface: %s", pos_state->get_name().c_str());
                RCLCPP_INFO(get_node()->get_logger(),"Got velocity state interface: %s", state_handle->get_name().c_str());
       
            }

            subscriber_is_active_ = true;
            RCLCPP_INFO(get_node()->get_logger(),"Controller Is Activated!\n");
            previous_updated_timestamp_ = get_node()->get_clock()->now();
            return controller_interface::CallbackReturn::SUCCESS;
        }

    }

    controller_interface::CallbackReturn DiffBotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(), "Called on_deactivate\n");
        subscriber_is_active_ = false;
        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn  DiffBotController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_cleanup\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // if errors occur what to do ?
    controller_interface::CallbackReturn  DiffBotController::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_error\n");
        return controller_interface::CallbackReturn::SUCCESS; 
    }

    // shutdown phase of the controllers
    controller_interface::CallbackReturn  DiffBotController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_DEBUG(get_node()->get_logger(),"Called on_shutdown\n");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DiffBotController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {   

        if(is_reset == true)
        {
            is_reset = true;
            DiffBotController::reset();
            RCLCPP_INFO(get_node()->get_logger(), "The data has been cleared!!\n");
            return controller_interface::return_type::OK;
        }

        
        
        // to get previous velocity command 
        auto velocity_command = velocity_command_ptr_.readFromRT();
        if (!velocity_command || !(*velocity_command)) 
        {
            return controller_interface::return_type::OK;
        }

        const auto msg_linear = (*velocity_command)->linear;
        const auto msg_angular = (*velocity_command)->angular;
        double v_x_des = msg_linear.x;
        double omega_des = msg_angular.z;

        std::vector<double> wheel_velocity(num_wheels_);
        wheel_velocity[0] = (2*v_x_des - omega_des*wheel_separation_)/(2*wheel_radius_);
        wheel_velocity[1] = (2*v_x_des + omega_des*wheel_separation_)/(2*wheel_radius_);
        wheel_velocity[2] = (2*v_x_des - omega_des*wheel_separation_)/(2*wheel_radius_);
        wheel_velocity[3] = (2*v_x_des + omega_des*wheel_separation_)/(2*wheel_radius_);

        registered_wheel_handles_[0].velocity_command.get().set_value(wheel_velocity[0]);
        registered_wheel_handles_[1].velocity_command.get().set_value(wheel_velocity[1]);
        registered_wheel_handles_[2].velocity_command.get().set_value(wheel_velocity[2]);
        registered_wheel_handles_[3].velocity_command.get().set_value(wheel_velocity[3]);
        
        return controller_interface::return_type::OK;
    }


    bool DiffBotController::reset()
    {
        if(is_reset==true)
        {
            this->velocity_command_ptr_.reset();
            this->velocity_command_subscriber_.reset();   
        }
        return 0;
    }

    DiffBotController::~DiffBotController() {}

}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  diff_bot_controller::DiffBotController, controller_interface::ControllerInterface)



