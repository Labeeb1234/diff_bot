#ifndef DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diff_drive_controller/odometry.hpp"
#include "diff_drive_controller/speed_limiter.hpp"
#include "diff_drive_controller/visibility_control.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace diff_bot_controller
{
class DiffBotController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::Twist;

public:
   
  DiffBotController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
    ~DiffBotController();

protected:
    struct WheelHandle 
    {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };
    
    std::vector<WheelHandle> registered_wheel_handles_;
    std::vector<std::string> wheel_joint_names_;

    bool subscriber_is_active_ = false;
    //rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;

    //std::queue<Twist> previous_commands_; 

    double wheel_radius_;
    double wheel_separation_;
    
    //rclcpp::Time previous_update_timestamp_{0};

    double publish_rate_ = 50.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

    bool use_stamped_vel;
};
}

#endif