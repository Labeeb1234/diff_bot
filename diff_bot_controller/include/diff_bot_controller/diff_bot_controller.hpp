#ifndef DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h" 
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_state_;
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };
    
    std::vector<WheelHandle> registered_wheel_handles_;
    std::vector<std::string> wheel_joint_names_;
    int num_wheels_;

    bool subscriber_is_active_ = false;
    //rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;

    //std::queue<Twist> previous_commands_; 

    double wheel_radius_;
    double wheel_separation_;
    
    // storing the timestamp of previous published msg
    rclcpp::Time previous_updated_timestamp_{0};


    double publish_rate_ = 50.0;
    rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
    rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
    double cmd_vel_timeout_ = 0.5; // in [s]


    // custom reset function
    bool is_reset = false;
    bool reset();


};
}

#endif
