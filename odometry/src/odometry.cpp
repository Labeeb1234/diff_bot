#include <cmath>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float64.hpp"

class OdometryPublisher : public rclcpp::Node
{
    public:
        
        double wheel_radius = 0.15;
        double wheel_separation = 0.45;
        double left_wheel_velocity = 0.0;
        double right_wheel_velocity = 0.0;
        // inital robot positions
        double x_pose = 0.0;
        double y_pose = 0.0;
        double th_pose = 0.0;
        // storage variables
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_velocity_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_velocity_pub; 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd_sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::TimerBase::SharedPtr timer_;

        OdometryPublisher() : Node("odometry_publisher")
        {

            // creating publishers
            left_wheel_velocity_pub = create_publisher<std_msgs::msg::Float64>("/left_wheel_velocity", 10);
            right_wheel_velocity_pub = create_publisher<std_msgs::msg::Float64>("/right_wheel_velocity", 10);
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
            transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            //creating subscribers
            velocity_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&OdometryPublisher::velCmdCallback, this, std::placeholders::_1));

            // creating timer
            timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdometryPublisher::publishOdometry, this));
            
        }
    private:
        void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            double u = msg->linear.x;
            double r = msg->angular.z;

            this->left_wheel_velocity = (1/this->wheel_radius)*(u - (this->wheel_separation/2)*r);
            this->right_wheel_velocity = (1/this->wheel_radius)*(u + (this->wheel_separation/2)*r);
            
            //publishing the wheel velocity data
            auto left_wheel_velocity_msg = std::make_shared<std_msgs::msg::Float64>();
            auto right_wheel_velocity_msg = std::make_shared<std_msgs::msg::Float64>();
            left_wheel_velocity_msg->data = this->left_wheel_velocity;
            left_wheel_velocity_pub->publish(*left_wheel_velocity_msg);
            right_wheel_velocity_msg->data = this->right_wheel_velocity;
            right_wheel_velocity_pub->publish(*right_wheel_velocity_msg);

        }

        void publishOdometry()
        {
            auto current_sim_time = get_clock()->now();

            double body_u;
            double body_r;

            body_u = (this->wheel_radius/2)*(this->left_wheel_velocity + this->right_wheel_velocity);
            body_r = (this->wheel_radius)/(this->wheel_separation)*(this->right_wheel_velocity - this->left_wheel_velocity);

            // Euler integration to update robot pose
            this->x_pose = this->x_pose + 0.1*(body_u*std::cos(this->th_pose));
            this->y_pose = this->y_pose + 0.1*(body_u*std::sin(this->th_pose));
            this->th_pose = this->th_pose + 0.1*body_r;

            // populate odom msgs using nav_msgs pkgs functions
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            odom_msg->header.stamp = current_sim_time;
            odom_msg->header.frame_id = "odom_frame";
            odom_msg->child_frame_id = "base_link";

            //setting robot pose to the odom_msgs
            odom_msg->pose.pose.position.x = this->x_pose;
            odom_msg->pose.pose.position.y = this->y_pose;
            odom_msg->pose.pose.position.z = 0.0;

            //publishing twist msgs through odom
            odom_msg->twist.twist.linear.x = body_u;
            odom_msg->twist.twist.linear.z = body_r;

            //setting robot orientation to the odom_msgs
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, this->th_pose);
            odom_msg->pose.pose.orientation = tf2::toMsg(quaternion);

            // publishing odom data
            odom_pub_->publish(*odom_msg);

            // tf2 broadcaster
            
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = current_sim_time;
            transform_stamped.header.frame_id = "odom_frame";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform.translation.x = this->x_pose;
            transform_stamped.transform.translation.y = this->y_pose;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation = tf2::toMsg(quaternion);

            transform_broadcaster_->sendTransform(transform_stamped);

        }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}


