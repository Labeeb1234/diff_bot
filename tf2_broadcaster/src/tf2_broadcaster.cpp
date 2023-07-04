#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"


class TransformBroadcasterNode : public rclcpp::Node
{
public:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    TransformBroadcasterNode() : Node("transform_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_bot_controller/odom", rclcpp::SystemDefaultsQoS(), std::bind(&TransformBroadcasterNode::odomCallback, this, std::placeholders::_1));
    }

    private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = odom_msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = odom_msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}