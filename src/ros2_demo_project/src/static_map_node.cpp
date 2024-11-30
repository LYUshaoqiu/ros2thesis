#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticMapNode : public rclcpp::Node {
public:
    StaticMapNode() : Node("static_map_node") {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_map_transform();
    }

private:
    void publish_map_transform() {
        geometry_msgs::msg::TransformStamped transformStamped;
        
        // 设置父坐标系（map）到子坐标系（base_link）的静态变换
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        
        // 设定静态变换的位置（这里设为原点）
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        
        // 设定静态变换的旋转（四元数，表示无旋转）
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        // 广播静态变换
        static_broadcaster_->sendTransform(transformStamped);
        RCLCPP_INFO(this->get_logger(), "Static transform from 'map' to 'base_link' published");
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticMapNode>());
    rclcpp::shutdown();
    return 0;
}
