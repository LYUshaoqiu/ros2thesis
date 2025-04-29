#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>  // 使用Image消息类型

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/kinect/depth_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&KinectNode::publish_data, this)
        );
    }

private:
    void publish_data() {
        auto message = sensor_msgs::msg::Image();
        
        // 配置模拟的深度图像数据
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "camera_frame";
        message.height = 480;                     // 图像高度
        message.width = 640;                      // 图像宽度
        message.encoding = "mono8";               // 单通道8位灰度图像
        message.step = message.width;             // 每行的字节数
        message.data.resize(message.step * message.height, 128);  // 填充图像数据为中灰色
        
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
    return 0;
}
