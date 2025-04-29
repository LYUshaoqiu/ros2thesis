// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>

// class MmwaveRadarNode : public rclcpp::Node {
// public:
//     MmwaveRadarNode() : Node("mmwave_radar_node") {
//         publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mmwave_radar/point_cloud", 10);
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(200), 
//             std::bind(&MmwaveRadarNode::publish_data, this)
//         );
//     }

// private:
//     void publish_data() {
//         auto message = sensor_msgs::msg::PointCloud2();
//         // Fill message with dummy data
//         publisher_->publish(message);
//     }

//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MmwaveRadarNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class MmwaveRadarNode : public rclcpp::Node {
public:
    MmwaveRadarNode() : Node("mmwave_radar_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mmwave_radar/point_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&MmwaveRadarNode::publish_data, this)
        );
    }

private:
    void publish_data() {
        // 设置点云消息的基本属性
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "map";  // 设定坐标系

        // 点云的基本设置
        int num_points = 200;  // 模拟的点数
        message.height = 3;
        message.width = num_points;
        message.is_dense = false;
        message.is_bigendian = false;

        // 设置点云字段 (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(num_points);

        // 使用迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");

        for (int i = 0; i < num_points; ++i) {
            *iter_x = static_cast<float>(rand() % 10) / 10.0f;  // 随机 x 值
            *iter_y = static_cast<float>(rand() % 10) / 10.0f;  // 随机 y 值
            *iter_z = static_cast<float>(rand() % 10) / 10.0f;  // 随机 z 值

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        // 发布点云数据
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing simulated point cloud data");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MmwaveRadarNode>());
    rclcpp::shutdown();
    return 0;
}
