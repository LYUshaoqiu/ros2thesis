#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class RoomMapNode : public rclcpp::Node {
public:
    RoomMapNode() : Node("room_map_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("room_map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RoomMapNode::publish_room_map, this)
        );
    }

private:
    void publish_room_map() {
        // 地板
        create_marker(0, "floor", 10.0, 10.0, 0.1, 0.0, 0.0, 0.05, 0.5, 0.5, 0.5, 1.0);

        // 天花板
        create_marker(1, "ceiling", 10.0, 10.0, 0.1, 0.0, 0.0, 2.95, 0.5, 0.5, 0.5, 0.3);

        // 墙壁1
        create_marker(2, "wall_1", 10.0, 0.1, 3.0, 0.0, -5.0, 1.5, 0.7, 0.7, 0.7, 0.8);

        // 墙壁2
        create_marker(3, "wall_2", 10.0, 0.1, 3.0, 0.0, 5.0, 1.5, 0.7, 0.7, 0.7, 0.8);

        // 墙壁3
        create_marker(4, "wall_3", 0.1, 10.0, 3.0, -5.0, 0.0, 1.5, 0.7, 0.7, 0.7, 0.8);

        // 墙壁4
        create_marker(5, "wall_4", 0.1, 10.0, 3.0, 5.0, 0.0, 1.5, 0.7, 0.7, 0.7, 0.8);

        // 门（在墙壁1上开一个小口）
        create_marker(6, "door", 2.0, 0.1, 2.0, 0.0, -4.9, 1.0, 0.6, 0.3, 0.2, 1.0);

        // 窗户（在墙壁2上开一个小窗）
        create_marker(7, "window", 2.0, 0.1, 1.5, 0.0, 4.9, 2.0, 0.3, 0.7, 0.9, 0.5);

        RCLCPP_INFO(this->get_logger(), "Published detailed room map markers.");
    }

    void create_marker(int id, const std::string& ns, float scale_x, float scale_y, float scale_z,
                       float pos_x, float pos_y, float pos_z,
                       float color_r, float color_g, float color_b, float color_a) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置大小
        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;

        // 设置位置
        marker.pose.position.x = pos_x;
        marker.pose.position.y = pos_y;
        marker.pose.position.z = pos_z;

        // 设置颜色和透明度
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.color.a = color_a;

        // 发布Marker
        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoomMapNode>());
    rclcpp::shutdown();
    return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// class RoomMapNode : public rclcpp::Node {
// public:
//     RoomMapNode() : Node("room_map_node") {
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("room_map", 10);
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(1),
//             std::bind(&RoomMapNode::publish_room_map, this)
//         );
//     }

// private:
//     void publish_room_map() {
//         auto marker = visualization_msgs::msg::Marker();
//         marker.header.frame_id = "map";
//         marker.header.stamp = this->get_clock()->now();
//         marker.ns = "room";
//         marker.id = 0;
//         marker.type = visualization_msgs::msg::Marker::CUBE;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         // 设置房间尺寸
//         marker.scale.x = 3.0;
//         marker.scale.y = 2.0;
//         marker.scale.z = 1.0;

//         // 设置位置
//         marker.pose.position.x = 3.0;
//         marker.pose.position.y = 0.0;
//         marker.pose.position.z = 1.5;

//         // 设置颜色和透明度
//         marker.color.r = 0.0f;
//         marker.color.g = 0.5f;
//         marker.color.b = 0.5f;
//         marker.color.a = 1.0f;  // 不透明

//         // 发布Marker
//         marker_pub_->publish(marker);
//         RCLCPP_INFO(this->get_logger(), "Published Room map marker with size 10x10x3 at map origin.");
//     }

//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<RoomMapNode>());
//     rclcpp::shutdown();
//     return 0;
// }
