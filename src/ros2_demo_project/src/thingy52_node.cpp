#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class Thingy52Node : public rclcpp::Node {
public:
    Thingy52Node() : Node("thingy52_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thingy52/accel_data", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&Thingy52Node::publish_data, this)
        );
    }

private:
    void publish_data() {
        auto message = std_msgs::msg::Float32MultiArray();
        message.data = {random_value(), random_value(), random_value()};
        publisher_->publish(message);
    }

    float random_value() {
        return (float(rand()) / float(RAND_MAX)) * 2.0 - 1.0;
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thingy52Node>());
    rclcpp::shutdown();
    return 0;
}


