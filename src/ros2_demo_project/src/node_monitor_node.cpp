#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <fstream>
#include <map>
#include <string>

class NodeMonitorNode : public rclcpp::Node {
public:
    NodeMonitorNode() : Node("node_monitor_node") {
        // 读取参数或设置默认文件路径
        this->declare_parameter<std::string>("output_directory", "./");
        this->get_parameter("output_directory", output_directory_);

        // 创建定时器，定时检查并记录所有节点
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), 
            std::bind(&NodeMonitorNode::check_and_record_nodes, this)
        );
        RCLCPP_INFO(this->get_logger(), "Node Monitor Node is running with output directory: %s", output_directory_.c_str());
    }

private:
    void check_and_record_nodes() {
        // 获取所有节点名称
        auto node_list = this->get_node_names();
        RCLCPP_INFO(this->get_logger(), "Number of nodes detected: %zu", node_list.size());
        
        for (const auto& node_name : node_list) {
            RCLCPP_INFO(this->get_logger(), "Detected node: %s", node_name.c_str());
            

            // 如果该节点还没有文件，则创建一个文件并订阅其消息
            if (node_subscribers_.find(node_name) == node_subscribers_.end()) {
                // 订阅该节点的所有主题
                subscribe_to_node(node_name);
            }
        }
    }

    void subscribe_to_node(const std::string& node_name) {
        // 为每个节点创建一个对应的订阅者，保存到 node_subscribers_
        auto sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10,
            [this, node_name](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
                // 处理接收到的消息，将其写入文件
                write_message_to_file(node_name, msg);
                RCLCPP_INFO(this->get_logger(), "Received message from node: %s", node_name.c_str());
            }
        );
        node_subscribers_[node_name] = sub;

        // 指定文件路径并尝试创建
        std::string file_path = output_directory_ + "/" + node_name + ".txt";
        std::ofstream outfile(file_path, std::ios_base::app);
        if (outfile.is_open()) {
            outfile << "Monitoring node: " << node_name << "\n";
            RCLCPP_INFO(this->get_logger(), "Created file: %s", file_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to create file: %s", file_path.c_str());
        }
        outfile.close();
    }

    void write_message_to_file(const std::string& node_name, const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
        std::ofstream outfile(output_directory_ + "/" + node_name + ".txt", std::ios_base::app);
        outfile << "Received message from node: " << node_name << "\n";

        // 输出新增的参数
        for (const auto& parameter : msg->new_parameters) {
            outfile << "New Parameter: " << parameter.name << " = " << parameter.value.string_value << "\n";
        }

        // 输出更改的参数
        for (const auto& parameter : msg->changed_parameters) {
            outfile << "Changed Parameter: " << parameter.name << " = " << parameter.value.string_value << "\n";
        }

        // 输出删除的参数
        for (const auto& parameter : msg->deleted_parameters) {
            outfile << "Deleted Parameter: " << parameter.name << "\n";
        }

        outfile << "-------------------------------------\n";
        outfile.close();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr> node_subscribers_;
    std::string output_directory_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
