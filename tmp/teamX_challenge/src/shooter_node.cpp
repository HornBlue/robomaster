#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace rclcpp;

class ShooterNode : public Node {
public:
    ShooterNode() : Node("shooter_node") {
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/decision/aim_angle", 10, 
            bind(&ShooterNode::angle_callback, this, placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "打击节点已启动，等待打击方向...");
    }
private:
    Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;

    void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // 接收到打击方向
        RCLCPP_INFO(this->get_logger(), "收到打击方向 -> X: %.2f, Y: %.2f", msg->x, msg->y);
        
        // 此处可添加控制硬件（如舵机云台）的代码
        // ...
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<ShooterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}