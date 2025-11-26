#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "teamX_challenge/msg/target_array.hpp"

using namespace rclcpp;

class DecisionNode : public Node {
public:
    DecisionNode() : Node("decision_node") {
        // 获取弹道和解算参数
        // ...
        
        targets_sub_ = this->create_subscription<teamX_challenge::msg::TargetArray>(
            "/vision/targets", 10, bind(&DecisionNode::targets_callback, this, placeholders::_1)
        );
        aim_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/decision/aim_angle", 10);
        RCLCPP_INFO(this->get_logger(), "决策节点已启动。只打击装甲板。");
    }
private:
    Subscription<teamX_challenge::msg::TargetArray>::SharedPtr targets_sub_;
    Publisher<geometry_msgs::msg::Vector3>::SharedPtr aim_pub_;
    vector<int> color_priority_;

    void targets_callback(const teamX_challenge::msg::TargetArray::SharedPtr msg) {
        vector<const teamX_challenge::msg::Target*> armor_targets;
        for (const auto& t : msg->targets) {
            if (t.type == "ARMOR") {
                armor_targets.push_back(&t);
            }
        }

        if (armor_targets.empty()) {
            RCLCPP_INFO(this->get_logger(), "未检测到装甲板。");
            return;
        }

        // 按颜色优先级排序
        sort(armor_targets.begin(), armor_targets.end(), [this](const teamX_challenge::msg::Target* a, const teamX_challenge::msg::Target* b) {
            auto rank_a = find(color_priority_.begin(), color_priority_.end(), a->color_id) - color_priority_.begin();
            auto rank_b = find(color_priority_.begin(), color_priority_.end(), b->color_id) - color_priority_.begin();
            return rank_a < rank_b;
        });
        
        const teamX_challenge::msg::Target* best_armor = armor_targets[0];
        RCLCPP_INFO(this->get_logger(), "选中装甲板: 颜色ID=%d, 中心=(%.2f, %.2f)",
                   best_armor->color_id, best_armor->center.x, best_armor->center.y);

        // 计算打击角度
        geometry_msgs::msg::Vector3 angle_msg;
        // 简化：这里返回像素坐标作为方向，实际应进行相机标定和弹道解算
        angle_msg.x = best_armor->center.x; // 水平方向 (像素)
        angle_msg.y = best_armor->center.y; // 垂直方向 (像素)
        angle_msg.z = 0.0;
        
        aim_pub_->publish(angle_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}