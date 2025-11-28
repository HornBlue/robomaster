// ==================== 弹丸打击相关代码 ====================

// 包含文件
#include "referee_pkg/srv/hit_armor.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 弹道计算类
class BallisticCalculator {
public:
    BallisticCalculator() : initial_velocity_(1.5), g_(9.8) {}
    
    // 计算弹道命中点
    geometry_msgs::msg::Point calculateHitPoint(const geometry_msgs::msg::Point& target_center,
                                               double yaw, double pitch, double roll) {
        geometry_msgs::msg::Point hit_point;
        
        // 将欧拉角转换为旋转矩阵（外旋ZYX顺序）
        Eigen::Matrix3d rotation_matrix = eulerToRotationMatrix(yaw, pitch, roll);
        
        // 计算初始速度方向（在相机坐标系中，Z轴向前）
        Eigen::Vector3d initial_direction(0, 0, 1);
        Eigen::Vector3d world_direction = rotation_matrix * initial_direction;
        
        // 计算飞行时间（假设目标在Z=0平面）
        double t = calculateFlightTime(target_center, world_direction);
        
        // 计算命中点
        hit_point.x = initial_velocity_ * world_direction.x() * t;
        hit_point.y = initial_velocity_ * world_direction.y() * t;
        hit_point.z = initial_velocity_ * world_direction.z() * t - 0.5 * g_ * t * t;
        
        return hit_point;
    }
    
    void setGravity(double g) { g_ = g; }
    
private:
    double initial_velocity_;
    double g_;
    
    Eigen::Matrix3d eulerToRotationMatrix(double yaw, double pitch, double roll) {
        // 外旋ZYX顺序：先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll
        Eigen::Matrix3d Rz, Ry, Rx;
        
        // Z轴旋转 (yaw)
        Rz << cos(yaw), -sin(yaw), 0,
              sin(yaw),  cos(yaw), 0,
              0,         0,        1;
              
        // Y轴旋转 (pitch)
        Ry << cos(pitch),  0, sin(pitch),
              0,           1, 0,
             -sin(pitch),  0, cos(pitch);
             
        // X轴旋转 (roll)
        Rx << 1, 0,        0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);
              
        return Rz * Ry * Rx;
    }
    
    double calculateFlightTime(const geometry_msgs::msg::Point& target,
                              const Eigen::Vector3d& direction) {
        // 简化计算：假设目标在Z=0平面，求解二次方程
        double a = -0.5 * g_;
        double b = initial_velocity_ * direction.z();
        double c = 0; // 假设发射点高度为0
        
        // 解二次方程：a*t² + b*t + c = 0
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return 0.0; // 无解，返回0
        }
        
        double t1 = (-b + sqrt(discriminant)) / (2 * a);
        double t2 = (-b - sqrt(discriminant)) / (2 * a);
        
        // 返回正的时间解
        return (t1 > 0) ? t1 : t2;
    }
};

// 在TestNode类中添加的成员变量
private:
    // 服务端
    rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr hit_armor_service_;
    
    // 弹道计算器
    BallisticCalculator ballistic_calculator_;

// 在TestNode构造函数中添加服务创建
hit_armor_service_ = this->create_service<referee_pkg::srv::HitArmor>(
    "/referee/hit_armor",
    bind(&TestNode::handleHitArmor, this, std::placeholders::_1, std::placeholders::_2));
RCLCPP_INFO(this->get_logger(), "HitArmor service created");

// 服务处理函数实现
void TestNode::handleHitArmor(const std::shared_ptr<referee_pkg::srv::HitArmor::Request> request,
                             std::shared_ptr<referee_pkg::srv::HitArmor::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received HitArmor service call");
    
    try {
        // 设置重力加速度
        ballistic_calculator_.setGravity(request->g);
        
        // 计算装甲板中心点
        geometry_msgs::msg::Point armor_center = calculateArmorCenter(request->modelpoint);
        
        // 计算弹道命中点
        geometry_msgs::msg::Point hit_point = ballistic_calculator_.calculateHitPoint(
            armor_center, request->yaw, request->pitch, request->roll);
        
        // 设置响应时间戳
        response->header = request->header;
        
        // 计算命中结果（简化：如果命中点在装甲板中心附近则认为命中）
        double distance = sqrt(pow(hit_point.x - armor_center.x, 2) + 
                              pow(hit_point.y - armor_center.y, 2) + 
                              pow(hit_point.z - armor_center.z, 2));
        
        // 设置响应数据
        response->hit_point = hit_point;
        response->armor_center = armor_center;
        response->hit_distance = distance;
        response->is_hit = (distance < 0.1); // 10cm内认为命中
        
        RCLCPP_INFO(this->get_logger(), 
                   "Ballistic calculation completed - Hit: %s, Distance: %.3f", 
                   response->is_hit ? "YES" : "NO", distance);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in ballistic calculation: %s", e.what());
        response->is_hit = false;
    }
}

// 计算装甲板中心点函数
geometry_msgs::msg::Point TestNode::calculateArmorCenter(const std::vector<geometry_msgs::msg::Point>& model_points) {
    geometry_msgs::msg::Point center;
    
    if (model_points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No model points provided");
        return center;
    }
    
    // 计算所有点的平均值作为中心点
    for (const auto& point : model_points) {
        center.x += point.x;
        center.y += point.y;
        center.z += point.z;
    }
    
    center.x /= model_points.size();
    center.y /= model_points.size();
    center.z /= model_points.size();
    
    return center;
}

// 在阶段5添加特殊处理
case 5:
    RCLCPP_INFO(this->get_logger(), "Race Stage 5: Finished");
    RCLCPP_INFO(this->get_logger(), "Stage 5: HitArmor service is active for ballistic testing");
    break;
