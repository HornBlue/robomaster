#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/apply_link_wrench.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <string>
#include <vector>
#include <memory>

class ProjectileLauncher : public rclcpp::Node
{
public:
    ProjectileLauncher() : Node("projectile_launcher")
    {
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");
        wrench_client_ = this->create_client<gazebo_msgs::srv::ApplyLinkWrench>("apply_link_wrench");

        // Wait for services
        while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for spawn service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for spawn_entity service...");
        }
        while (!delete_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for delete service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for delete_entity service...");
        }
        while (!wrench_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for apply_link_wrench service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for apply_link_wrench service...");
        }

        initial_velocity_ = 1.5f;
        projectile_mass_ = 0.1f;  // kg, must match SDF
    }

    void launchProjectile(float yaw, float pitch, float roll, float timeout)
    {
        std::string proj_name = "projectile_" + std::to_string(counter_++);
        std::string link_name = proj_name + "::link";

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw); // ZYX extrinsic = RPY
        quat.normalize();

        // SDF with consistent mass
        std::string sdf = R"(
<sdf version='1.6'>
  <model name=')" + proj_name + R"('>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>)" + std::to_string(projectile_mass_) + R"(</mass>
        <inertia>
          <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </collision>
      <visual name='visual'>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material><ambient>1 0 0 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>)";

        auto spawn_req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        spawn_req->name = proj_name;
        spawn_req->xml = sdf;
        spawn_req->initial_pose.position.x = 0.0;
        spawn_req->initial_pose.position.y = 0.0;
        spawn_req->initial_pose.position.z = 0.0;
        spawn_req->initial_pose.orientation.w = quat.w();
        spawn_req->initial_pose.orientation.x = quat.x();
        spawn_req->initial_pose.orientation.y = quat.y();
        spawn_req->initial_pose.orientation.z = quat.z();

        auto spawn_future = spawn_client_->async_send_request(spawn_req);

        // Compute velocity vector
        double vx = initial_velocity_ * cos(pitch) * cos(yaw);
        double vy = initial_velocity_ * cos(pitch) * sin(yaw);
        double vz = initial_velocity_ * sin(pitch);

        // Delay a bit to ensure model is spawned before applying wrench
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Apply impulse via wrench (F = m * a, but we use short duration)
        double dt = 0.01; // seconds
        double fx = projectile_mass_ * vx / dt;
        double fy = projectile_mass_ * vy / dt;
        double fz = projectile_mass_ * vz / dt;

        auto wrench_req = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
        wrench_req->link_name = link_name;
        wrench_req->wrench.force.x = fx;
        wrench_req->wrench.force.y = fy;
        wrench_req->wrench.force.z = fz;
        wrench_req->duration.sec = 0;
        wrench_req->duration.nanosec = static_cast<uint32_t>(dt * 1e9);

        auto wrench_future = wrench_client_->async_send_request(wrench_req);

        // Set up auto-delete
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timeout * 1000)),
            [this, proj_name]() {
                deleteProjectile(proj_name);
            }
        );
        timers_.push_back(timer);

        RCLCPP_INFO(this->get_logger(), "Launched %s: v=(%.2f,%.2f,%.2f) m/s, timeout=%.1fs",
                   proj_name.c_str(), vx, vy, vz, timeout);
    }

private:
    void deleteProjectile(const std::string &name)
    {
        auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        req->name = name;
        delete_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Deleted %s", name.c_str());
    }

    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr wrench_client_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    float initial_velocity_;
    float projectile_mass_;
    int counter_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProjectileLauncher>();

    RCLCPP_INFO(node->get_logger(), "Launching demo projectiles...");

    // Demo launches (all from origin, with different directions)
    // node->launchProjectile(0.0f, 0.0f, 0.0f, 5.0f);          // +x
    // node->launchProjectile(M_PI_2, 0.0f, 0.0f, 5.0f);        // +y
    node->launchProjectile(0.0f, M_PI_4, 0.0f, 6.0f);        // 45° up in x

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}