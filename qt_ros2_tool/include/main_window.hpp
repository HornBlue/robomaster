// main_window.hpp

#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "test1.h"  // 注意：根据你的UI文件生成的头文件名
#include "geometry_msgs/msg/pose.hpp"

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/header.hpp>

#include <vector>
#include <string>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void publish();


private slots:
    void on_publishButton_clicked();
    void spinOnce();
    void value_x(int);
    void value_y(int);
    void value_z(int);
    void value_yaw(int);
    void value_pitch(int);
    void value_roll(int);
    void move();

private:
    float x = 0, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0;
    int move_type = 0;
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_move;
    QTimer *ros_timer_;
};

#endif // MAIN_WINDOW_HPP