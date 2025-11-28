// main.cpp
#include <QApplication>
#include "main_window.hpp"

int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建Qt应用
    QApplication app(argc, argv);
    
    // 创建并显示主窗口
    MainWindow window;
    window.show();
    
    // 运行Qt事件循环
    return app.exec();
}