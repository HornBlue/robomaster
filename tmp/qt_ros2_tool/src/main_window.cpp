// main_window.cpp
#include "main_window.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , node_(std::make_shared<rclcpp::Node>("qt_ros2_node"))
{
    // 设置UI
    ui->setupUi(this);
    
    // 创建ROS2发布者
    publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("pose", 10);
    publisher_move = node_->create_publisher<std_msgs::msg::Int32>("type", 10);
    
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &MainWindow::spinOnce);
    ros_timer_->start(10);  // 每10ms处理一次ROS2事件
    
    connect(ui->pushButton, &QPushButton::clicked, 
            this, &MainWindow::on_publishButton_clicked);    
    connect(ui->PB_MV, &QPushButton::clicked, 
            this, &MainWindow::move);

    ui->HS_yaw->setMinimum(-180);  // 最小值
    ui->HS_yaw->setMaximum(180);  // 最大值
    ui->HS_yaw->setSingleStep(1);  // 步长
    ui->HS_roll->setMinimum(-180);  // 最小值
    ui->HS_roll->setMaximum(180);  // 最大值
    ui->HS_roll->setSingleStep(1);  // 步长
    ui->HS_pitch->setMinimum(-180);  // 最小值
    ui->HS_pitch->setMaximum(180);  // 最大值
    ui->HS_pitch->setSingleStep(1);  // 步长
    ui->HS_x->setMinimum(-50);  // 最小值
    ui->HS_x->setMaximum(50);  // 最大值
    ui->HS_x->setSingleStep(1);  // 步长
    ui->HS_y->setMinimum(-50);  // 最小值
    ui->HS_y->setMaximum(50);  // 最大值
    ui->HS_y->setSingleStep(1);  // 步长
    ui->HS_z->setMinimum(-50);  // 最小值
    ui->HS_z->setMaximum(50);  // 最大值
    ui->HS_z->setSingleStep(1);  // 步长

    connect(ui->HS_x, &QSlider::valueChanged, 
            this, &MainWindow::value_x);
    connect(ui->HS_y, &QSlider::valueChanged, 
            this, &MainWindow::value_y);
    connect(ui->HS_z, &QSlider::valueChanged, 
            this, &MainWindow::value_z);
    connect(ui->HS_yaw, &QSlider::valueChanged, 
            this, &MainWindow::value_yaw);
    connect(ui->HS_pitch, &QSlider::valueChanged, 
            this, &MainWindow::value_pitch);
    connect(ui->HS_roll, &QSlider::valueChanged, 
            this, &MainWindow::value_roll);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::publish()
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Pose message;
    message.position.x = x;
    message.position.y = y;
    message.position.z = z;
    message.orientation = tf2::toMsg(q);

    publisher_->publish(message);

    // 改进状态栏
    QString status = QString("x: %1, y: %2, z: %3 | Yaw: %4°, Pitch: %5°, Roll: %6°")
        .arg(x, 0, 'f', 2)
        .arg(y, 0, 'f', 2)
        .arg(z, 0, 'f', 2)
        .arg(yaw * 180 / M_PI, 0, 'f', 1)
        .arg(pitch * 180 / M_PI, 0, 'f', 1)
        .arg(roll * 180 / M_PI, 0, 'f', 1);
    ui->statusbar->showMessage(status, 2000); // 显示2秒
}


void MainWindow::on_publishButton_clicked()
{
    QString x_s = ui->LE_x->text().trimmed();
    QString y_s = ui->LE_y->text().trimmed();
    QString z_s = ui->LE_z->text().trimmed();
    QString yaw_s = ui->LE_yaw->text().trimmed();
    QString pitch_s = ui->LE_pitch->text().trimmed();
    QString roll_s = ui->LE_roll->text().trimmed();

    x = x_s.toFloat();
    y = y_s.toFloat();
    z = z_s.toFloat();    
    yaw = yaw_s.toFloat() * M_PI / 180;
    pitch = pitch_s.toFloat() * M_PI / 180;
    roll = roll_s.toFloat() * M_PI / 180;

    ui->HS_x->setValue(static_cast<int>(x * 10));
    ui->HS_y->setValue(static_cast<int>(y * 10));
    ui->HS_z->setValue(static_cast<int>(z * 10));
    ui->HS_yaw->setValue(yaw_s.toFloat());
    ui->HS_pitch->setValue(pitch_s.toFloat());
    ui->HS_roll->setValue(roll_s.toFloat());

    publish();

    publish();
}

void MainWindow::spinOnce()
{
    rclcpp::spin_some(node_);
}

void MainWindow::move(){
    move_type = 1 - move_type;
    std_msgs::msg::Int32 message;
    message.data = move_type;

    publisher_move->publish(message);
}

void MainWindow::value_x(int val){
    x = float(val) / 10;
    ui->LE_x->setText(QString::number(x));
    publish();
}
void MainWindow::value_y(int val){
    y = float(val) / 10;
    ui->LE_y->setText(QString::number(y));
    publish();
}
void MainWindow::value_z(int val){
    z = float(val) / 10;
    ui->LE_z->setText(QString::number(z));
    publish();
}
void MainWindow::value_yaw(int val){
    yaw = val * M_PI / 180;
    ui->LE_yaw->setText(QString::number(val));
    publish();
}
void MainWindow::value_pitch(int val){
    pitch = val * M_PI / 180;
    ui->LE_pitch->setText(QString::number(val));
    publish();
}
void MainWindow::value_roll(int val){
    roll = val * M_PI / 180;
    ui->LE_roll->setText(QString::number(val));
    publish();
}
