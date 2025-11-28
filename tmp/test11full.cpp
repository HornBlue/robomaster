#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "referee_pkg/msg/race_stage.hpp"  // 添加比赛阶段消息头文件

using namespace std;
using namespace rclcpp;
using namespace cv;
struct DetectedObject {
    string type;
    vector<Point2f> corners;
    //Point2f center;
    string number; // 存储识别到的数字
};
class TestNode : public rclcpp::Node {
 public:
  TestNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing TestNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&TestNode::callback_camera, this, std::placeholders::_1));

    // 添加比赛阶段订阅者
    race_stage_sub_ = this->create_subscription<referee_pkg::msg::RaceStage>(
        "/referee/race_stage", 10,
        bind(&TestNode::callback_race_stage, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

    // 性能优化参数
    detection_scale = 0.5;  // 图像缩放因子，提高处理速度
    min_contour_area = 100; // 最小轮廓面积，减少计算量
    
    // 初始化比赛阶段
    current_stage_ = 0;
    
    // ========== 抗光照干扰参数初始化 ==========
    // 自适应参数
    use_adaptive_threshold = true;
    clahe_clip_limit = 2.0;
    clahe_grid_size = 8;
    
    // 颜色范围自适应参数
    red_hue_margin = 10;      // 红色色调容差
    green_hue_margin = 10;    // 绿色色调容差
    min_saturation = 60;      // 最小饱和度阈值
    min_value = 60;           // 最小亮度阈值
    
    // 形态学操作参数
    morph_kernel_size = 3;
    
    // 创建CLAHE对象
    clahe = createCLAHE(clahe_clip_limit, Size(clahe_grid_size, clahe_grid_size));
    
    RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
  }

  ~TestNode() { cv::destroyWindow("Detection Result"); }
  
 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  
  // 添加比赛阶段回调函数
  void callback_race_stage(referee_pkg::msg::RaceStage::SharedPtr msg);
  
  // 球体检测
  void detectSpheres(const Mat& hsv, Mat& result_image);
  
  // 绿色矩形检测
  void detectGreenRectangles(const Mat& hsv, Mat& result_image);
  
  // 装甲板检测
  void detectArmorPlates(const Mat& hsv, Mat& result_image);
  
  // 数字识别
  string recognizeNumber(const Mat& number_roi);
  
  // 稳定的球体点计算方法
  vector<Point2f> calculateStableSpherePoints(const Point2f &center, float radius);
  
  // 矩形角点排序（左下，右下，右上，左上）
  vector<Point2f> sortRectanglePoints(vector<Point2f> points);
  
  // ========== 抗光照干扰新增函数 ==========
  // 图像预处理 - 增强对比度和减少光照影响
  Mat preprocessImage(const Mat& input_image);
  
  // 自适应颜色范围计算
  void getAdaptiveColorRange(const Mat& hsv, Scalar& lower_red1, Scalar& upper_red1, 
                            Scalar& lower_red2, Scalar& upper_red2,
                            Scalar& lower_green, Scalar& upper_green);
  
  // 动态阈值分割
  Mat adaptiveColorSegmentation(const Mat& hsv, const string& color_type);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  
  // 添加比赛阶段订阅者
  rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr race_stage_sub_;
  
  //vector<Point2f> Point_V;
  vector<DetectedObject> detected_objects_;  // 替换原来的 Point_V
  // 性能优化参数
  double detection_scale;
  int min_contour_area;
  
  // 当前比赛阶段
  int current_stage_;
  
  // ========== 抗光照干扰新增成员变量 ==========
  Ptr<CLAHE> clahe;
  bool use_adaptive_threshold;
  double clahe_clip_limit;
  int clahe_grid_size;
  int red_hue_margin;
  int green_hue_margin;
  int min_saturation;
  int min_value;
  int morph_kernel_size;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// ========== 抗光照干扰新增函数实现 ==========
Mat TestNode::preprocessImage(const Mat& input_image) {
    Mat processed = input_image.clone();
    
    // 转换为HSV颜色空间
    Mat hsv;
    cvtColor(processed, hsv, COLOR_BGR2HSV);
    
    // 分离通道
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    
    // 对V通道（亮度）进行CLAHE直方图均衡化，增强对比度
    clahe->apply(hsv_channels[2], hsv_channels[2]);
    
    // 合并通道
    merge(hsv_channels, hsv);
    
    // 转换回BGR
    cvtColor(hsv, processed, COLOR_HSV2BGR);
    
    // 高斯模糊去噪
    GaussianBlur(processed, processed, Size(3, 3), 0);
    
    return processed;
}

void TestNode::getAdaptiveColorRange(const Mat& hsv, Scalar& lower_red1, Scalar& upper_red1, 
                                    Scalar& lower_red2, Scalar& upper_red2,
                                    Scalar& lower_green, Scalar& upper_green) {
    // 计算图像的平均亮度
    Mat v_channel;
    extractChannel(hsv, v_channel, 2);
    Scalar mean_val = mean(v_channel);
    double brightness_factor = mean_val[0] / 128.0; // 归一化亮度因子
    
    // 自适应调整饱和度阈值
    int adaptive_saturation = static_cast<int>(min_saturation * (1.0 + (1.0 - brightness_factor) * 0.3));
    adaptive_saturation = min(max(adaptive_saturation, 50), 120);
    
    // 自适应调整亮度阈值
    int adaptive_value = static_cast<int>(min_value * (1.0 + (1.0 - brightness_factor) * 0.3));
    adaptive_value = min(max(adaptive_value, 50), 100);
    
    // 红色范围（考虑色调环绕）
    lower_red1 = Scalar(0, adaptive_saturation, adaptive_value);
    upper_red1 = Scalar(red_hue_margin, 255, 255);
    lower_red2 = Scalar(180 - red_hue_margin, adaptive_saturation, adaptive_value);
    upper_red2 = Scalar(180, 255, 255);
    
    // 绿色范围
    lower_green = Scalar(60 - green_hue_margin, adaptive_saturation, adaptive_value);
    upper_green = Scalar(60 + green_hue_margin, 255, 255);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Adaptive params - Sat: %d, Val: %d, Brightness: %.2f", 
                adaptive_saturation, adaptive_value, brightness_factor);
}

Mat TestNode::adaptiveColorSegmentation(const Mat& hsv, const string& color_type) {
    Mat mask;
    
    if (color_type == "red") {
        Scalar lower_red1, upper_red1, lower_red2, upper_red2, lower_green, upper_green;
        getAdaptiveColorRange(hsv, lower_red1, upper_red1, lower_red2, upper_red2, lower_green, upper_green);
        
        Mat mask1, mask2;
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2;
    }
    else if (color_type == "green") {
        Scalar lower_red1, upper_red1, lower_red2, upper_red2, lower_green, upper_green;
        getAdaptiveColorRange(hsv, lower_red1, upper_red1, lower_red2, upper_red2, lower_green, upper_green);
        
        inRange(hsv, lower_green, upper_green, mask);
    }
    
    // 自适应形态学操作
    int adaptive_kernel_size = morph_kernel_size;
    if (mask.cols > 1000) { // 根据图像尺寸调整核大小
        adaptive_kernel_size = 5;
    }
    
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(adaptive_kernel_size, adaptive_kernel_size));
    
    // 先开运算去噪，后闭运算填充空洞
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    
    return mask;
}

// 添加比赛阶段回调函数实现
void TestNode::callback_race_stage(referee_pkg::msg::RaceStage::SharedPtr msg) {
    current_stage_ = msg->stage;
    RCLCPP_INFO(this->get_logger(), "Received race stage: %d", current_stage_);
    
    // 这里可以根据比赛阶段进行相应的处理
    // 例如：不同阶段启用不同的检测算法
    switch(current_stage_) {
        case 1:
            RCLCPP_INFO(this->get_logger(), "Race Stage 1: Preparation phase");
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "Race Stage 2: Start phase");
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "Race Stage 3: Main competition phase");
            break;
        case 4:
            RCLCPP_INFO(this->get_logger(), "Race Stage 4: Ending phase");
            break;
        case 5:
            RCLCPP_INFO(this->get_logger(), "Race Stage 5: Finished");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown race stage: %d", current_stage_);
            break;
    }
}

vector<Point2f> TestNode::calculateStableSpherePoints(const Point2f &center,
                                                      float radius) {
  vector<Point2f> points;

  // 简单稳定的几何计算，避免漂移
  // 左、下、右、上
  points.push_back(Point2f(center.x - radius, center.y));  // 左点 (1)
  points.push_back(Point2f(center.x, center.y + radius));  // 下点 (2)
  points.push_back(Point2f(center.x + radius, center.y));  // 右点 (3)
  points.push_back(Point2f(center.x, center.y - radius));  // 上点 (4)

  return points;
}

vector<Point2f> TestNode::sortRectanglePoints(vector<Point2f> points) {
  if (points.size() != 4) return points;
  
  // 计算中心点
  Point2f center(0, 0);
  for (const auto& p : points) {
    center += p;
  }
  center.x /= 4;
  center.y /= 4;
  
  // 排序：左上，右上，右下，左下
  vector<Point2f> sorted(4);
  for (const auto& p : points) {
    if (p.x < center.x && p.y < center.y) sorted[3] = p; // 左上
    else if (p.x > center.x && p.y < center.y) sorted[2] = p; // 右上
    else if (p.x > center.x && p.y > center.y) sorted[1] = p; // 右下
    else if (p.x < center.x && p.y > center.y) sorted[0] = p; // 左下
  }
  
  return sorted;
}

    
void TestNode::detectSpheres(const Mat& hsv, Mat& result_image) {
  // ========== 抗光照干扰修改：使用自适应颜色分割 ==========
  Mat mask = adaptiveColorSegmentation(hsv, "red");

  // 找轮廓
  vector<vector<Point>> contours;
  findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area < 500) continue;

    // 计算最小外接圆
    Point2f center;
    float radius = 0;
    minEnclosingCircle(contours[i], center, radius);

    // 计算圆形度
    double perimeter = arcLength(contours[i], true);
    double circularity = 4 * CV_PI * area / (perimeter * perimeter);

    if (circularity > 0.7 && radius > 15 && radius < 200) {
      vector<Point2f> sphere_points = calculateStableSpherePoints(center, radius);

      // 绘制检测到的球体
      circle(result_image, center, static_cast<int>(radius), Scalar(0, 255, 0), 2);
      circle(result_image, center, 3, Scalar(0, 0, 255), -1);

      // 绘制球体上的四个点
      vector<Scalar> point_colors = {
          Scalar(255, 0, 0),    // 蓝色 - 左
          Scalar(0, 255, 0),    // 绿色 - 下
          Scalar(0, 255, 255),  // 黄色 - 右
          Scalar(255, 0, 255)   // 紫色 - 上
      };
      DetectedObject sphere_obj;
        sphere_obj.type = "sphere";
        sphere_obj.corners = sphere_points;
        //sphere_obj.center = center;
        
        detected_objects_.push_back(sphere_obj);
      for (int j = 0; j < 4; j++) {
        circle(result_image, sphere_points[j], 6, point_colors[j], -1);
        circle(result_image, sphere_points[j], 6, Scalar(0, 0, 0), 2);

        // 标注序号
        string point_text = to_string(j + 1);
        putText(result_image, point_text,
                Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 3);
        putText(result_image, point_text,
                Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);

        // // 添加到发送列表
        // Point_V.push_back(sphere_points[j]);
      }

      // 显示半径信息
      string info_text = "Sphere R:" + to_string((int)radius);
      putText(result_image, info_text, Point(center.x - 25, center.y + 5),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);

      RCLCPP_INFO(this->get_logger(), "Found sphere: (%.1f, %.1f) R=%.1f", 
                  center.x, center.y, radius);
    }
  }
}

void TestNode::detectGreenRectangles(const Mat& hsv, Mat& result_image) {
  // ========== 抗光照干扰修改：使用自适应颜色分割 ==========
  Mat green_mask = adaptiveColorSegmentation(hsv, "green");

  // 找轮廓
  vector<vector<Point>> contours;
  findContours(green_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = contourArea(contour);
    if (area < 500) continue;

    // 多边形近似
    vector<Point> approx;
    double epsilon = 0.02 * arcLength(contour, true);
    approxPolyDP(contour, approx, epsilon, true);

    // 检查是否为四边形
    if (approx.size() == 4) {
      Rect rect = boundingRect(approx);
      //double aspect_ratio = static_cast<double>(rect.width) / rect.height;
      
      // 筛选合理的矩形
      //if (aspect_ratio > 0.5 && aspect_ratio < 2.0) {
        // 绘制矩形
        
        for (int i = 0; i < 4; i++) {
          line(result_image, approx[i], approx[(i + 1) % 4], Scalar(0, 0, 255), 2, 3);
        }

        // 排序角点
        vector<Point2f> sorted_points = sortRectanglePoints(
            vector<Point2f>(approx.begin(), approx.end()));
        vector<Scalar> point_colors = {
            Scalar(255, 0, 0),    // 蓝色 - 左下
            Scalar(0, 255, 0),    // 绿色 - 右下
            Scalar(0, 255, 255),  // 黄色 - 右上
            Scalar(255, 0, 255)   // 紫色 - 左上
        };
        // 绘制角点并添加到发布列表
        for (int i = 0; i < 4; i++) {
          circle(result_image, sorted_points[i], 8, point_colors[i], -1);
          //Point_V.push_back(sorted_points[i]);
          
          // 标注角点序号
          string text = to_string(i + 1);
          putText(result_image, text, 
                  Point(sorted_points[i].x + 12, sorted_points[i].y - 12),
                  FONT_HERSHEY_SIMPLEX, 0.7, point_colors[i], 2);
        }
        DetectedObject rectangle_obj;
        rectangle_obj.type = "rect";
        rectangle_obj.corners = sorted_points;
        //rectangle_obj.center = center;
        
        detected_objects_.push_back(rectangle_obj);
        // 标注矩形信息
        string info = "Green Rect";
        putText(result_image, info, Point(rect.x, rect.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

        RCLCPP_INFO(this->get_logger(), "Found green rectangle");
      //}
    }
  }
}
class DigitMatcher {
private:
    vector<string> template_paths = {
        "src/player_pkg/small_num1.png",
        "src/player_pkg/small_num2.png",
        "src/player_pkg/small_num3.png",
        "src/player_pkg/small_num4.png",
        "src/player_pkg/small_num5.png"
    };
    vector<Mat> templates;
    double match_threshold = 1.0;  // 形状匹配阈值（越小越相似）

    // 预处理函数
    Mat preprocess(const Mat& img) {
        if (img.empty()) return Mat();

        // 1. 去红框
        Mat hsv, red_mask1, red_mask2, red_mask, no_red;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 80, 50), Scalar(10, 255, 255), red_mask1);
        inRange(hsv, Scalar(160, 80, 50), Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
        dilate(red_mask, red_mask, kernel);
        no_red = img.clone();
        no_red.setTo(Scalar(0,0,0), red_mask);

        // 2. 二值化（保留轮廓）
        Mat gray, binary;
        cvtColor(no_red, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 100, 255, THRESH_BINARY);

        // 3. 轻微去噪
        erode(binary, binary, getStructuringElement(MORPH_RECT, Size(2,2)));
        dilate(binary, binary, getStructuringElement(MORPH_RECT, Size(3,3)));

        // 4. 强制白字黑底
        int white_pixels = countNonZero(binary);
        if (white_pixels < binary.total() * 0.1) {
            binary = 255 - binary;
            //cout << "[调试] 预处理：强制反转（白字黑底）" << endl;
        }

        return binary;
    }

    // 检测弯曲数量
    int countCurves(const Mat& binary_img) {
        vector<vector<Point>> contours;
        findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return 0;

        vector<Point> max_contour = contours[0];
        for (auto& c : contours) {
            if (contourArea(c) > contourArea(max_contour)) {
                max_contour = c;
            }
        }

        vector<int> hull;
        convexHull(max_contour, hull, false);
        vector<Vec4i> defects;
        convexityDefects(max_contour, hull, defects);

        int curve_count = 0;
      for (auto& d : defects) {
          float depth = d[3] / 256.0;
          // 降低阈值（从20→15），让3的中间浅弯被识别
          if (depth > 15) curve_count++;  
      }
      return curve_count;
    }

    // 加载模板
    bool loadTemplates() {
        templates.clear();
        char cwd[1024];
        getcwd(cwd, sizeof(cwd));
        //cout << "[调试] 当前工作目录：" << cwd << endl;

        for (int i=0; i<template_paths.size(); i++) {
            const string& path = template_paths[i];
            Mat temp = imread(path);
            if (temp.empty()) {
                //cout << "[错误] 模板加载失败：" << path << endl;
                return false;
            }
            Mat temp_pre = preprocess(temp);
            if (i == 0) {  
            // 纵向膨胀，强化1的竖线特征
            dilate(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,5)));  
            // 横向腐蚀，消除可能的横向噪声（避免像5的横杠）
            erode(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(3,1)));  
            //cout << "[调试] 模板1增强直线特征" << endl;
        }
            // 模板2单独检查
            if (i == 1) {
                int white_pixels = countNonZero(temp_pre);
                if (white_pixels < temp_pre.total() * 0.2) {
                    temp_pre = 255 - temp_pre;
                    //cout << "[调试] 模板2：强制反转（白字黑底）" << endl;
                }
            }
            if (i == 2) {  
            // 局部膨胀中间区域（假设弯曲在中间），强化弯曲轮廓
            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
            dilate(temp_pre, temp_pre, kernel);  
            //cout << "[调试] 模板3增强中间弯曲特征" << endl;
        }
            if (i == 4) {  // 数字5的索引是4
    // 轻微腐蚀，让横杠更清晰
    erode(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,1)));
    //cout << "[调试] 模板5增强横杠特征" << endl;
}
            templates.push_back(temp_pre);
            //cout << "[调试] 模板" << (i+1) << "加载成功，尺寸：" << temp_pre.size() << endl;
        }
        return true;
    }

public:
    // 构造函数
    DigitMatcher() {
        if (!loadTemplates()) {
            cerr << "[错误] 模板加载失败，程序退出" << endl;
            exit(-1);
        }
    }

    // 核心匹配函数（类内定义，位置正确）
    string matchDigit(const Mat& input_img) {
    if (input_img.empty()) return "unknown";

    // 1. 预处理
    Mat img_pre = preprocess(input_img);
    if (img_pre.empty()) return "unknown";
    imshow("待识别数字预处理后", img_pre);
    waitKey(1);

    // 2. 提取待识别轮廓
    vector<vector<Point>> target_contours;
    findContours(img_pre, target_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (target_contours.empty()) return "unknown";
    vector<Point> target_contour = target_contours[0];

    // 3. 计算弯曲数
    int target_curves = countCurves(img_pre);
    //cout << "[调试] 待识别数字弯曲数量：" << target_curves << endl;

    // 4. 形状匹配 + 存储所有数字的距离（核心：定义shape_distances）
    double min_shape_dist = 1e9;
    int best_match_idx = -1;
    vector<double> shape_distances;  // 定义：存储每个数字的形状匹配距离
    shape_distances.resize(templates.size(), 1e9);  // 初始化，默认值1e9

    for (int i=0; i<templates.size(); i++) {
        const Mat& temp = templates[i];
        if (temp.empty()) continue;

        vector<vector<Point>> temp_contours;
        findContours(temp, temp_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (temp_contours.empty()) continue;
        vector<Point> temp_contour = temp_contours[0];

        // 计算形状距离
        double dist = matchShapes(target_contour, temp_contour, CONTOURS_MATCH_I1, 0);
        shape_distances[i] = dist;  // 存储当前数字的距离
        //cout << "[调试] 形状匹配数字" << (i+1) << "，距离：" << dist << endl;

        // 更新最小距离和最佳索引
        if (dist < min_shape_dist) {
            min_shape_dist = dist;
            best_match_idx = i;
        }
    }

    // 5. 所有修正逻辑（整合）
    // 修正1：1被误判为2（弯曲数≥2）
    if (best_match_idx == 0 && target_curves >= 2) {
        //cout << "[调试] 修正：有" << target_curves << "个弯曲，改为数字2" << endl;
        best_match_idx = 1;
        min_shape_dist = shape_distances[1];  // 同步更新距离
    }

    // 修正2：5被误判为2（弯曲数=2）
    if (best_match_idx == 4) {  // 当前最佳是5（索引4）
    int digit2_idx = 1;  // 数字2的索引是1
    double digit2_dist = shape_distances[digit2_idx];  // 取2的距离
    double distance_diff = digit2_dist - shape_distances[4];  // 2与5的距离差（正值表示5更近）

    // 条件调整：弯曲数=2，且5与2的距离差<0.3（差距不大时才修正）
    if (target_curves == 2 && distance_diff < 0.3) {  
        //cout << "[调试] 修正：有2个弯曲（5通常1个），且5与2距离接近（差" << distance_diff 
         //    << "）→ 改为数字2" << endl;
        best_match_idx = digit2_idx;
        min_shape_dist = digit2_dist;
    } else if (target_curves == 2) {
        // 新增：当5的距离远小于2时，即使弯曲数=2，也保留5（提示弯曲数可能误判）
       // cout << "[调试] 保留数字5：虽然弯曲数=2，但5的距离（" << shape_distances[4] 
         //    << "）远小于2的距离（" << digit2_dist << "），可能弯曲数误判" << endl;
    }
}

    // 修正3：5被误判为1（弯曲数=0）
    if (best_match_idx == 4 && target_curves == 0) {
       // cout << "[调试] 修正：弯曲数0（5通常1个），改为数字1" << endl;
        best_match_idx = 0;
        min_shape_dist = shape_distances[0];
    }

    // 修正4：强制匹配1（弯曲数=0）
    if (target_curves == 0) {
        double digit1_dist = shape_distances[0];  // 直接取存储的1的距离
        if (digit1_dist < 1.5) {
          //  cout << "[调试] 弯曲数0，强制匹配数字1" << endl;
            best_match_idx = 0;
            min_shape_dist = digit1_dist;
        }
    }

    // 修正5：3被误判为5（距离接近+弯曲数=2）【核心解决3→5问题】
    if (best_match_idx == 4) {  // 当前最佳是5
        double digit3_dist = shape_distances[2];  // 取3的距离（索引2）
        // 距离差<0.05（临界相似）且弯曲数=2（3的典型特征）
        if (abs(digit3_dist - min_shape_dist) < 0.05 && target_curves == 2) {
         //   cout << "[调试] 修正：3与5距离接近（差" << abs(digit3_dist - min_shape_dist) 
         //        << "），弯曲数2→改为3" << endl;
            best_match_idx = 2;  // 修正为3
            min_shape_dist = digit3_dist;  // 同步更新距离
        }
    }

    // 6. 最终结果判断（兜底逻辑）
    if (best_match_idx != -1) {
        // 强制匹配1的场景直接输出
        if (target_curves == 0 && best_match_idx == 0) {
       //     cout << "[调试] 强制匹配生效，输出数字1" << endl;
            return "1";
        }
        // 普通场景：阈值放宽到1.0
        if (min_shape_dist < 1.0) {
            return to_string(best_match_idx + 1);
        } else {
        //    cout << "[调试] 无匹配结果，最小形状距离：" << min_shape_dist << endl;
            return "unknown";
        }
    } else {
       // cout << "[调试] 无匹配结果，无有效索引" << endl;
        return "unknown";
    }
}
};
void TestNode::detectArmorPlates(const Mat& hsv, Mat& result_image) {
  // ========== 抗光照干扰修改：使用自适应颜色分割 ==========
  Mat red_mask = adaptiveColorSegmentation(hsv, "red");

  // 形态学操作，连接相邻区域
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
  morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel);
  //cv::imshow("Result", red_mask);
  // 找轮廓
  vector<vector<Point>> contours;
  findContours(red_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<RotatedRect> light_bars;
  
  // 筛选灯条
  for (const auto& contour : contours) {
     double area = contourArea(contour);
    // if (area < 300) continue;
    RotatedRect light_rect = minAreaRect(contour);
    Size2f rect_size = light_rect.size;
    
    float width = min(rect_size.width, rect_size.height);
    float height = max(rect_size.width, rect_size.height);
    float aspect_ratio = height / width;
    
    // 灯条通常有较大的长宽比
    if (aspect_ratio > 2.0 && height > 5) {
      light_bars.push_back(light_rect);
      
      // 绘制检测到的灯条
      Point2f vertices[4];
      light_rect.points(vertices);
      // for (int i = 0; i < 4; i++) {
      //   line(result_image, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 255), 2);
      // }
    }
  }

  // 配对灯条形成装甲板
  for (size_t i = 0; i < light_bars.size(); i++) {
    for (size_t j = i + 1; j < light_bars.size(); j++) {
      Point2f center1 = light_bars[i].center;
      Point2f center2 = light_bars[j].center;
      
      float distance = norm(center1 - center2);
      float angle_diff = abs(light_bars[i].angle - light_bars[j].angle);

      // 配对条件：距离适中，角度相似
      if (distance > 50 && distance < 300 && angle_diff < 20) {
        // 计算装甲板的四个角点
        Point2f left_center, right_center;
        if (center1.x < center2.x) {
          left_center = center1;
          right_center = center2;
        } else {
          left_center = center2;
          right_center = center1;
        }
        float height1 = max(light_bars[i].size.height, light_bars[i].size.width);
        float height2 = max(light_bars[j].size.height, light_bars[j].size.width);
        
        // 计算装甲板的高度（取两个灯条高度的平均值）
        float avg_height = (height1 + height2) / 2.0f;
        float width = distance;
        
        //cout<<avg_height<<endl;
        // 计算装甲板的四个角点（左下，右下，右上，左上）
        vector<Point2f> armor_points(4);
        armor_points[0] = Point2f(left_center.x - 5, left_center.y + avg_height / 2);  // 左下
        armor_points[1] = Point2f(right_center.x + 5, right_center.y + avg_height / 2); // 右下
        armor_points[2] = Point2f(right_center.x + 5, right_center.y - avg_height / 2); // 右上
        armor_points[3] = Point2f(left_center.x - 5, left_center.y - avg_height / 2);   // 左上
        
        // 计算装甲板中心
        Point2f armor_center(0, 0);
        for (const auto& p : armor_points) {
          armor_center += p;
        }
        armor_center.x /= 4;
        armor_center.y /= 4;
        
        // 检测数字区域（装甲板中间区域）
        Rect number_roi(left_center.x + 15, left_center.y - avg_height / 2 - 20, 
                        right_center.x - left_center.x - 30, avg_height+40);
        // 检测数字区域（装甲板中间区域）
        // Rect number_roi(left_center.x + 10, left_center.y - avg_height / 2 + 10, 
        //                 right_center.x - left_center.x - 20, avg_height - 20);
        
        // 确保ROI在图像范围内
        Rect image_rect(0, 0, result_image.cols, result_image.rows);
        Rect valid_roi = number_roi & image_rect;
        if (valid_roi.area() <= 0) {
            continue; // 如果交集面积为0，跳过
        }
        
        // 提取数字区域进行识别
        Mat number_region = result_image(valid_roi);
        // 确保ROI在图像范围内
        // if (number_roi.x >= 0 && number_roi.y >= 0 && 
        //     number_roi.x + number_roi.width <= result_image.cols &&
        //     number_roi.y + number_roi.height <= result_image.rows) {
          
          // 提取数字区域进行识别
          //Mat number_region = result_image(number_roi);
                  // 检测数字区域（装甲板中间区域）

          // 识别数字
          DigitMatcher matcher;
          string detected_number = matcher.matchDigit(number_region);
          //string detected_number = recognizeNumber(number_region);
          cout<<detected_number<<endl;
          // 只有当识别到有效数字时才认为是装甲板
          if (detected_number != "unknown") {
            // 绘制装甲板
            // for (int k = 0; k < 4; k++) {
            //   line(result_image, armor_points[k], armor_points[(k + 1) % 4], 
            //        Scalar(255, 0, 0), 3);
            // }
            
            for (int k = 0; k < 4; k++) {
              circle(result_image, armor_points[k], 3, Scalar(0,0,255), -1);
              
              // 标注角点序号
              string text = to_string(k + 1);
              putText(result_image, text, 
                      Point(armor_points[k].x + 10, armor_points[k].y - 10),
                      FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 3);
              putText(result_image, text, 
                      Point(armor_points[k].x + 10, armor_points[k].y - 10),
                      FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
            }
            
            // 标注装甲板信息（包含识别到的数字）
            string info = "Armor" + detected_number;
            putText(result_image, info, 
                    Point(armor_center.x - 30, armor_center.y - 20),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
            
            // 添加到检测对象列表
            DetectedObject armor_obj;
            armor_obj.type = "armor";
            armor_obj.corners = armor_points;
            armor_obj.number = detected_number; // 存储识别到的数字
            
            detected_objects_.push_back(armor_obj);
            
            RCLCPP_INFO(this->get_logger(), "Found armor plate with number: %s", 
                        detected_number.c_str());
          }
        //}
      }
    }
  }
}

void TestNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  try {
    // 图像转换
    cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
      cv::Mat image(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));
      cv::Mat bgr_image;
      cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
      cv_ptr = std::make_shared<cv_bridge::CvImage>();
      cv_ptr->header = msg->header;
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = bgr_image;
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    // ========== 抗光照干扰修改：图像预处理 ==========
    Mat processed_image = preprocessImage(image);

    // 性能优化：缩放图像
    if (detection_scale != 1.0) {
      resize(processed_image, processed_image, Size(), detection_scale, detection_scale, INTER_LINEAR);
    }

    // 创建结果图像
    cv::Mat result_image = processed_image.clone();

    // 转换到 HSV 空间
    cv::Mat hsv;
    cv::cvtColor(processed_image, hsv, cv::COLOR_BGR2HSV);

    // 清空点集
    //Point_V.clear();
    detected_objects_.clear();
    // 并行检测不同类型的目标
    detectSpheres(hsv, result_image);
    detectGreenRectangles(hsv, result_image);
    detectArmorPlates(hsv, result_image);

    // 显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    
    msg_object.num_objects = detected_objects_.size();
    for (const auto& target : detected_objects_) {
        referee_pkg::msg::Object obj;
        obj.target_type = target.type;
        
        // 如果是装甲板，在类型中添加数字信息
        if (target.type == "armor" && !target.number.empty()) {
            obj.target_type = "armor_red_" + target.number;
        }
        
        for (const auto& corner : target.corners) {
            geometry_msgs::msg::Point p;
            p.x = corner.x / detection_scale;
            p.y = corner.y / detection_scale;
            p.z = 0.0;
            obj.corners.push_back(p);
        }
        msg_object.objects.push_back(obj);
    }
    Target_pub->publish(msg_object);
    
    // 性能监测
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    RCLCPP_INFO(this->get_logger(), "Published %d targets in %ld ms", 
                msg_object.num_objects, duration.count());

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}
