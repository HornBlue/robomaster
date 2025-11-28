[file name]: test7.cpp
[file content begin]
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

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

    // 性能优化参数
    detection_scale = 0.5;  // 图像缩放因子，提高处理速度
    min_contour_area = 100; // 最小轮廓面积，减少计算量

    // 光照自适应参数
    adaptive_brightness_threshold = 100; // 亮度阈值，用于判断光照条件
    use_adaptive_color_ranges = true;    // 启用自适应颜色范围
    
    // 历史帧缓存，用于稳定性滤波
    frame_buffer_size = 5;
    
    RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
  }

  ~TestNode() { cv::destroyWindow("Detection Result"); }
  
 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  
  // 光照自适应预处理
  Mat adaptivePreprocessing(const Mat& image);
  
  // 自适应颜色范围计算
  void calculateAdaptiveColorRanges(const Mat& hsv);
  
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
  
  // 稳定性滤波 - 去除抖动检测
  vector<DetectedObject> stabilityFilter(const vector<DetectedObject>& current_objects);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  //vector<Point2f> Point_V;
  vector<DetectedObject> detected_objects_;  // 替换原来的 Point_V
  
  // 历史检测结果缓存
  vector<vector<DetectedObject>> object_history_;
  
  // 性能优化参数
  double detection_scale;
  int min_contour_area;
  
  // 光照自适应参数
  int adaptive_brightness_threshold;
  bool use_adaptive_color_ranges;
  int frame_buffer_size;
  
  // 自适应颜色范围
  Scalar red_lower1, red_upper1, red_lower2, red_upper2;
  Scalar green_lower, green_upper;
  Scalar blue_lower, blue_upper;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// 光照自适应预处理
Mat TestNode::adaptivePreprocessing(const Mat& image) {
    Mat processed = image.clone();
    
    // 计算图像平均亮度
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    Scalar mean_brightness = mean(hsv_channels[2]);
    
    // 根据亮度调整处理策略
    if (mean_brightness[0] < adaptive_brightness_threshold) {
        // 低光照条件 - 增强亮度和对比度
        processed.convertTo(processed, -1, 1.3, 20);
        
        // 轻度高斯模糊去噪
        GaussianBlur(processed, processed, Size(3, 3), 0.5);
        
        RCLCPP_DEBUG(this->get_logger(), "Low light condition detected, applying enhancement");
    } else {
        // 高光照条件 - 降低对比度，防止过曝
        processed.convertTo(processed, -1, 0.9, -10);
        
        RCLCPP_DEBUG(this->get_logger(), "High light condition detected, applying reduction");
    }
    
    // 直方图均衡化（只在HSV的V通道进行）
    Mat hsv_processed;
    cvtColor(processed, hsv_processed, COLOR_BGR2HSV);
    vector<Mat> hsv_processed_channels;
    split(hsv_processed, hsv_processed_channels);
    equalizeHist(hsv_processed_channels[2], hsv_processed_channels[2]);
    merge(hsv_processed_channels, hsv_processed);
    cvtColor(hsv_processed, processed, COLOR_HSV2BGR);
    
    return processed;
}

// 自适应颜色范围计算
void TestNode::calculateAdaptiveColorRanges(const Mat& hsv) {
    // 计算图像整体亮度
    vector<Mat> channels;
    split(hsv, channels);
    Scalar mean_brightness = mean(channels[2]);
    
    // 根据亮度动态调整颜色范围
    float brightness_factor = mean_brightness[0] / 128.0; // 归一化到1附近
    
    if (use_adaptive_color_ranges) {
        // 红色范围 - 自适应调整
        red_lower1 = Scalar(0, max(50, int(120 * brightness_factor)), max(40, int(70 * brightness_factor)));
        red_upper1 = Scalar(15, 255, 255);
        red_lower2 = Scalar(165, max(50, int(120 * brightness_factor)), max(40, int(70 * brightness_factor)));
        red_upper2 = Scalar(180, 255, 255);
        
        // 绿色范围 - 自适应调整
        green_lower = Scalar(35, max(60, int(100 * brightness_factor)), max(50, int(80 * brightness_factor)));
        green_upper = Scalar(85, 255, 255);
        
        // 蓝色范围 - 自适应调整（如果需要）
        blue_lower = Scalar(100, max(60, int(100 * brightness_factor)), max(50, int(80 * brightness_factor)));
        blue_upper = Scalar(130, 255, 255);
    } else {
        // 固定颜色范围
        red_lower1 = Scalar(0, 120, 70);
        red_upper1 = Scalar(10, 255, 255);
        red_lower2 = Scalar(170, 120, 70);
        red_upper2 = Scalar(180, 255, 255);
        
        green_lower = Scalar(75, 100, 80);
        green_upper = Scalar(95, 255, 255);
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

// 稳定性滤波
vector<DetectedObject> TestNode::stabilityFilter(const vector<DetectedObject>& current_objects) {
    vector<DetectedObject> filtered_objects;
    
    // 将当前检测结果加入历史
    object_history_.push_back(current_objects);
    
    // 保持历史帧数量不超过设定值
    if (object_history_.size() > frame_buffer_size) {
        object_history_.erase(object_history_.begin());
    }
    
    // 如果历史帧不足，直接返回当前结果
    if (object_history_.size() < 3) {
        return current_objects;
    }
    
    // 对每个当前检测的对象，检查其在历史帧中的稳定性
    for (const auto& current_obj : current_objects) {
        int stable_count = 0;
        
        // 检查历史帧中是否存在相似对象
        for (const auto& history_frame : object_history_) {
            for (const auto& history_obj : history_frame) {
                if (history_obj.type == current_obj.type) {
                    // 简单的位置稳定性检查
                    if (history_obj.corners.size() == current_obj.corners.size()) {
                        double total_distance = 0;
                        for (size_t i = 0; i < history_obj.corners.size(); i++) {
                            total_distance += norm(history_obj.corners[i] - current_obj.corners[i]);
                        }
                        double avg_distance = total_distance / history_obj.corners.size();
                        
                        // 如果平均距离小于阈值，认为是稳定检测
                        if (avg_distance < 20.0) {
                            stable_count++;
                            break;
                        }
                    }
                }
            }
        }
        
        // 如果对象在多帧中稳定出现，则保留
        double stability_ratio = static_cast<double>(stable_count) / object_history_.size();
        if (stability_ratio > 0.6) { // 60%以上的帧中稳定出现
            filtered_objects.push_back(current_obj);
            RCLCPP_DEBUG(this->get_logger(), "Object type %s passed stability filter (ratio: %.2f)", 
                        current_obj.type.c_str(), stability_ratio);
        }
    }
    
    return filtered_objects;
}
    
void TestNode::detectSpheres(const Mat& hsv, Mat& result_image) {
  // 使用自适应颜色范围进行红色检测
  Mat mask1, mask2, mask;
  cv::inRange(hsv, red_lower1, red_upper1, mask1);
  cv::inRange(hsv, red_lower2, red_upper2, mask2);
  mask = mask1 | mask2;

  // 自适应形态学操作 - 根据图像大小调整
  int morph_size = max(3, min(7, result_image.rows / 150));
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size));
  
  // 先闭运算填充内部空洞，再开运算去除噪声
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);
  morphologyEx(mask, mask, MORPH_OPEN, kernel);

  // 找轮廓
  vector<vector<Point>> contours;
  findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    
    // 自适应面积阈值
    double min_area = max(100.0, result_image.total() * 0.0005);
    double max_area = result_image.total() * 0.05;
    
    if (area < min_area || area > max_area) continue;

    // 计算最小外接圆
    Point2f center;
    float radius = 0;
    minEnclosingCircle(contours[i], center, radius);

    // 计算圆形度
    double perimeter = arcLength(contours[i], true);
    double circularity = 0;
    if (perimeter > 0) {
        circularity = 4 * CV_PI * area / (perimeter * perimeter);
    }

    // 自适应圆形度阈值
    double min_circularity = 0.65;
    double min_radius = max(8.0, result_image.rows * 0.01);
    double max_radius = result_image.rows * 0.2;

    if (circularity > min_circularity && radius > min_radius && radius < max_radius) {
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
  // 使用自适应颜色范围进行绿色检测
  Mat green_mask;
  cv::inRange(hsv, green_lower, green_upper, green_mask);

  // 自适应形态学操作
  int morph_size = max(3, min(7, result_image.rows / 150));
  Mat kernel = getStructuringElement(MORPH_RECT, Size(morph_size, morph_size));
  morphologyEx(green_mask, green_mask, MORPH_CLOSE, kernel);
  morphologyEx(green_mask, green_mask, MORPH_OPEN, kernel);

  // 找轮廓
  vector<vector<Point>> contours;
  findContours(green_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = contourArea(contour);
    
    // 自适应面积阈值
    double min_area = max(200.0, result_image.total() * 0.001);
    double max_area = result_image.total() * 0.1;
    
    if (area < min_area || area > max_area) continue;

    // 多边形近似
    vector<Point> approx;
    double epsilon = 0.02 * arcLength(contour, true);
    approxPolyDP(contour, approx, epsilon, true);

    // 检查是否为四边形
    if (approx.size() == 4) {
      Rect rect = boundingRect(approx);
      double aspect_ratio = static_cast<double>(rect.width) / rect.height;
      
      // 筛选合理的矩形 - 放宽宽高比限制
      if (aspect_ratio > 0.3 && aspect_ratio < 3.0) {
        // 计算矩形度
        double rect_area = rect.width * rect.height;
        double rect_ratio = area / rect_area;
        
        // 要求矩形度足够高
        if (rect_ratio > 0.7) {
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
            
            // 标注角点序号
            string text = to_string(i + 1);
            putText(result_image, text, 
                    Point(sorted_points[i].x + 12, sorted_points[i].y - 12),
                    FONT_HERSHEY_SIMPLEX, 0.7, point_colors[i], 2);
          }
          DetectedObject rectangle_obj;
          rectangle_obj.type = "rect";
          rectangle_obj.corners = sorted_points;
          
          detected_objects_.push_back(rectangle_obj);
          
          // 标注矩形信息
          string info = "Green Rect";
          putText(result_image, info, Point(rect.x, rect.y - 10),
                  FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);

          RCLCPP_INFO(this->get_logger(), "Found green rectangle");
        }
      }
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

    // 改进的预处理函数 - 针对光照变化优化
    Mat preprocess(const Mat& img) {
        if (img.empty()) return Mat();

        // 1. 光照自适应增强
        Mat enhanced;
        if (img.rows < 50 || img.cols < 50) {  // 小图像使用更强的增强
            // 高斯模糊去噪
            GaussianBlur(img, enhanced, Size(3, 3), 0);
            
            // 对比度增强
            enhanced.convertTo(enhanced, -1, 1.5, 10);
            
            // 直方图均衡化
            vector<Mat> channels;
            split(enhanced, channels);
            for (int i = 0; i < channels.size(); i++) {
                equalizeHist(channels[i], channels[i]);
            }
            merge(channels, enhanced);
        } else {
            enhanced = img.clone();
        }

        // 2. 自适应去红框 - 增强红色检测范围
        Mat hsv, red_mask1, red_mask2, red_mask, no_red;
        cvtColor(enhanced, hsv, COLOR_BGR2HSV);
        
        // 使用更宽泛的红色范围以适应光照变化
        inRange(hsv, Scalar(0, 40, 20), Scalar(20, 255, 255), red_mask1);
        inRange(hsv, Scalar(150, 40, 20), Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;
        
        // 自适应形态学操作
        int morph_size = max(2, min(5, img.rows / 60));
        Mat kernel_red = getStructuringElement(MORPH_RECT, Size(morph_size, morph_size));
        dilate(red_mask, red_mask, kernel_red);
        erode(red_mask, red_mask, getStructuringElement(MORPH_RECT, Size(morph_size-1, morph_size-1)));
        
        no_red = enhanced.clone();
        no_red.setTo(Scalar(0,0,0), red_mask);

        // 3. 多尺度二值化 - 适应不同光照条件
        Mat gray, binary;
        cvtColor(no_red, gray, COLOR_BGR2GRAY);
        
        // 根据图像特性选择二值化方法
        if (gray.rows < 40 || gray.cols < 40) {
            // 小图像使用自适应阈值
            adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, 
                             THRESH_BINARY, 11, 5); // 增加C值以适应光照变化
        } else {
            // 正常图像使用Otsu阈值或自适应阈值
            Scalar mean_val = mean(gray);
            if (mean_val[0] < 50 || mean_val[0] > 200) {
                // 极端光照条件使用自适应阈值
                adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, 
                                 THRESH_BINARY, 15, 8);
            } else {
                // 正常光照使用Otsu
                threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
            }
        }

        // 4. 自适应形态学操作
        int morph_size_binary = max(1, min(gray.rows, gray.cols) / 40);
        Mat morph_kernel = getStructuringElement(MORPH_RECT, 
                                               Size(morph_size_binary, morph_size_binary));
        
        // 根据图像大小调整形态学操作强度
        if (gray.rows < 40) {
            morphologyEx(binary, binary, MORPH_CLOSE, morph_kernel);
        } else {
            morphologyEx(binary, binary, MORPH_CLOSE, morph_kernel);
            morphologyEx(binary, binary, MORPH_OPEN, morph_kernel);
        }

        // 5. 自适应背景判断
        int white_pixels = countNonZero(binary);
        double white_ratio = static_cast<double>(white_pixels) / binary.total();
        
        // 动态判断是否需要反转
        if (white_ratio < 0.15 || white_ratio > 0.85) {
            binary = 255 - binary;
            cout << "[调试] 预处理：自适应反转（白字黑底），白像素比例：" << white_ratio << endl;
        }

        return binary;
    }

    // 改进的弯曲数量检测 - 更稳定的特征提取
    int countCurves(const Mat& binary_img) {
        vector<vector<Point>> contours;
        findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return 0;

        // 找到最大轮廓
        vector<Point> max_contour = contours[0];
        double max_area = contourArea(max_contour);
        for (auto& c : contours) {
            double area = contourArea(c);
            if (area > max_area) {
                max_contour = c;
                max_area = area;
            }
        }

        // 过滤太小轮廓
        if (max_area < 30) return 0;

        vector<int> hull;
        convexHull(max_contour, hull, false);
        
        // 计算凸包缺陷
        vector<Vec4i> defects;
        if (hull.size() > 3) {
            convexityDefects(max_contour, hull, defects);
        }

        int curve_count = 0;
        for (auto& d : defects) {
            float depth = d[3] / 256.0;
            // 自适应深度阈值 - 根据轮廓大小调整
            float adaptive_threshold = max(8.0f, min(25.0f, static_cast<float>(sqrt(max_area)) * 0.3f));
            if (depth > adaptive_threshold) {
                curve_count++;
            }
        }

        // 特殊处理：数字1通常没有弯曲，但可能被误检
        if (curve_count > 3) curve_count = 3; // 最多3个弯曲
        
        return curve_count;
    }

    // 多尺度模板匹配
    vector<Mat> generateMultiScaleTemplates(const Mat& template_img) {
        vector<Mat> multi_scale_templates;
        
        // 原始尺寸
        multi_scale_templates.push_back(template_img.clone());
        
        // 生成不同尺度的模板
        vector<double> scales = {0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3};
        for (double scale : scales) {
            Mat scaled;
            resize(template_img, scaled, Size(), scale, scale, INTER_LINEAR);
            multi_scale_templates.push_back(scaled);
        }
        
        return multi_scale_templates;
    }

    // 加载模板并生成多尺度版本
    bool loadTemplates() {
        templates.clear();
        char cwd[1024];
        getcwd(cwd, sizeof(cwd));
        cout << "[调试] 当前工作目录：" << cwd << endl;

        for (int i=0; i<template_paths.size(); i++) {
            const string& path = template_paths[i];
            Mat temp = imread(path);
            if (temp.empty()) {
                cout << "[错误] 模板加载失败：" << path << endl;
                return false;
            }
            
            Mat temp_pre = preprocess(temp);
            
            // 数字特定的形态学优化
            switch(i) {
                case 0: // 数字1 - 强化竖线
                    dilate(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,5)));  
                    erode(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(3,1)));  
                    break;
                case 1: // 数字2
                    {
                        int white_pixels = countNonZero(temp_pre);
                        if (white_pixels < temp_pre.total() * 0.2) {
                            temp_pre = 255 - temp_pre;
                        }
                    }
                    break;
                case 2: // 数字3 - 强化弯曲特征
                    dilate(temp_pre, temp_pre, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));  
                    break;
                case 4: // 数字5 - 强化横杠
                    erode(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,1)));
                    break;
            }
            
            // 生成多尺度模板
            vector<Mat> multi_scale = generateMultiScaleTemplates(temp_pre);
            templates.insert(templates.end(), multi_scale.begin(), multi_scale.end());
            
            cout << "[调试] 模板" << (i+1) << "加载成功，生成" << multi_scale.size() 
                 << "个尺度版本" << endl;
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

    // 改进的匹配函数 - 光照自适应
    string matchDigit(const Mat& input_img) {
        if (input_img.empty()) return "unknown";

        // 1. 增强预处理
        Mat img_pre = preprocess(input_img);
        if (img_pre.empty()) return "unknown";
        
        // 显示预处理结果用于调试
        imshow("待识别数字预处理后", img_pre);
        waitKey(1);

        // 2. 提取主轮廓
        vector<vector<Point>> target_contours;
        findContours(img_pre, target_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (target_contours.empty()) return "unknown";
        
        // 选择最大轮廓
        vector<Point> target_contour = target_contours[0];
        double target_area = contourArea(target_contour);
        for (auto& c : target_contours) {
            double area = contourArea(c);
            if (area > target_area) {
                target_contour = c;
                target_area = area;
            }
        }

        // 过滤太小轮廓
        if (target_area < 25) return "unknown";

        // 3. 计算综合特征
        int target_curves = countCurves(img_pre);
        
        // 计算宽高比特征
        Rect bbox = boundingRect(target_contour);
        double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
        
        cout << "[调试] 待识别数字 - 弯曲数:" << target_curves 
             << " 宽高比:" << aspect_ratio 
             << " 面积:" << target_area << endl;

        // 4. 多尺度形状匹配
        double min_shape_dist = 1e9;
        int best_match_idx = -1;
        vector<double> shape_distances(templates.size(), 1e9);

        for (int i=0; i<templates.size(); i++) {
            const Mat& temp = templates[i];
            if (temp.empty()) continue;

            vector<vector<Point>> temp_contours;
            findContours(temp, temp_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            if (temp_contours.empty()) continue;
            
            vector<Point> temp_contour = temp_contours[0];
            double dist = matchShapes(target_contour, temp_contour, CONTOURS_MATCH_I1, 0);
            shape_distances[i] = dist;

            // 计算当前模板对应的数字索引（考虑多尺度）
            int digit_idx = i % 5; // 5个数字，每个有多个尺度
            
            // 根据特征调整距离
            double adjusted_dist = dist;
            
            // 特征权重调整 - 考虑光照条件
            if (digit_idx == 0 && target_curves >= 2) { // 数字1不应该有多个弯曲
                adjusted_dist += 0.5;
            }
            if (digit_idx == 4 && target_curves == 0) { // 数字5应该有弯曲
                adjusted_dist += 0.3;
            }
            if (digit_idx == 2 && target_curves < 2) { // 数字3应该有2个弯曲
                adjusted_dist += 0.3;
            }

            // 宽高比特征调整
            if (digit_idx == 0 && aspect_ratio < 0.3) { // 数字1通常较瘦
                adjusted_dist -= 0.2;
            }

            if (adjusted_dist < min_shape_dist) {
                min_shape_dist = adjusted_dist;
                best_match_idx = digit_idx;
            }
        }

        // 5. 综合决策
        string result = "unknown";
        
        // 基于特征的综合判断 - 放宽阈值以适应光照变化
        if (min_shape_dist < 1.2) { // 放宽阈值
            result = to_string(best_match_idx + 1);
            
            // 后处理验证
            if (best_match_idx == 0 && target_curves >= 2) {
                // 数字1被误判，优先选择数字2
                if (shape_distances[1 % 5] < 1.2) {
                    result = "2";
                    cout << "[调试] 特征修正：1→2（弯曲数过多）" << endl;
                }
            }
            else if (best_match_idx == 4 && target_curves == 0) {
                // 数字5被误判，优先选择数字1
                if (shape_distances[0] < 1.2) {
                    result = "1";
                    cout << "[调试] 特征修正：5→1（无弯曲）" << endl;
                }
            }
            else if (best_match_idx == 2 && target_curves == 1) {
                // 数字3需要至少2个弯曲
                if (shape_distances[4] < min_shape_dist + 0.15) {
                    result = "5";
                    cout << "[调试] 特征修正：3→5（弯曲数不足）" << endl;
                }
            }
            
            cout << "[调试] 匹配结果：数字" << result << "，距离：" << min_shape_dist << endl;
        } else {
            cout << "[调试] 无匹配结果，最小距离：" << min_shape_dist << endl;
        }

        return result;
    }
};
void TestNode::detectArmorPlates(const Mat& hsv, Mat& result_image) {
  // 使用自适应颜色范围检测红色灯条
  Mat red_mask1, red_mask2, red_mask;
  cv::inRange(hsv, red_lower1, red_upper1, red_mask1);
  cv::inRange(hsv, red_lower2, red_upper2, red_mask2);
  red_mask = red_mask1 | red_mask2;

  // 自适应形态学操作
  int morph_size = max(2, min(5, result_image.rows / 200));
  Mat kernel = getStructuringElement(MORPH_RECT, Size(morph_size, morph_size));
  morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
  morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel);

  // 找轮廓
  vector<vector<Point>> contours;
  findContours(red_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<RotatedRect> light_bars;
  
  // 筛选灯条 - 使用自适应阈值
  for (const auto& contour : contours) {
     double area = contourArea(contour);
     
     // 自适应面积阈值
     double min_area = max(50.0, result_image.total() * 0.0003);
     double max_area = result_image.total() * 0.02;
     
     if (area < min_area || area > max_area) continue;
     
    RotatedRect light_rect = minAreaRect(contour);
    Size2f rect_size = light_rect.size;
    
    float width = min(rect_size.width, rect_size.height);
    float height = max(rect_size.width, rect_size.height);
    float aspect_ratio = height / width;
    
    // 灯条通常有较大的长宽比 - 放宽条件以适应不同角度
    if (aspect_ratio > 1.8 && height > 10) {
      light_bars.push_back(light_rect);
    }
  }

  // 配对灯条形成装甲板 - 使用更宽松的配对条件
  for (size_t i = 0; i < light_bars.size(); i++) {
    for (size_t j = i + 1; j < light_bars.size(); j++) {
      Point2f center1 = light_bars[i].center;
      Point2f center2 = light_bars[j].center;
      
      float distance = norm(center1 - center2);
      float angle_diff = abs(light_bars[i].angle - light_bars[j].angle);
      
      // 角度差归一化
      if (angle_diff > 90) angle_diff = 180 - angle_diff;

      // 放宽配对条件以适应不同光照和角度
      float min_distance = max(30.0f, result_image.rows * 0.05f);
      float max_distance = result_image.rows * 0.3f;
      
      if (distance > min_distance && distance < max_distance && angle_diff < 25) {
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
        
        // 计算装甲板的四个角点（左下，右下，右上，左上）
        vector<Point2f> armor_points(4);
        float width_extension = max(5.0f, avg_height * 0.2f); // 自适应宽度扩展
        float height_extension = avg_height * 0.6f; // 高度扩展
        
        armor_points[0] = Point2f(left_center.x - width_extension, left_center.y + height_extension);  // 左下
        armor_points[1] = Point2f(right_center.x + width_extension, right_center.y + height_extension); // 右下
        armor_points[2] = Point2f(right_center.x + width_extension, right_center.y - height_extension); // 右上
        armor_points[3] = Point2f(left_center.x - width_extension, left_center.y - height_extension);   // 左上
        
        // 计算装甲板中心
        Point2f armor_center(0, 0);
        for (const auto& p : armor_points) {
          armor_center += p;
        }
        armor_center.x /= 4;
        armor_center.y /= 4;
        
        // 改进的数字区域提取 - 自适应ROI大小
        float roi_width = max(25.0f, distance * 0.7f);  // 最小宽度25像素
        float roi_height = max(30.0f, avg_height * 1.4f); // 最小高度30像素
        
        Rect number_roi(armor_center.x - roi_width/2, 
                       armor_center.y - roi_height/2,
                       roi_width, roi_height);
        
        // 确保ROI在图像范围内
        Rect image_rect(0, 0, result_image.cols, result_image.rows);
        Rect valid_roi = number_roi & image_rect;
        
        if (valid_roi.area() > 150) { // 确保ROI足够大
            // 提取数字区域进行识别
            Mat number_region = result_image(valid_roi);
            
            // 识别数字
            DigitMatcher matcher;
            string detected_number = matcher.matchDigit(number_region);
            
            cout << "[装甲板检测] 识别到数字: " << detected_number << endl;
            
            // 只有当识别到有效数字时才认为是装甲板
            if (detected_number != "unknown") {
                // 绘制装甲板角点
                for (int k = 0; k < 4; k++) {
                  circle(result_image, armor_points[k], 4, Scalar(0,0,255), -1);
                  
                  // 标注角点序号
                  string text = to_string(k + 1);
                  putText(result_image, text, 
                          Point(armor_points[k].x + 10, armor_points[k].y - 10),
                          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 3);
                  putText(result_image, text, 
                          Point(armor_points[k].x + 10, armor_points[k].y - 10),
                          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
                }
                
                // 绘制装甲板边框
                for (int k = 0; k < 4; k++) {
                  line(result_image, armor_points[k], armor_points[(k + 1) % 4], 
                       Scalar(255, 255, 0), 2);
                }
                
                // 标注装甲板信息（包含识别到的数字）
                string info = "Armor " + detected_number;
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
        }
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

    // 光照自适应预处理
    Mat processed_image = adaptivePreprocessing(image);

    // 性能优化：缩放图像
    if (detection_scale != 1.0) {
      resize(processed_image, processed_image, Size(), detection_scale, detection_scale, INTER_LINEAR);
    }

    // 创建结果图像
    cv::Mat result_image = processed_image.clone();

    // 转换到 HSV 空间
    cv::Mat hsv;
    cv::cvtColor(processed_image, hsv, cv::COLOR_BGR2HSV);

    // 计算自适应颜色范围
    calculateAdaptiveColorRanges(hsv);
    
    // 清空检测结果
    detected_objects_.clear();
    
    // 检测不同类型的目标
    detectSpheres(hsv, result_image);
    detectGreenRectangles(hsv, result_image);
    detectArmorPlates(hsv, result_image);
    
    // 应用稳定性滤波
    vector<DetectedObject> filtered_objects = stabilityFilter(detected_objects_);

    // 显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    
    msg_object.num_objects = filtered_objects.size();
    for (const auto& target : filtered_objects) {
        referee_pkg::msg::Object obj;
        obj.target_type = target.type;
        
        // 如果是装甲板，在类型中添加数字信息
        if (target.type == "armor" && !target.number.empty()) {
            obj.target_type = "armor_" + target.number;
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
    
    RCLCPP_INFO(this->get_logger(), "Published %d filtered targets in %ld ms", 
                msg_object.num_objects, duration.count());

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}
[file content end]