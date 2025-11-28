#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
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

    RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
  }

  ~TestNode() { cv::destroyWindow("Detection Result"); }
  
 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  //vector<Point2f> Point_V;
  vector<DetectedObject> detected_objects_;  // 替换原来的 Point_V
  // 性能优化参数
  double detection_scale;
  int min_contour_area;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
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
  // 红色检测 - 使用稳定的范围
  Mat mask1, mask2, mask;
  cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
  mask = mask1 | mask2;

  // 适度的形态学操作
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);
  morphologyEx(mask, mask, MORPH_OPEN, kernel);

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
  // 绿色检测范围
  Mat green_mask;
  cv::inRange(hsv, cv::Scalar(75, 100, 80), cv::Scalar(95, 255, 255), green_mask);

  // 形态学操作
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  morphologyEx(green_mask, green_mask, MORPH_CLOSE, kernel);
  morphologyEx(green_mask, green_mask, MORPH_OPEN, kernel);

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

string TestNode::recognizeNumber(const Mat& number_roi) {
    if (number_roi.empty()) return "unknown";
    
    // 转换为灰度图
    Mat gray;
    cvtColor(number_roi, gray, COLOR_BGR2GRAY);
    
    // 二值化处理
    Mat binary;
    threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    
    // 形态学操作，去除噪声
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(binary, binary, MORPH_OPEN, kernel);
    
    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) return "unknown";
    
    // 找到最大的轮廓（应该是数字）
    double max_area = 0;
    int max_index = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_index = i;
        }
    }
    
    if (max_index == -1) return "unknown";
    
    // 获取数字的边界框
    Rect num_rect = boundingRect(contours[max_index]);
    
    // 提取数字区域
    Mat digit_roi = binary(num_rect);
    
    // 调整大小以便于分析
    Mat resized_digit;
    resize(digit_roi, resized_digit, Size(40, 60));
    
    // 计算数字区域的宽高比和面积比例
    double aspect_ratio = static_cast<double>(num_rect.width) / num_rect.height;
    double area_ratio = max_area / (number_roi.rows * number_roi.cols);
    
    // 改进的数字识别逻辑 - 使用多种特征组合
    // 计算数字的水平和垂直投影
    Mat horizontal_proj, vertical_proj;
    reduce(resized_digit, horizontal_proj, 1, REDUCE_AVG);
    reduce(resized_digit, vertical_proj, 0, REDUCE_AVG);
    
    // 计算投影特征
    int h_peaks = 0, v_peaks = 0;
    double h_avg = mean(horizontal_proj)[0];
    double v_avg = mean(vertical_proj)[0];
    
    // 计算水平投影的峰值
    for (int i = 1; i < horizontal_proj.rows - 1; i++) {
        if (horizontal_proj.at<uchar>(i, 0) > h_avg * 1.5 && 
            horizontal_proj.at<uchar>(i, 0) > horizontal_proj.at<uchar>(i-1, 0) &&
            horizontal_proj.at<uchar>(i, 0) > horizontal_proj.at<uchar>(i+1, 0)) {
            h_peaks++;
        }
    }
    
    // 计算垂直投影的峰值
    for (int i = 1; i < vertical_proj.cols - 1; i++) {
        if (vertical_proj.at<uchar>(0, i) > v_avg * 1.5 && 
            vertical_proj.at<uchar>(0, i) > vertical_proj.at<uchar>(0, i-1) &&
            vertical_proj.at<uchar>(0, i) > vertical_proj.at<uchar>(0, i+1)) {
            v_peaks++;
        }
    }
    
    // 计算数字的对称性
    Mat flipped;
    flip(resized_digit, flipped, 1);
    double symmetry = compareHist(resized_digit, flipped, HISTCMP_CORREL);
    
    // 计算数字的质心位置
    Moments m = moments(resized_digit);
    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;
    double centroid_x_ratio = cx / resized_digit.cols;
    double centroid_y_ratio = cy / resized_digit.rows;
    
    // 基于多种特征的数字识别
    // 数字1: 细长，垂直投影峰值少，质心偏上
    if (aspect_ratio < 0.6 && v_peaks <= 2 && centroid_x_ratio > 0.4 && centroid_x_ratio < 0.6) {
        return "1";
    }
    // 数字2: 中等宽高比，有弯曲，水平投影有2-3个峰值
    else if (aspect_ratio > 0.6 && aspect_ratio < 0.9 && h_peaks >= 2 && h_peaks <= 3 && 
             centroid_y_ratio < 0.6) {
        return "2";
    }
    // 数字3: 类似2但更对称，质心居中
    else if (aspect_ratio > 0.7 && aspect_ratio < 1.0 && symmetry > 0.7 && 
             centroid_x_ratio > 0.4 && centroid_x_ratio < 0.6) {
        return "3";
    }
    // 数字4: 通常有直角，垂直投影有多个峰值
    else if (aspect_ratio > 0.7 && aspect_ratio < 1.1 && v_peaks >= 3 && 
             centroid_x_ratio < 0.5) {
        return "4";
    }
    // 数字5: 上部有横线，下部有弯曲，水平投影有多个峰值
    else if (aspect_ratio > 0.7 && aspect_ratio < 1.0 && h_peaks >= 3 && 
             centroid_y_ratio > 0.4) {
        return "5";
    }
    
    return "unknown";
}

void TestNode::detectArmorPlates(const Mat& hsv, Mat& result_image) {
  // 检测红色灯条 - 装甲板的灯条通常是红色
  Mat red_mask1, red_mask2, red_mask;
  cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), red_mask1);
  cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), red_mask2);
  red_mask = red_mask1 | red_mask2;

  // 形态学操作，连接相邻区域
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
  morphologyEx(red_mask, red_mask, MORPH_OPEN, kernel);
  cv::imshow("Result", red_mask);
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
    if (aspect_ratio > 2.0 && height > 20) {
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
        Rect number_roi(left_center.x + 10, left_center.y - avg_height / 2 - 10, 
                        right_center.x - left_center.x - 20, avg_height+20);
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
          string detected_number = recognizeNumber(number_region);
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
            string info = "Armor " ;//+ detected_number;
            putText(result_image, info, 
                    Point(armor_center.x - 30, armor_center.y - 20),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
            
            // 添加到检测对象列表
            DetectedObject armor_obj;
            armor_obj.type = "armor";
            armor_obj.corners = armor_points;
            //armor_obj.number = detected_number; // 存储识别到的数字
            
            detected_objects_.push_back(armor_obj);
            
            //RCLCPP_INFO(this->get_logger(), "Found armor plate with number: %s", 
            //            detected_number.c_str());
          //}
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

    // 性能优化：缩放图像
    Mat processed_image;
    if (detection_scale != 1.0) {
      resize(image, processed_image, Size(), detection_scale, detection_scale, INTER_LINEAR);
    } else {
      processed_image = image;
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
    
    RCLCPP_INFO(this->get_logger(), "Published %d targets in %ld ms", 
                msg_object.num_objects, duration.count());

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}