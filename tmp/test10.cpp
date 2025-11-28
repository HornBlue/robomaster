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

class DigitMatcher {
private:
    vector<string> template_paths = {
        "src/player_pkg/small_num1.png",
        "src/player_pkg/small_num2.png",
        "src/player_pkg/small_num3.png",
        "src/player_pkg/small_num4.png",
        "src/player_pkg/small_num5.png"
    };
    vector<vector<Mat>> multi_scale_templates; 
    // 大幅降低阈值，确保匹配能通过
    double base_threshold = 1.5;       // 基础阈值（原0.9→1.5）
    double threshold_3 = 1.8;          // 3的阈值（原1.2→1.8）
    double threshold_4 = 1.7;          // 4的阈值（原1.1→1.7）

    // 1. 大幅放宽特征条件（允许更大波动）
    struct DigitFeature {
        int min_curves;    // 最小弯曲数（放宽范围）
        int max_curves;    // 最大弯曲数（放宽范围）
        double min_ratio;  // 宽高比下限（降低，允许更宽/更窄）
        double max_ratio;  // 宽高比上限（提高）
        bool need_bar;     // 4的横杠检测从"必须有"改为"优先有"
        bool need_double_arc; // 3的双弧检测从"必须有"改为"优先有"
    };
    vector<DigitFeature> digit_features = {
        {0, 2, 0.05, 0.6, false, false},  // 1：0-2弯（原0-1），宽高比0.05-0.6（更宽松）
        {0, 3, 0.4, 1.1, false, false},   // 2：0-3弯（原1-2），宽高比0.4-1.1
        {0, 4, 0.6, 1.5, false, false},   // 3：取消"必须双弧"，弯数0-4，宽高比0.6-1.5
        {0, 3, 0.5, 1.3, false, false},   // 4：取消"必须横杠"，弯数0-3，宽高比0.5-1.3
        {0, 3, 0.5, 1.3, false, false}    // 5：放宽所有条件
    };

    // 2. 优化预处理（保留更多轮廓信息，避免过度过滤）
    Mat preprocess(const Mat& img) {
        if (img.empty()) return Mat();

        Mat enhanced;
        // 小图像增强更保守（避免模糊数字）
        if (img.rows < 50 || img.cols < 50) {
            bilateralFilter(img, enhanced, 3, 30, 30); // 弱保边去噪（原5→3）
            enhanced.convertTo(enhanced, -1, 1.1, 3);  // 弱对比度增强（原1.2→1.1）
        } else {
            enhanced = img.clone();
        }

        // 去红框更保守（避免误删数字）
        Mat hsv, red_mask1, red_mask2, red_mask, no_red;
        cvtColor(enhanced, hsv, COLOR_BGR2HSV);
        // 缩小红色检测范围，仅去除明显红框
        inRange(hsv, Scalar(0, 100, 70), Scalar(5, 255, 255), red_mask1);
        inRange(hsv, Scalar(175, 100, 70), Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;
        
        // 弱化形态学操作（避免数字被腐蚀）
        Mat kernel_red = getStructuringElement(MORPH_RECT, Size(2,2)); // 原3→2
        erode(red_mask, red_mask, kernel_red);
        dilate(red_mask, red_mask, getStructuringElement(MORPH_RECT, Size(1,1))); // 原2→1
        
        no_red = enhanced.clone();
        no_red.setTo(Scalar(0,0,0), red_mask);

        // 二值化更宽松（保留更多细节）
        Mat gray, binary;
        cvtColor(no_red, gray, COLOR_BGR2GRAY);
        if (gray.rows < 40) {
            // 小图像用更大块大小和偏移，避免细节丢失
            adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, 
                             THRESH_BINARY, 9, 3); // 原7→9，2→3
        } else {
            threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }

        // 形态学操作更弱（避免轮廓被破坏）
        int morph_size = max(1, min(gray.rows, gray.cols) / 80); // 原50→80（更小的核）
        Mat morph_kernel = getStructuringElement(MORPH_ELLIPSE, Size(morph_size, morph_size));
        morphologyEx(binary, binary, MORPH_OPEN, morph_kernel);
        morphologyEx(binary, binary, MORPH_CLOSE, morph_kernel);

        // 白字黑底判断更宽松
        int white_pixels = countNonZero(binary);
        if (white_pixels < binary.total() * 0.1) { // 原0.15→0.1
            binary = 255 - binary;
            cout << "[调试] 反转图像（白字黑底）" << endl;
        }

        return binary;
    }

    // 3. 简化特征检测（从"必须满足"改为"加分项"，避免过滤过严）
    // 3的双弧检测（仅作为参考，不决定候选）
    bool detect3DoubleArc(const Mat& binary_img) {
        vector<vector<Point>> contours;
        findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return false;

        vector<Point> max_contour = contours[0];
        double max_area = contourArea(max_contour);
        if (max_area < 20) return false; // 放宽面积限制（原50→20）

        Rect bbox = boundingRect(max_contour);
        int mid_y = bbox.y + bbox.height / 2;
        int upper_points = 0, lower_points = 0;
        for (const auto& p : max_contour) {
            if (p.y < mid_y) upper_points++;
            else lower_points++;
        }
        return upper_points > 5 && lower_points > 5; // 放宽点数要求（原10→5）
    }

    // 4的横杠检测（仅作为参考）
    bool detect4HorizontalBar(const Mat& binary_img) {
        vector<vector<Point>> contours;
        findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return false;

        vector<Point> max_contour = contours[0];
        Rect bbox = boundingRect(max_contour);
        if (bbox.width < 5 || bbox.height < 5) return false; // 放宽尺寸（原10→5）

        int upper_y = bbox.y + bbox.height / 4; // 扩大中间区域范围
        int lower_y = bbox.y + bbox.height * 3 / 4;
        Mat middle_roi = binary_img(Rect(bbox.x, upper_y, bbox.width, lower_y - upper_y));
        int white_count = countNonZero(middle_roi);
        return white_count > (middle_roi.cols * middle_roi.rows) * 0.2; // 原0.3→0.2
    }

    // 4. 弯曲数计算更宽松（避免漏检）
    int countCurves(const Mat& binary_img) {
        vector<vector<Point>> contours;
        findContours(binary_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return 0;

        vector<Point> max_contour = contours[0];
        double max_area = contourArea(max_contour);
        if (max_area < 20) return 0; // 放宽面积（原50→20）

        vector<int> hull;
        convexHull(max_contour, hull, false);
        vector<Vec4i> defects;
        if (hull.size() > 3) {
            convexityDefects(max_contour, hull, defects);
        }

        int curve_count = 0;
        float adaptive_threshold = max(5.0f, min(20.0f, static_cast<float>(sqrt(max_area)) * 0.2f)); // 降低阈值（原8→5）
        for (auto& d : defects) {
            float depth = d[3] / 256.0;
            if (depth > adaptive_threshold) {
                curve_count++;
            }
        }
        return curve_count; // 不限制最大弯数（原min(curve_count,3)）
    }

    // 5. 模板生成（确保覆盖更多尺度）
    vector<Mat> generateMultiScale(const Mat& template_img, int digit_idx) {
        vector<Mat> scales;
        vector<double> base_scales = {0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4}; // 增加更多尺度
        for (double s : base_scales) {
            Mat scaled;
            resize(template_img, scaled, Size(), s, s, INTER_LINEAR);
            scales.push_back(scaled);
        }
        return scales;
    }

    // 6. 模板加载（确保模板预处理正确）
    bool loadTemplates() {
        multi_scale_templates.clear();
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
            // 模板预处理更保守（保留特征）
            switch(i) {
                case 0: // 1
                    dilate(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,3))); // 弱膨胀
                    break;
                case 1: // 2
                {
                    int white_pixels = countNonZero(temp_pre);
                    if (white_pixels < temp_pre.total() * 0.15) temp_pre = 255 - temp_pre; // 放宽反转条件
                    break;
                }
                case 2: // 3
                    break; // 不做处理，保留原始特征
                case 3: // 4
                {
                    Mat kernel_h = getStructuringElement(MORPH_RECT, Size(2, 1)); // 弱水平膨胀
                    dilate(temp_pre, temp_pre, kernel_h);
                    break;
                }
                case 4: // 5
                    erode(temp_pre, temp_pre, getStructuringElement(MORPH_RECT, Size(1,1)));
                    break;
                default:
                    break;
            }
            
            vector<Mat> scales = generateMultiScale(temp_pre, i);
            multi_scale_templates.push_back(scales);
            cout << "[调试] 模板" << (i+1) << "加载成功，生成" << scales.size() << "个尺度版本" << endl;
        }
        return true;
    }

    Mat resizeMat(const Mat& img, double scale) {
        Mat resized;
        resize(img, resized, Size(), scale, scale, INTER_LINEAR);
        return resized;
    }

public:
    DigitMatcher() {
        if (!loadTemplates()) {
            cerr << "[错误] 模板加载失败，程序退出" << endl;
            exit(-1);
        }
    }

    // 7. 匹配逻辑（确保候选不为空，匹配更宽松）
    string matchDigit(const Mat& input_img) {
        if (input_img.empty()) return "unknown";

        Mat img_pre = preprocess(input_img);
        if (img_pre.empty()) return "unknown";
        imshow("待识别数字预处理后", img_pre);
        waitKey(1);

        // 提取轮廓和基础特征
        vector<vector<Point>> target_contours;
        findContours(img_pre, target_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (target_contours.empty()) {
            cout << "[调试] 无轮廓" << endl;
            return "unknown";
        }
        
        vector<Point> target_contour = target_contours[0];
        double target_area = contourArea(target_contour);
        for (auto& c : target_contours) {
            double area = contourArea(c);
            if (area > target_area) {
                target_contour = c;
                target_area = area;
            }
        }
        if (target_area < 10) { // 大幅降低面积门槛（原30→10）
            cout << "[调试] 轮廓面积过小" << endl;
            return "unknown";
        }

        Rect bbox = boundingRect(target_contour);
        double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
        int target_curves = countCurves(img_pre);
        bool is_3_double_arc = detect3DoubleArc(img_pre);
        bool is_4_has_bar = detect4HorizontalBar(img_pre);

        // 输出详细特征，方便调试
        cout << "[调试] 特征详情 - 面积:" << target_area 
             << " 宽高比:" << aspect_ratio 
             << " 弯曲数:" << target_curves 
             << " 3双弧:" << is_3_double_arc 
             << " 4横杠:" << is_4_has_bar << endl;

        // 8. 特征过滤（从"严格筛选"改为"宽松包含"）
        vector<int> candidate_digits;
        for (int d=0; d<5; d++) {
            DigitFeature& feat = digit_features[d];
            // 基础特征：只要在范围内就保留（不严格卡条件）
            bool base_match = (target_curves >= feat.min_curves && target_curves <= feat.max_curves) &&
                              (aspect_ratio >= feat.min_ratio && aspect_ratio <= feat.max_ratio);
            candidate_digits.push_back(d); // 即使不满足，也先加入候选（最后靠距离筛选）
        }

        // 9. 模板匹配（计算所有数字的距离，按距离排序）
        vector<pair<int, double>> digit_dist; 
        for (int d : candidate_digits) {
            vector<Mat>& scales = multi_scale_templates[d];
            double min_d = 1e9;
            for (Mat& temp : scales) {
                vector<vector<Point>> temp_contours;
                findContours(temp, temp_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                if (temp_contours.empty()) continue;

                vector<Point> temp_contour = temp_contours[0];
                double dist = matchShapes(target_contour, temp_contour, CONTOURS_MATCH_I1, 0);
                if (dist < min_d) min_d = dist;
            }
            digit_dist.push_back({d, min_d});
        }

        // 按距离排序（确保距离最近的优先）
        sort(digit_dist.begin(), digit_dist.end(), 
             [](const pair<int, double>& a, const pair<int, double>& b) {
                 return a.second < b.second;
             });

        // 10. 决策（接受更大的距离，确保有结果）
        string result = "unknown";
        for (auto& pair : digit_dist) {
            int d = pair.first;
            double dist = pair.second;
            double threshold = base_threshold;
            if (d == 2) threshold = threshold_3;
            if (d == 3) threshold = threshold_4;

            // 专属特征加分（有特征则降低阈值要求）
            if (d == 2 && is_3_double_arc) threshold += 0.3;
            if (d == 3 && is_4_has_bar) threshold += 0.3;

            if (dist < threshold) {
                result = to_string(d + 1);
                cout << "[调试] 匹配成功：数字" << result 
                     << "（距离：" << dist << "，阈值：" << threshold << "）" << endl;
                break;
            }
        }

        // 终极兜底：如果所有都不满足，返回距离最近的（即使超过阈值）
        if (result == "unknown" && !digit_dist.empty()) {
            result = to_string(digit_dist[0].first + 1);
            cout << "[调试] 兜底匹配：数字" << result 
                 << "（距离：" << digit_dist[0].second << "，超过阈值）" << endl;
        }

        return result;
    }
};
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
