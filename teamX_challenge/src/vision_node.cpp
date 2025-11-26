#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include "teamX_challenge/msg/target_array.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

enum TargetType { CIRCLE, RECTANGLE, ARMOR };
struct Target { int color_id; TargetType type; Point2f center; float area; Scalar draw_color; };
struct ColorParams { int id; Scalar lower1, upper1, lower2, upper2; };

class VisionNode : public Node {
public:
    VisionNode() : Node("vision_node") {
        this->declare_parameter<string>("save_path", "");
        save_path_ = this->get_parameter("save_path").as_string();
        this->declare_parameter<int>("kernel_size", 5);
        kernel_size_ = this->get_parameter("kernel_size").as_int();
        // ... 声明并获取其他参数 ...
        
        // 解析颜色参数
        parse_color_parameters();

        targets_pub_ = this->create_publisher<teamX_challenge::msg::TargetArray>("/vision/targets", 10);
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, bind(&VisionNode::image_callback, this, placeholders::_1)
        );
        namedWindow("识别结果", WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "视觉节点已启动。");
    }
private:
    string save_path_; int kernel_size_;
    vector<ColorParams> color_params_list_;
    Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    Publisher<teamX_challenge::msg::TargetArray>::SharedPtr targets_pub_;

    void parse_color_parameters() {
        auto colors_json = this->get_parameter("colors").as_json_object();
        for (const auto& color_pair : colors_json) {
            ColorParams cp;
            const auto& json = color_pair.second.as_json_object();
            cp.id = json["id"].as_int();
            cp.lower1 = Scalar(json["hue_low1"].as_int(), json["sat_low"].as_int(), json["val_low"].as_int());
            cp.upper1 = Scalar(json["hue_high1"].as_int(), json["sat_high"].as_int(), json["val_high"].as_int());
            cp.lower2 = Scalar(json["hue_low2"].as_int(), json["sat_low"].as_int(), json["val_low"].as_int());
            cp.upper2 = Scalar(json["hue_high2"].as_int(), json["sat_high"].as_int(), json["val_high"].as_int());
            color_params_list_.push_back(cp);
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            Mat frame = cv_ptr->image, result_frame = frame.clone();
            vector<Target> all_targets;

            for (const auto& cp : color_params_list_) {
                Mat hsv, mask, mask1, mask2;
                cvtColor(frame, hsv, COLOR_BGR2HSV);
                inRange(hsv, cp.lower1, cp.upper1, mask1);
                inRange(hsv, cp.lower2, cp.upper2, mask2);
                mask = mask1 | mask2;

                Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size_, kernel_size_));
                morphologyEx(mask, mask, MORPH_CLOSE, kernel);
                morphologyEx(mask, mask, MORPH_OPEN, kernel);

                vector<vector<Point>> contours;
                findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                
                for (size_t i = 0; i < contours.size(); i++) {
                    Target t; t.color_id = cp.id; t.area = contourArea(contours[i]);
                    t.draw_color = (cp.id == 1) ? Scalar(0, 0, 255) : Scalar(255, 0, 0); // 红/蓝

                    if (is_circle(contours[i], t.area)) {
                        minEnclosingCircle(contours[i], t.center, (float*)0); t.type = CIRCLE; all_targets.push_back(t);
                    } else {
                        RotatedRect rr = minAreaRect(contours[i]); Rect r = rr.boundingRect();
                        float ratio = max(r.width, r.height) / (float)min(r.width, r.height);
                        t.center = (Point2f)r.tl() + (Point2f)r.size() / 2.0f;

                        if (ratio > this->get_parameter("armor_ratio").as_double() && t.area > this->get_parameter("min_armor_area").as_double()) {
                            t.type = ARMOR; all_targets.push_back(t);
                        } else if (ratio < this->get_parameter("rect_ratio").as_double() && t.area > this->get_parameter("min_rect_area").as_double()) {
                            t.type = RECTANGLE; all_targets.push_back(t);
                        }
                    }
                }
            }

            teamX_challenge::msg::TargetArray targets_msg;
            for (const auto& t : all_targets) {
                circle(result_frame, t.center, 5, t.draw_color, -1);
                string label = get_type_str(t.type) + " (C:" + to_string(t.color_id) + ")";
                putText(result_frame, label, t.center + Point2f(10, -10), FONT_HERSHEY_SIMPLEX, 0.6, t.draw_color, 2);
                
                teamX_challenge::msg::Target target_msg;
                target_msg.color_id = t.color_id; target_msg.type = get_type_str(t.type);
                target_msg.center.x = t.center.x; target_msg.center.y = t.center.y;
                target_msg.area = t.area;
                targets_msg.targets.push_back(target_msg);
            }
            
            targets_pub_->publish(targets_msg);
            imshow("识别结果", result_frame);
            if (waitKey(1) == 's') {
                string file = save_path_ + "result_" + to_string(msg->header.stamp.sec) + ".png";
                imwrite(file, result_frame);
                RCLCPP_INFO(this->get_logger(), "截图已保存至: %s", file.c_str());
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    bool is_circle(const vector<Point>& c, double area) {
        double p = arcLength(c, true);
        return (p > 0) && (4 * CV_PI * area / (p*p) > this->get_parameter("circle_circularity").as_double()) && (area > this->get_parameter("min_circle_area").as_double());
    }
    string get_type_str(TargetType t) { switch(t) { case CIRCLE: return "CIRCLE"; case RECTANGLE: return "RECTANGLE"; case ARMOR: return "ARMOR"; default: return "UNKNOWN"; } }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}