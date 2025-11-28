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

// 在构造函数中添加初始化
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

// 在各检测函数中替换原有的颜色分割代码
// 球体检测中使用：
Mat mask = adaptiveColorSegmentation(hsv, "red");

// 绿色矩形检测中使用：
Mat green_mask = adaptiveColorSegmentation(hsv, "green");

// 装甲板检测中使用：
Mat red_mask = adaptiveColorSegmentation(hsv, "red");

// 在callback_camera中添加图像预处理
Mat processed_image = preprocessImage(image);
