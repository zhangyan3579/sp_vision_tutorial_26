#include "tools.hpp"
#include <opencv2/opencv.hpp>

ResizeParams resizeAndCenterImage(const std::string& input_path, cv::Mat& output_mat) {
    // 读取原始图片
    cv::Mat image = cv::imread(input_path);
    if (image.empty()) {
        throw std::runtime_error("无法读取图片: " + input_path);
    }
    
    int original_width = image.cols;
    int original_height = image.rows;
    
    // 计算缩放比例
    double scale_x = 640.0 / original_width;
    double scale_y = 640.0 / original_height;
    double scale = std::min(scale_x, scale_y);  // 等比例缩放，取较小的比例
    
    // 计算缩放后的尺寸
    int new_width = static_cast<int>(original_width * scale);
    int new_height = static_cast<int>(original_height * scale);
    
    // 缩放图片
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(new_width, new_height));
    
    // 创建640x640的黑色画布
    output_mat = cv::Mat::zeros(640, 640, image.type());
    
    // 计算居中位置
    int offset_x = (640 - new_width) / 2;
    int offset_y = (640 - new_height) / 2;
    
    // 将缩放后的图片复制到画布中央
    cv::Mat roi = output_mat(cv::Rect(offset_x, offset_y, new_width, new_height));
    resized_image.copyTo(roi);
    
    // 返回缩放参数
    ResizeParams params;
    params.scale = scale;
    params.offset_x = offset_x;
    params.offset_y = offset_y;
    params.new_width = new_width;
    params.new_height = new_height;
    
    return params;
}