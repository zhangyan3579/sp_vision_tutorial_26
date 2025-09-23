#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include "tools.hpp"
#include <iostream>

int main() {
    try {
        std::string image_path;
        
        // 询问用户输入图片路径
        fmt::print("请输入图片路径: ");
        std::getline(std::cin, image_path);
        
        // 检查文件是否存在
        cv::Mat test_image = cv::imread(image_path);
        if (test_image.empty()) {
            fmt::print(stderr, "错误: 无法读取图片文件 '{}'\n", image_path);
            return -1;
        }
        
        cv::Mat result_image;
        ResizeParams params = resizeAndCenterImage(image_path, result_image);
        
        // 使用fmt输出缩放参数
        fmt::print("\n=== 图片缩放参数 ===\n");
        fmt::print("原始图片尺寸: {} x {}\n", test_image.cols, test_image.rows);
        fmt::print("缩放比例: {:.4f}\n", params.scale);
        fmt::print("缩放后图片尺寸: {} x {}\n", params.new_width, params.new_height);
        fmt::print("在画布中的偏移量: X={}, Y={}\n", params.offset_x, params.offset_y);
        fmt::print("画布尺寸: 640 x 640\n");
        
        // 显示结果
        cv::imshow("缩放后的图片", result_image);
        fmt::print("\n按任意键退出...\n");
        cv::waitKey(0);
        
    } catch (const std::exception& e) {
        fmt::print(stderr, "错误: {}\n", e.what());
        return -1;
    }
    
    return 0;
}