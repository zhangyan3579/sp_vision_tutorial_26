#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>

// 缩放参数结构体
struct ResizeParams {
    double scale;       // 缩放比例
    int offset_x;       // X轴偏移量
    int offset_y;       // Y轴偏移量
    int new_width;      // 缩放后的宽度
    int new_height;     // 缩放后的高度
};

/**
 * @brief 将图片等比例缩放并居中放置在640x640画布上
 * @param input_path 输入图片路径
 * @param output_mat 输出图像
 * @return ResizeParams 缩放参数
 */
ResizeParams resizeAndCenterImage(const std::string& input_path, cv::Mat& output_mat);

#endif // TOOLS_HPP