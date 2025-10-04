#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class myCamera {
public:
    /**
     * @brief 构造函数，初始化并打开相机
     */
    myCamera();
    
    /**
     * @brief 析构函数，释放相机资源
     */
    ~myCamera();
    
    /**
     * @brief 读取一帧图像
     * @return 读取到的图像，如果读取失败返回空Mat
     */
    cv::Mat read();

private:
    /**
     * @brief 将原始帧数据转换为OpenCV Mat格式
     * @param raw 原始帧数据
     * @return 转换后的图像
     */
    cv::Mat transfer(MV_FRAME_OUT& raw);
    
    void* handle_;                          // 相机句柄
    bool is_initialized_;                   // 相机是否初始化成功
    bool is_grabbing_;                      // 相机是否正在采集
    MV_CC_DEVICE_INFO_LIST device_list_;    // 设备列表
};

#endif // MY_CAMERA_HPP