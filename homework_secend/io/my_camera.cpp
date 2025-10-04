#include"my_camera.hpp"
#include <iostream>

myCamera::myCamera() : handle_(nullptr), is_initialized_(false), is_grabbing_(false) {
    // 枚举USB设备
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    if (ret != MV_OK) {
        std::cerr << "枚举设备失败，错误码: " << ret << std::endl;
        return;
    }
    
    if (device_list_.nDeviceNum == 0) {
        std::cerr << "未找到相机设备" << std::endl;
        return;
    }
    
    // 创建句柄
    ret = MV_CC_CreateHandle(&handle_, device_list_.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cerr << "创建句柄失败，错误码: " << ret << std::endl;
        return;
    }
    
    // 打开设备
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        std::cerr << "打开设备失败，错误码: " << ret << std::endl;
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }
    
    // 设置相机参数
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);
    
    // 开始采集
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        std::cerr << "开始采集失败，错误码: " << ret << std::endl;
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }
    
    is_grabbing_ = true;
    is_initialized_ = true;
    std::cout << "相机初始化成功" << std::endl;
}

myCamera::~myCamera() {
    if (handle_ != nullptr) {
        if (is_grabbing_) {
            MV_CC_StopGrabbing(handle_);
            is_grabbing_ = false;
        }
        
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    
    std::cout << "相机资源已释放" << std::endl;
}

cv::Mat myCamera::read() {
    if (!is_initialized_ || handle_ == nullptr) {
        std::cerr << "相机未初始化或句柄无效" << std::endl;
        return cv::Mat();
    }
    
    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;
    
    int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
    if (ret != MV_OK) {
        std::cerr << "获取图像缓冲区失败，错误码: " << ret << std::endl;
        return cv::Mat();
    }
    
    cv::Mat img = transfer(raw);
    
    ret = MV_CC_FreeImageBuffer(handle_, &raw);
    if (ret != MV_OK) {
        std::cerr << "释放图像缓冲区失败，错误码: " << ret << std::endl;
    }
    
    return img;
}

cv::Mat myCamera::transfer(MV_FRAME_OUT& raw) {
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    };
    
    cv::cvtColor(img, img, type_map.at(pixel_type));
    
    return img;
}