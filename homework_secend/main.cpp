#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main()
{
    // 初始化相机
    myCamera camera;
    
    // 初始化YOLO类
    auto_aim::YOLO yolo_detector("./configs/yolo.yaml", true);
    
    tools::logger()->info("Camera and YOLO detector initialized successfully");
    
    while (true) {
        // 调用相机读取图像
        cv::Mat img = camera.read();
        if (img.empty()) {
            tools::logger()->error("Failed to capture image from camera");
            break;
        }
        
        // 调用yolo识别装甲板
        auto armors = yolo_detector.detect(img);
        
        // 在图像上绘制检测结果
        for (const auto& armor : armors) {
            // 检查是否有足够的点来绘制矩形
            if (armor.points.size() == 4) {
                // 使用红色绘制装甲板的四个关键点并连接成闭合矩形
                tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255), 2);
                
                // 可选：在装甲板中心显示类型和名称信息
                std::string armor_info = auto_aim::COLORS[armor.color] + " " + 
                                       auto_aim::ARMOR_NAMES[armor.name];
                cv::Point center = cv::Point(armor.center.x, armor.center.y);
                tools::draw_text(img, armor_info, center, cv::Scalar(0, 255, 255), 0.6, 2);
            }
        }
        
        // 显示检测到的装甲板数量
        std::string count_text = "Detected: " + std::to_string(armors.size()) + " armors";
        tools::draw_text(img, count_text, cv::Point(10, 30), cv::Scalar(255, 255, 0), 0.8, 2);
        
        // 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("Armor Detection", img);
        
        // 按'q'键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    
    tools::logger()->info("Program exited normally");
    return 0;
}