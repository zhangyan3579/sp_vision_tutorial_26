#include <fmt/core.h>
#include <fmt/format.h>
#include <iostream>
#include <vector>

int main() {
    // 使用fmt::print打印信息
    fmt::print("Hello, fmt library!\n");
    
    // 使用fmt::format格式化字符串
    std::string name = "刘鹏";
    int age = 25;
    double score = 95.5;
    
    std::string info = fmt::format("姓名: {}, 年龄: {}, 分数: {:.1f}", name, age, score);
    fmt::print("{}\n", info);
    
    // 格式化数字
    fmt::print("二进制: {:b}, 十六进制: {:x}\n", 255, 255);
    
    // 格式化容器
    std::vector<int> numbers = {1, 2, 3, 4, 5};
    fmt::print("数字列表: {}\n", fmt::join(numbers, ", "));
    
    // 位置参数
    fmt::print("{1} {0} {2}\n", "世界", "你好", "!");
    
    return 0;
}