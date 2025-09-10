#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>

#include "./FSM/robot_controller.hpp"

// 这个需要替换
// #include "./control/user_controller.hpp"
#include "./control/UserBalanceCtrl.h"

using namespace unitree::common;
using namespace unitree::robot;

namespace fs = std::filesystem;

//运行命令  ./build//state_machine_example --param params
int main(int argc, char const *argv[]) 
{
    std::string param_folder; //定义param文件夹名称的变量
    // parse command line params
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "--param" && i + 1 < argc)
        {
            param_folder = argv[i + 1]; //存放params文件的文件夹名称
        }
    }
    fs::path param = fs::current_path() / param_folder; // params.json路径

    // 时间
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss; //是一种可以将数据写入或读取字符串的流对象。
    //将格式化后的时间戳写入字符串流 ss
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%H_%M_%S");    

    // log
    fs::path log_folder = fs::current_path() / "logs" / ss.str(); // log文件夹名称
    fs::create_directories(log_folder); // 创建文件夹

    std::ofstream cfg_file(log_folder / "cfg.txt");     //cfg.txt中保存参数param文件夹路径
    cfg_file << "param_folder: " << param << std::endl;
    cfg_file.close();

    fs::path log_file_name = log_folder / "log.txt";    // log.txt文件路径


    // 用户定义的控制器，需要修改
    // 定义并实例化一个模板类 RobotController 的对象，
    // 其中模板参数为 ExampleUserController 类
    RobotController<UserBalanceCtrl> robot_controller(log_file_name); 

    // 加载控制器参数
    robot_controller.LoadParam(param);


    // robot_controller.InitDdsModel();
    robot_controller.InitDdsModel("lo"); //仿真是要用lo,初始化通信通道DDS

    robot_controller.StartControl();    // 控制器起作用

    return 0;

}