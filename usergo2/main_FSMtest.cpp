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

#include "./interface/robot_controller.hpp"
#include "./control/user_controller.hpp"
#include "./interface/robot_interface.hpp"

using namespace unitree::common;
using namespace unitree::robot;

namespace fs = std::filesystem;

int main(int argc, char const *argv[]) //运行命令  ./build//state_machine_example --param params
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
    fs::path param = fs::current_path() / param_folder;

    // log
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss; //是一种可以将数据写入或读取字符串的流对象。
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%H_%M_%S");    //将格式化后的时间戳写入字符串流 ss

    fs::path log_folder = fs::current_path() / "logs" / ss.str();
    fs::create_directories(log_folder);

    std::ofstream cfg_file(log_folder / "cfg.txt");
    cfg_file << "param_folder: " << param << std::endl;
    cfg_file.close();

    fs::path log_file_name = log_folder / "log.txt";

    RobotController<ExampleUserController> robot_controller(log_file_name);
    robot_controller.LoadParam(param);

    // robot_controller.InitDdsModel();
    robot_controller.InitDdsModel("lo"); //仿真是要用lo

    robot_controller.StartControl();

    return 0;
}