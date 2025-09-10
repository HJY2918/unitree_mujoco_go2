#pragma once   //一个非标准但广泛支持的预处理指令，确保头文件只会被包含一次，防止重复定义问题。

#include <array>
#include <vector>
#include <filesystem>   //用于文件系统操作，如路径处理。
#include <fstream>    //文件流类，用于文件读写。
#include <string>

#include "../interface/robot_interface.hpp"
#include "../interface/gamepad.hpp"
#include "../cfg.hpp" // 配置文件hpp

namespace fs = std::filesystem;

namespace unitree::common
{
    class BasicUserController //父类，包含了一些纯虚函数，任何继承此类的子类都必须实现这些虚函数。
    {
    public:
        BasicUserController() {}  //无参构造函数

        virtual void LoadParam(fs::path &param_folder) = 0;   //从给定的路径加载参数文件
                                                                //virtual 关键字表示它们是虚函数，虚函数在类中定义，但其实现需要由派生类（继承该类的子类）提供。
        virtual void Reset(RobotInterface &robot_interface, Gamepad &gamepad) = 0;  //重置控制器的状态，通常在控制循环开始时调用。

        virtual void GetInput(RobotInterface &robot_interface, Gamepad &gamepad) = 0; //从 RobotInterface（机器人接口）和 Gamepad（手柄输入）中获取必要的数据。

        virtual void Calculate() = 0;  //执行计算逻辑。

        virtual std::vector<float> GetLog() = 0;  //获取日志数据，用于记录输入、输出等关键信息。

        float dt, kp, kd;
        std::array<float, 12> init_pos;
        std::array<float, 12> jpos_des; //目标关节位置的数组
    };
} // namespace unitree::common
