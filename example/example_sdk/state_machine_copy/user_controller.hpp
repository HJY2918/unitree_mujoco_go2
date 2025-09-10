#pragma once   //一个非标准但广泛支持的预处理指令，确保头文件只会被包含一次，防止重复定义问题。

#include <array>
#include <vector>
#include <filesystem>   //用于文件系统操作，如路径处理。
#include <fstream>    //文件流类，用于文件读写。
#include <string>

#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "cfg.hpp"

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

        virtual void Calculate() = 0;  //执行计算逻辑，计算目标关节位置。

        virtual std::vector<float> GetLog() = 0;  //获取日志数据，用于记录输入、输出等关键信息。

        float dt, kp, kd;
        std::array<float, 12> init_pos;
        std::array<float, 12> jpos_des; //目标关节位置的数组
    };

    class ExampleUserController : public BasicUserController
    {
    public:
        ExampleUserController() {}

        void LoadParam(fs::path &param_folder)
        {
            // load param file
            std::ifstream cfg_file(param_folder / "params.json");
            std::cout << "Read params from: " << param_folder / "params.json" << std::endl;
            std::stringstream ss;
            ss << cfg_file.rdbuf();
            FromJsonString(ss.str(), cfg);

            // get data from json 读取机器人配置文件
            dt = cfg.dt;
            kp = cfg.kp;
            kd = cfg.kd;
            for (int i = 0; i < 12; ++i)
            {
                init_pos.at(i) = cfg.init_pos.at(i); //通过调用 at() 函数，访问容器中第 i 个元素，at函数提供边界检查，更安全
            }
        }

        void GetInput(RobotInterface &robot_interface, Gamepad &gamepad)
        {
            // save necessary data from input

            // record command 记录手柄的命令，这里可以修改
            cmd.at(0) = gamepad.ly;
            cmd.at(1) = -gamepad.lx;
            cmd.at(2) = -gamepad.rx;

            // 处理后的关节位置、速度数据（应该可以修改自己的数据处理方法：如滤波、归一化、滑动平均滤波）
            for (int i = 0; i < 12; ++i)
            {
                jpos_processed.at(i) = robot_interface.jpos.at(i) - init_pos.at(i); // 关节相对于初始位置的量
                jvel_processed.at(i) = robot_interface.jvel.at(i) / 2.0; //没看懂为什么要除以2.0
            }

            // // 定义一个滤波系数
            // float alpha = 0.1;  // 可以根据需求调整
            // for (int i = 0; i < 12; ++i)
            // {
            //     // 处理关节位置，应用低通滤波
            //     jpos_processed.at(i) = alpha * (robot_interface.jpos.at(i) - init_pos.at(i)) +
            //                         (1 - alpha) * jpos_processed.at(i); // 上次的处理结果
            //     // 处理关节速度，应用低通滤波
            //     jvel_processed.at(i) = alpha * (robot_interface.jvel.at(i) / 2.0) + 
            //                         (1 - alpha) * jvel_processed.at(i); // 上次的处理结果
            // }
        }

        void Reset(RobotInterface &robot_interface, Gamepad &gamepad)
        {
            GetInput(robot_interface, gamepad);  //首先调用getuinput获得手柄命令和机器人状态
            Calculate(); //将机器人的目标位置改成初始位置
        }

        void Calculate()
        {
            // calculate jpos_des
            jpos_des = init_pos;
        }

        std::vector<float> GetLog()
        {
            // record input, output and other info into a vector
            std::vector<float> log;
            for (int i = 0; i < 3; ++i)
            {
                log.push_back(cmd.at(i)); // 输入
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jpos_processed.at(i)); // 输出数据
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jvel_processed.at(i)); // 输出数据
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jpos_des.at(i));  // 目标输出
            }
            
            return log;
        }

        // cfg
        ExampleCfg cfg;

        // state
        std::array<float, 3> cmd;
        std::array<float, 12> jpos_processed;
        std::array<float, 12> jvel_processed;
    };
} // namespace unitree::common
