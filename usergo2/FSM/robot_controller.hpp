#pragma once

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <future>
#include <algorithm>


#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/common/thread/thread.hpp"

#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/common/time/time_tool.hpp"

#include "FSM.hpp"
// #include "gamepad.hpp"

#include "../interface/robot_interface.hpp"
#include "../interface/gamepadexample.hpp" //新增


using namespace unitree::common;
using namespace unitree::robot;
namespace fs = std::filesystem;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

// RobotController 类中的所有涉及 USER_CTRL 的地方都会替换为 ExampleUserController，
// 从而为 RobotController 提供 ExampleUserController 中定义的具体功能
template <typename USER_CTRL> // USER_CTRL 是模板参数
class RobotController
{
protected:
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;           // 发布底层控制指令
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;     // 订阅状态指令
    ThreadPtr low_cmd_write_thread_ptr, control_thread_ptr;                         // 写入控制
    unitree_go::msg::dds_::LowCmd_ cmd;                                             // cmd信息
    unitree_go::msg::dds_::LowState_ state;                                         // 状态信息

    Gamepad gamepad;                                                                // 手柄数据
    REMOTE_DATA_RX rx;  //rx类包含用来表示手柄的按钮和遥感的数据RF_RX   或者  buff【40】  

    FiniteStateMachine state_machine;  //状态机部分
    USER_CTRL ctrl;                    // 设计到USER_CTRL的，均调用ExampleUserController 中定义的具体功能
    RobotInterface robot_interface;    //接口

    std::mutex state_mutex, cmd_mutex;  //？

    std::ofstream log_file;             // 输出log文件

    uint64_t ctrl_dt_micro_sec = 2000; //控制线程的执行频率，单位是微秒 0.002s

    std::vector<float> compute_time;


    GamepadExample gamepadexample;//

public:
    RobotController() {}

    RobotController(fs::path &log_file_name) // 日志log
    {
        // set log file
        log_file = std::ofstream(log_file_name, std::ios::binary); //打开文件：通过 std::ofstream 构造函数，打开指定路径 log_file_name 的文件，模式为二进制（std::ios::binary）。
                                                                   //写入数据准备：一旦文件被成功打开，程序可以通过 log_file 对象将日志数据以二进制形式写入文件。如果文件不存在，std::ofstream 会创建一个新文件。
    }

    void LoadParam(fs::path &param_folder)
    {
        ctrl.LoadParam(param_folder); //调用控制器的 LoadParam 方法加载参数
    }

    void InitDdsModel(const std::string &networkInterface = "")   //DDS通信初始化初始化
    {
        // init dds
        // ChannelFactory::Instance()->Init(0, networkInterface); 
        gamepadexample.InitDdsModel("lo");
        ChannelFactory::Instance()->Init(1, networkInterface); //仿真为Init(1, "lo"); 实物Init(0, networkInterface);

        /*create publisher*/
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher->InitChannel();

        /*create subscriber*/
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber->InitChannel(std::bind(&RobotController::LowStateMessageHandler, this, std::placeholders::_1), 1);


    }

    void StartControl() 
    {
        // waiting for gamepad command to start the control thread
        std::chrono::milliseconds duration(100);  //调用std::chrono::milliseconds类，创建一个duration对象，代表100毫秒
        // listen to gamepad command 监听遥控器输入
        while (true)
        {
            std::cout << "Press R2 to start!" << std::endl;
            std::this_thread::sleep_for(duration);  //每隔100ms调用sleep_for函数暂停一下

            InteprateGamePad();
            if (gamepad.R2.on_press) //如果R2被按下了，就跳出循环
            {
                break;
            }
        }

        // prepare for start
        std::cout << "Start!" << std::endl;
        Damping(); // 开始控制，首先进入阻尼模式
        ctrl_dt_micro_sec = static_cast<uint64_t>(ctrl.dt * 1000000);  //精确控制线程的执行频率，ctrl.dt的单位是微秒（microseconds），并通过 static_cast 将其强制转换为 uint64_t 类型

        // Start the control thread //不断调用 ControlStep 函数来进行机器人控制的主要逻辑
        control_thread_ptr = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE, ctrl_dt_micro_sec, &RobotController::ControlStep, this); 

        // Start the lowlevel command thread
        std::this_thread::sleep_for(duration);
        StartSendCmd(); //StartSendCmd() 通过 CreateRecurrentThreadEx 以 2000 微秒的时间步长启动一个新线程，周期性地将命令发送出去

        // keep the main thread alive 
        while (true)
        {
            std::this_thread::sleep_for(duration);
        }
    }


private:

    void LowStateMessageHandler(const void *message) //收到low_state之后处理信息的回调函数
    {
        state = *(unitree_go::msg::dds_::LowState_ *)message;
        {
            std::lock_guard<std::mutex> lock(state_mutex); //mutex防止多个线程竞争
            robot_interface.GetState(state); //获取电机和imu数据
        }
    }

    // void InteprateGamePad()  //更新手柄数据
    // {
    //     // update gamepad
    //     memcpy(rx.buff, &state.wireless_remote()[0], 40);  //将wireless_remote第一个元素开始的钱40字节复制到rx.buff
    //     gamepad.update(rx.RF_RX);  //更新手柄数据
    // }

    void InteprateGamePad()  //更新手柄数据
    {
        // update gamepad
       // 使用 getJoystickMsg() 访问 protected 成员
        rx.RF_RX.btn.value = gamepadexample.getJoystickMsg().keys();
        gamepad.update(rx.RF_RX);  //更新手柄数据
    }


    void LowCmdwriteHandler() 
    {
        // write low-level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            lowcmd_publisher->Write(cmd);
        }
    }

    void StartSendCmd()
    {   
        /*loop publishing thread 启动新的线程*/
        low_cmd_write_thread_ptr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &RobotController::LowCmdwriteHandler, this);
    }

    //-------------------------------------------------------------------------------------
    // FMS状态机
   void UpdateStateMachine()
    {
        // R2 -> Stand   5
        // A -> Ctrl     A
        // L2 + B -> Stop   6+B
        //只有站起来才能ctl
        if (gamepad.R2.on_press)
        {
            if (state_machine.Stand()) ////如果当前状态是 DAMPING 或 CTRL，则切换回 STAND 状态。
            {
                StandCallback(); //  参数赋值
            }
        }
        if (gamepad.A.on_press)  
        {
            if (state_machine.Ctrl())  //只有在当前状态为 STAND 且 pd_ratio 大于 0.95 时，才会切换到 CTRL 状态。
            {
                CtrlCallback(); //  参数赋值
            }
        }
        if (gamepad.L2.pressed && gamepad.B.pressed) 
        {
            state_machine.Stop(); //调整为阻尼部分
        } 
    }
    //-------------------------------------------------------------------------------------

    // 线程中不断调用的函数，重要！！！！！！！！！！！！！！！！！！！！！！
    void ControlStep()
    {
        // main loop

        // update state
        InteprateGamePad(); //从游戏手柄读取输入
        UpdateStateMachine(); //更新状态机的当前状态。

        //-------------------------------------------------------------------------------------
        // select control modes according to the state machine
        auto start = std::chrono::high_resolution_clock::now(); //用于获取当前的高精度时间戳
        if (state_machine.state == STATES::STAND)
        {
            Standing(); // 站立控制代码
        }
        if (state_machine.state == STATES::DAMPING)
        {
            Damping();
        }
        if (state_machine.state == STATES::CTRL)
        {
            UserControlStep(true);
            if (CheckTermination())
            {
                state_machine.Stop();
            }
        }
        //-------------------------------------------------------------------------------------
        // update low-level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            robot_interface.SetCommand(cmd);
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        compute_time.push_back(duration.count() / 1000.); //记录执行时间，因为duration.count() 是获取微秒值

        // write log
        WriteLog();

        if (compute_time.size() == 100) //每 100 次循环后，统计并输出执行时间的性能信息：
        {
            float sum = 0;
            for (auto &t : compute_time)
            {
                sum += t;
            }
            std::cout << "Performance: mean: " << sum / 100
                      << " ms; max: " << *std::max_element(compute_time.begin(), compute_time.end())
                      << " ms; min: " << *std::min_element(compute_time.begin(), compute_time.end())
                      << "ms." << std::endl;

            compute_time.clear();

            std::cout << "Current State: " << static_cast<size_t>(state_machine.state) << std::endl;
        }
    }

    void WriteLog()
    {
        if (log_file.is_open())
        {
            auto log = ctrl.GetLog();
            for (const auto &v : log) //for 循环使用范围基 for 循环语法来遍历 log 中的每一个元素 v
            {
                log_file << v << " ";
            }
            log_file << std::endl;
        }
    }

    void StandCallback() // 参数赋值
    {
        robot_interface.jpos_des = ctrl.init_pos;
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(ctrl.kp * state_machine.pd_ratio);
        robot_interface.kd.fill(ctrl.kd * state_machine.pd_ratio);
        robot_interface.tau_ff.fill(0.);
    }

    void CtrlCallback()
    {
        ctrl.Reset(robot_interface, gamepad);

        robot_interface.jpos_des = ctrl.init_pos;
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(ctrl.kp);
        robot_interface.kd.fill(ctrl.kd);
        robot_interface.tau_ff.fill(0.);
    }

    void Damping(float kd = 2.0)
    {
        robot_interface.jpos_des = ctrl.init_pos;
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(0.);
        robot_interface.kd.fill(kd);
        robot_interface.tau_ff.fill(0.);
    }

    void Standing(float kp = 40.0, float kd = 1.0)
    {
        if (gamepad.R2.pressed)
        {
            state_machine.Standing(true); //pd_ratio增加
        }
        if (gamepad.R1.pressed)
        {
            state_machine.Standing(false); //pd_ratio减少
        }

        UserControlStep(false); //更新了机器人的状态，但不会将控制指令发送给机器人

        robot_interface.kp.fill(kp * state_machine.pd_ratio);
        robot_interface.kd.fill(kd * state_machine.pd_ratio);
    }

    void UserControlStep(bool send = true) 
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl.GetInput(robot_interface, gamepad);
        }
        ctrl.Calculate(); // 计算控制量?

        if (send)
        {
            robot_interface.jpos_des = ctrl.jpos_des;
            // cmd = ctrl.cmd; // xin zeng
        }
    }

    bool CheckTermination() //查看机器人的状态，若机器人的姿态可能已经发生了翻转或倾斜，重力投影到机器人坐标系是正值，返回true
    {
        if (robot_interface.projected_gravity.at(2) > 0)
        {
            return true;
        }
        return false;
    }
};
