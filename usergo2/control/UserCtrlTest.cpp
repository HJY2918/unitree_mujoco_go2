#include "UserCtrlTest.h"

// 构造函数
unitree::common::UserBalanceCtrl::UserBalanceCtrl() {
    // 初始化代码
}

// 析构函数
unitree::common::UserBalanceCtrl::~UserBalanceCtrl() {
    // 清理代码
}

// 加载配置文件的参数：初始化位置，dt, kp, kd
void unitree::common::UserBalanceCtrl::LoadParam(fs::path &param_folder)
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
        // 通过调用 at() 函数，访问容器中第 i 个元素，at函数提供边界检查，更安全
        init_pos.at(i) = cfg.init_pos.at(i);
    }
}

// 读取机器人的状态以及遥控器信息， 这里状态需要补充滤波等估计??
void unitree::common::UserBalanceCtrl::GetInput(RobotInterface &robot_interface, Gamepad &gamepad)
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
        jvel_processed.at(i) = robot_interface.jvel.at(i) / 2.0;            // 没看懂为什么要除以2.0
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

// 重置控制器
void unitree::common::UserBalanceCtrl::Reset(RobotInterface &robot_interface, Gamepad &gamepad)
{
    GetInput(robot_interface, gamepad); // 首先调用getinput读取机器人的状态以及遥控器信息
    Calculate();                        // 将机器人的目标位置改成初始位置
}

// 计算控制量
void unitree::common::UserBalanceCtrl::Calculate()
{
    // calculate jpos_des
    jpos_des = init_pos;
}

std::vector<float> unitree::common::UserBalanceCtrl::GetLog()
{
    // record input, output and other info into a vector
    std::vector<float> log;
    for (int i = 0; i < 3; ++i)
    {
        log.push_back(cmd.at(i)); // 输入cmd
    }
    for (int i = 0; i < 12; ++i)
    {
        log.push_back(jpos_processed.at(i)); // 输出数据jpos_processed
    }
    for (int i = 0; i < 12; ++i)
    {
        log.push_back(jvel_processed.at(i)); // 输出数据jvel_processed
    }
    for (int i = 0; i < 12; ++i)
    {
        log.push_back(jpos_des.at(i)); // 目标输出jpos_des
    }

    return log;
}
