#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <sched.h>
#include <unistd.h>
#include <csignal>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

using namespace unitree::common;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

bool running = true;

// over watch the ctrl+c command 暂停程序
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}


// 初始化通信通道，实例化Custom类，并进入主循环，保持程序运行。
int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
        // "lo" 代表本地回环接口（loopback interface），如果要和其他通信，再修改
    }
    else
    {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();



    /* set real-time process 设置当前进程的调度策略为实时调度，确保在实时系统中具有更高优先级。*/
    setProcessScheduler();
    /* set the print format 设置标准输出流的浮点数格式为固定小数点格式，并保留 3 位小数。*/ 
    std::cout << std::fixed << std::setprecision(3); 

    IOInterface *ioInter; // IOInterface 是一个接口类，定义了与机器人或仿真系统进行数据交互的接口。
    CtrlPlatform ctrlPlat; // CtrlPlatform 是一个枚举，指定控制平台是仿真（GAZEBO）还是真实机器人（REALROBOT）。

    #ifdef COMPILE_WITH_SIMULATION
        ioInter = new IOROS();
        ctrlPlat = CtrlPlatform::GAZEBO;
    #endif // COMPILE_WITH_SIMULATION

    #ifdef COMPILE_WITH_REAL_ROBOT
        ioInter = new IOSDK();
        ctrlPlat = CtrlPlatform::REALROBOT;
    #endif // COMPILE_WITH_REAL_ROBOT
        //CtrlComponents 是一个控制组件类，负责整合机器人控制系统中的各个子系统。这里传入了 ioInter，表示控制组件将通过对应的 I/O 接口与机器人或仿真进行交互。
        CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
        //将控制平台设置为仿真平台（GAZEBO）或真实机器人平台（REALROBOT），之前的条件编译决定了具体使用哪个平台。
        ctrlComp->ctrlPlatform = ctrlPlat;
        //设置控制循环的时间间隔为 0.002 秒，也就是 500Hz 的控制频率。
        ctrlComp->dt = 0.002; // run at 500hz
        //将运行标志指向全局的 running 变量，这用于控制程序是否继续运行。
        ctrlComp->running = &running;

    #ifdef ROBOT_TYPE_A1
        ctrlComp->robotModel = new A1Robot();
    #endif
    #ifdef ROBOT_TYPE_Go1
        ctrlComp->robotModel = new Go1Robot();
    #endif

    // WaveGenerator 是一个步态生成器类，负责生成机器人行走的步态。
    //0.45：步态的周期。0.5：支撑时间与总时间的比例。Vec4(0, 0.5, 0.5, 0)：四条腿的步态相位。
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim


    // 这行代码生成或初始化控制对象，具体实现可能涉及准备好所有的控制相关对象和数据结构，为之后的控制循环做好准备。
    ctrlComp->geneObj(); 


    // ControlFrame 是控制框架类，它接收一个控制组件 ctrlComp 作为参数，用于管理控制系统的主循环。
    ControlFrame ctrlFrame(ctrlComp);

    // 控制框架
    while (running)
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
