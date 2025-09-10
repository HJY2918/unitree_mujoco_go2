#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void HighStateMessageHandler(const void *messages);  // 新增的 SportModeState 回调函数
    void LowCmdWrite();

private:
    // double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    //                                  0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    // double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    //                                    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};


    double dt = 0.002;
    double runing_time = 0.0;
    // double phase = 0.0;

    // 存储了目前机器人的状态信息
    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init 
    unitree_go::msg::dds_::LowState_ low_state{}; // default init
    unitree_go::msg::dds_::SportModeState_ sport_mode_state{};  // 新增的 SportModeState

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    // 新增的 SportModeState 订阅器
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportmode_subscriber;  

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    // example
    float Kp = 60.0;
    float Kd = 5.0;
    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 1000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;         
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /* create subscriber for SportModeState */
    sportmode_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));  // 添加订阅器
    sportmode_subscriber->InitChannel(std::bind(&Custom::HighStateMessageHandler, this, std::placeholders::_1), 1);  // 绑定回调函数

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

// LowState: 电机状态信息
// LowState: 电机状态和传感器信息的输出
void Custom::LowStateMessageHandler(const void *message)
{
    // 将传递的 message 转换为 LowState 消息类型
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;

    // // 使用 stringstream 来存储所有输出信息
    // std::stringstream ss;

    // ss << "--------------------------------------" << std::endl;
    // // 打印 IMU 四元数信息
    // ss << "IMU Quaternion: "
    //           << low_state.imu_state().quaternion()[0] << " "
    //           << low_state.imu_state().quaternion()[1] << " "
    //           << low_state.imu_state().quaternion()[2] << " "
    //           << low_state.imu_state().quaternion()[3] << std::endl;

    // // 打印 IMU 陀螺仪数据
    // ss << "IMU Gyroscope: "
    //           << low_state.imu_state().gyroscope()[0] << " "
    //           << low_state.imu_state().gyroscope()[1] << " "
    //           << low_state.imu_state().gyroscope()[2] << std::endl;

    // // 打印 IMU 加速度计数据
    // ss << "IMU Accelerometer: "
    //           << low_state.imu_state().accelerometer()[0] << " "
    //           << low_state.imu_state().accelerometer()[1] << " "
    //           << low_state.imu_state().accelerometer()[2] << std::endl;

    // // 打印所有电机的状态，可以使用循环
    // for (int i = 0; i < 12; i++)  // 假设有12个电机
    // {
    //     ss << "Motor " << i << " State: "
    //               << "Position: " << low_state.motor_state()[i].q() << " "
    //               << "Velocity: " << low_state.motor_state()[i].dq() << " "
    //               << "Estimated Torque: " << low_state.motor_state()[i].tau_est() << std::endl;
    // }

    // ss << "--------------------------------------" << std::endl;

    // // 最后一次性输出到控制台
    // std::cout << ss.str();
}

// SportModeState: 机器人位置和状态信息
void Custom::HighStateMessageHandler(const void *messages)
{
    // 将传递的 SportModeState 消息转换为正确的类型
    sport_mode_state = *(unitree_go::msg::dds_::SportModeState_ *)messages;

    // // 使用 stringstream 来存储所有输出信息
    // std::stringstream ss;

    // ss << "--------------------------------------" << std::endl;
    // // 输出机器人的位置（Position）
    // ss << "Robot Position: "
    //           << "X: " << sport_mode_state.position()[0] << " "
    //           << "Y: " << sport_mode_state.position()[1] << " "
    //           << "Z: " << sport_mode_state.position()[2] << std::endl;

    // // 输出机器人的速度（Velocity）
    // ss << "Robot Velocity: "
    //           << "Vx: " << sport_mode_state.velocity()[0] << " "
    //           << "Vy: " << sport_mode_state.velocity()[1] << " "
    //           << "Vz: " << sport_mode_state.velocity()[2] << std::endl;
    
    // ss << "--------------------------------------" << std::endl;

    // // 最后一次性输出到控制台
    // std::cout << ss.str();
}


void Custom::LowCmdWrite()
{

    runing_time += dt;

    // 步态周期 (Trot 步态, 假设周期为 1 秒)
    double gait_cycle = 1.0;  // 步态周期
    double phase_offset[4] = {0, 0.5, 0.5, 0};  // 前后腿的相位偏移
    double leg_phase[4];  // 每条腿的当前相位
    double step_height = 0.2;  // 抬腿高度
    double step_length = 0.1;  // 步长
    

    // 读取当前机器人状态
    for (int i = 0; i < 12; i++)
    {
        // 获取当前关节位置和速度
        double current_joint_position = low_state.motor_state()[i].q();
        double current_joint_velocity = low_state.motor_state()[i].dq();

        // 这里可以使用当前状态来调整控制量，例如 PID 控制
    }

    // 控制代码
    // for (int i = 0; i < 4; i++) {
    //     leg_phase[i] = fmod((runing_time / gait_cycle + phase_offset[i]), 1.0);

    //     // 判断支撑相还是摆动相
    //     if (leg_phase[i] < 0.5) {  // 支撑相，保持站立
    //         low_cmd.motor_cmd()[i * 3 + 0].q() = 0.0;  // hip (髋关节)
    //         low_cmd.motor_cmd()[i * 3 + 1].q() = 0.5;  // knee (膝关节)
    //         low_cmd.motor_cmd()[i * 3 + 2].q() = -1.0;  // ankle (踝关节)

    //         low_cmd.motor_cmd()[i * 3 + 0].dq() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 1].dq() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 2].dq() = 0;

    //         // 设置相应的控制参数（例如刚度、阻尼等）
    //         low_cmd.motor_cmd()[i * 3 + 0].kp() = 50.0;
    //         low_cmd.motor_cmd()[i * 3 + 1].kp() = 50.0;
    //         low_cmd.motor_cmd()[i * 3 + 2].kp() = 50.0;

    //         low_cmd.motor_cmd()[i * 3 + 0].kd() = 3.5;
    //         low_cmd.motor_cmd()[i * 3 + 1].kd() = 3.5;
    //         low_cmd.motor_cmd()[i * 3 + 2].kd() = 3.5;

    //         low_cmd.motor_cmd()[i * 3 + 0].tau() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 1].tau() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 2].tau() = 0;

    //     }
    //     else {  // 摆动相，抬腿前进
    //         double swing_phase = (leg_phase[i] - 0.5) * 2.0;  // 摆动相相位

    //         // 通过正弦函数控制腿的抬起和落下
    //         low_cmd.motor_cmd()[i * 3 + 0].q() = step_length * (0.5 - swing_phase);  // hip 关节在步长内移动
    //         low_cmd.motor_cmd()[i * 3 + 1].q() = 0.5 - step_height * sin(M_PI * swing_phase);  // 抬腿高度的正弦波
    //         low_cmd.motor_cmd()[i * 3 + 2].q() = -1.0;  // ankle 保持不变

    //         low_cmd.motor_cmd()[i * 3 + 0].dq() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 1].dq() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 2].dq() = 0;

    //         // 设置相应的控制参数
    //         low_cmd.motor_cmd()[i * 3 + 0].kp() = 30.0;  // 髋关节的控制刚度
    //         low_cmd.motor_cmd()[i * 3 + 1].kp() = 30.0;  // 膝关节的控制刚度
    //         low_cmd.motor_cmd()[i * 3 + 2].kp() = 30.0;  // 踝关节的控制刚度

    //         low_cmd.motor_cmd()[i * 3 + 0].kd() = 3.5;
    //         low_cmd.motor_cmd()[i * 3 + 1].kd() = 3.5;
    //         low_cmd.motor_cmd()[i * 3 + 2].kd() = 3.5;

    //         low_cmd.motor_cmd()[i * 3 + 0].tau() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 1].tau() = 0;
    //         low_cmd.motor_cmd()[i * 3 + 2].tau() = 0;

    //     }
    // }
        if(_percent_4<1)
        {
            std::cout<<"Read sensor data example: "<<std::endl;
            std::cout<<"Joint 0 pos: "<<low_state.motor_state()[0].q()<<std::endl;
            std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
            std::cout<<"Foot force "<<low_state.foot_force()[0]<<std::endl;
            std::cout<<std::endl;
        }
        if((_percent_4 == 1) && ( done == false))
        {
            std::cout<<"The example is done! "<<std::endl;
            std::cout<<std::endl;
            done = true;
        }
        if(firstRun)
        {
            for(int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    Custom custom;
    custom.Init(); // 初始化并启动控制线程

    while (1)
    {
        sleep(10);
    }

    return 0;
}