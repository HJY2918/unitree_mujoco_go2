#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "../common/mathTypes.h"
#include "../common/mathTools.h"


struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[12];

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 180;
        motorCmd[legID*3+0].Kd = 8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 180;
        motorCmd[legID*3+1].Kd = 8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 300;
        motorCmd[legID*3+2].Kd = 15;
    }
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 60;
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 40;
        motorCmd[legID*3+1].Kd = 4;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 80;
        motorCmd[legID*3+2].Kd = 7;
    }
    void setZeroGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0.8;
        motorCmd[legID*3+0].Kd = 0.8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0.8;
        motorCmd[legID*3+1].Kd = 0.8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0.8;
        motorCmd[legID*3+2].Kd = 0.8;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 3;
        motorCmd[legID*3+0].Kd = 2;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 3;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 2;
    }
};

// #include "unitree/idl/go2/LowCmd_.hpp"  // 新增

// constexpr double PosStopF = (2.146E+9f);
// constexpr double VelStopF = (16000.0f);

// class LowlevelCmd{
// public:
//     unitree_go::msg::dds_::LowCmd_ low_cmd;  // 使用新的 LowCmd 接口

//     LowlevelCmd() {
//         InitLowCmd();  // 初始化
//     }

//     void InitLowCmd() {
//         low_cmd.head()[0] = 0xFE;
//         low_cmd.head()[1] = 0xEF;
//         low_cmd.level_flag() = 0xFF;
//         low_cmd.gpio() = 0;

//         for(int i = 0; i < 20; i++) {
//             low_cmd.motor_cmd()[i].mode() = (0x01);  // 设置为伺服模式
//             low_cmd.motor_cmd()[i].q() = (PosStopF);
//             low_cmd.motor_cmd()[i].kp() = (0);
//             low_cmd.motor_cmd()[i].dq() = (VelStopF);
//             low_cmd.motor_cmd()[i].kd() = (0);
//             low_cmd.motor_cmd()[i].tau() = (0);
//         }
//     }

//     // 设置关节角度
//     void setQ(Vec12 q){
//         for(int i(0); i<12; ++i){
//             low_cmd.motor_cmd()[i].q() = q(i);
//         }
//     }

//     // 针对特定腿设置关节角度
//     void setQ(int legID, Vec3 qi){
//         low_cmd.motor_cmd()[legID * 3 + 0].q() = qi(0);
//         low_cmd.motor_cmd()[legID * 3 + 1].q() = qi(1);
//         low_cmd.motor_cmd()[legID * 3 + 2].q() = qi(2);
//     }

//     // 设置关节速度
//     void setQd(Vec12 qd){
//         for(int i(0); i<12; ++i){
//             low_cmd.motor_cmd()[i].dq() = qd(i);
//         }
//     }

//     // 针对特定腿设置关节速度
//     void setQd(int legID, Vec3 qdi){
//         low_cmd.motor_cmd()[legID * 3 + 0].dq() = qdi(0);
//         low_cmd.motor_cmd()[legID * 3 + 1].dq() = qdi(1);
//         low_cmd.motor_cmd()[legID * 3 + 2].dq() = qdi(2);
//     }

//     // 设置关节力矩
//     void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
//         for(int i(0); i<12; ++i){
//             if(std::isnan(tau(i))){
//                 printf("[ERROR] The setTau function meets Nan\n");
//             }
//             low_cmd.motor_cmd()[i].tau() = saturation(tau(i), torqueLimit);
//         }
//     }

//     // 设置关节速度为0
//     void setZeroDq(int legID){
//         low_cmd.motor_cmd()[legID * 3 + 0].dq() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 1].dq() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 2].dq() = 0;
//     }

//     // 设置所有关节速度为0
//     void setZeroDq(){
//         for(int i(0); i<4; ++i){
//             setZeroDq(i);
//         }
//     }

//     // 设置关节力矩为0
//     void setZeroTau(int legID){
//         low_cmd.motor_cmd()[legID * 3 + 0].tau() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 1].tau() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 2].tau() = 0;
//     }


//     // 0x00 可能表示停止模式或无控制模式
//     // 0x01 表示伺服模式（PMSM模式，用于位置和力矩控制）
//     // 0x02 可能表示速度控制模式
//     // 0x03 可能表示力矩控制模式
//     // 模拟站立增益设置
//     void setSimStanceGain(int legID){
//         low_cmd.motor_cmd()[legID * 3 + 0].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 0].kp() = 180;
//         low_cmd.motor_cmd()[legID * 3 + 0].kd() = 8;
//         low_cmd.motor_cmd()[legID * 3 + 1].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 1].kp() = 180;
//         low_cmd.motor_cmd()[legID * 3 + 1].kd() = 8;
//         low_cmd.motor_cmd()[legID * 3 + 2].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 2].kp() = 300;
//         low_cmd.motor_cmd()[legID * 3 + 2].kd() = 15;
//     }

//     // 实际站立增益设置
//     void setRealStanceGain(int legID){
//         low_cmd.motor_cmd()[legID * 3 + 0].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 0].kp() = 60;
//         low_cmd.motor_cmd()[legID * 3 + 0].kd() = 5;
//         low_cmd.motor_cmd()[legID * 3 + 1].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 1].kp() = 40;
//         low_cmd.motor_cmd()[legID * 3 + 1].kd() = 4;
//         low_cmd.motor_cmd()[legID * 3 + 2].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 2].kp() = 80;
//         low_cmd.motor_cmd()[legID * 3 + 2].kd() = 7;
//     }

//     // 设置增益为0
//     void setZeroGain(int legID){
//         low_cmd.motor_cmd()[legID * 3 + 0].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 0].kp() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 0].kd() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 1].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 1].kp() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 1].kd() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 2].mode() = (0x01);
//         low_cmd.motor_cmd()[legID * 3 + 2].kp() = 0;
//         low_cmd.motor_cmd()[legID * 3 + 2].kd() = 0;
//     }

//     // 所有腿设置稳定增益
//     void setZeroGain(){
//         for(int i(0); i<4; ++i){
//             setZeroGain(i);
//         }
//     }

//     // 摆动增益设置
//     void setStableGain(int legID){
//         low_cmd.motor_cmd()[legID*3+0].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+0].kp() = 0.8;
//         low_cmd.motor_cmd()[legID*3+0].kd() = 0.8;
//         low_cmd.motor_cmd()[legID*3+1].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+1].kp() = 0.8;
//         low_cmd.motor_cmd()[legID*3+1].kd() = 0.8;
//         low_cmd.motor_cmd()[legID*3+2].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+2].kp() = 0.8;
//         low_cmd.motor_cmd()[legID*3+2].kd() = 0.8;
//     }


//     void setStableGain(){
//         for(int i(0); i<4; ++i){
//             setStableGain(i);
//         }
//     }


//     void setSwingGain(int legID){
//         low_cmd.motor_cmd()[legID*3+0].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+0].kp() = 3;
//         low_cmd.motor_cmd()[legID*3+0].kd() = 2;
//         low_cmd.motor_cmd()[legID*3+1].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+1].kp() = 3;
//         low_cmd.motor_cmd()[legID*3+1].kd() = 2;
//         low_cmd.motor_cmd()[legID*3+2].mode() = (0x01);
//         low_cmd.motor_cmd()[legID*3+2].kp() = 3;
//         low_cmd.motor_cmd()[legID*3+2].kd() = 2;
//     }
// };

#endif  //LOWLEVELCMD_H