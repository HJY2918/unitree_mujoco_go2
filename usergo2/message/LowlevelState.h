/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "../common/mathTypes.h"
#include "../common/mathTools.h"
#include "../interface/CmdPanel.h"
#include "../common/enumClass.h"

struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[12];
    UserCommand userCmd; // 用户输入
    UserValue userValue;

    Vec34 getQ(){
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34 getQd(){
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat(){
        return imu.getRotMat();
    }

    Vec3 getAcc(){
        return imu.getAcc();
    }

    Vec3 getGyro(){
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

// #include "unitree/idl/go2/LowState_.hpp" // 新增


// struct IMU
// {
//     float quaternion[4];    // w, x, y, z
//     float gyroscope[3];
//     float accelerometer[3];

//     IMU(){
//         for(int i = 0; i < 3; i++){
//             quaternion[i] = 0;
//             gyroscope[i] = 0;
//             accelerometer[i] = 0;
//         }
//         quaternion[3] = 0;
//     }

//     RotMat getRotMat(){
//         Quat quat;
//         quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
//         return quatToRotMat(quat);
//     }

//     Vec3 getAcc(){
//         Vec3 acc;
//         acc << accelerometer[0], accelerometer[1], accelerometer[2];
//         return acc;
//     }

//     Vec3 getGyro(){
//         Vec3 gyro;
//         gyro << gyroscope[0], gyroscope[1], gyroscope[2];
//         return gyro;
//     }

//     Quat getQuat(){
//         Quat q;
//         q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
//         return q;
//     }


// };

// struct LowlevelState
// {
//     unitree_go::msg::dds_::LowState_ state;  // 使用新的 LowState 接口
//     // 从接口获取 IMU 数据
//     IMU getIMU() {
//         IMU imu;
//         const auto& imu_data = state.imu_state();
//         for (int i = 0; i < 4; i++) {
//             imu.quaternion[i] = imu_data.quaternion()[i];
//         }
//         for (int i = 0; i < 3; i++) {
//             imu.gyroscope[i] = imu_data.gyroscope()[i];
//             imu.accelerometer[i] = imu_data.accelerometer()[i];
//         }
//         return imu;
//     }

//     IMU imu=getIMU();
//     // MotorState motorState[12];
//     // UserCommand userCmd;
//     // UserValue userValue;

//     Vec34 getQ(){
//         Vec34 qLegs;
//         for(int i(0); i < 4; ++i){
//             qLegs.col(i)(0) =  state.motor_state()[3*i    ].q();
//             qLegs.col(i)(1) =  state.motor_state()[3*i + 1].q();
//             qLegs.col(i)(2) =  state.motor_state()[3*i + 2].q();
//         }
//         return qLegs;
//     }

//     Vec34 getQd(){
//         Vec34 qdLegs;
//         for(int i(0); i < 4; ++i){
//             qdLegs.col(i)(0) =  state.motor_state()[3*i    ].dq();
//             qdLegs.col(i)(1) =  state.motor_state()[3*i + 1].dq();
//             qdLegs.col(i)(2) =  state.motor_state()[3*i + 2].dq();
//         }
//         return qdLegs;
//     }

//     RotMat getRotMat(){
//         return imu.getRotMat();
//     }

//     Vec3 getAcc(){
//         return imu.getAcc();
//     }

//     Vec3 getGyro(){
//         return imu.getGyro();
//     }

//     Vec3 getAccGlobal(){
//         return getRotMat() * getAcc();
//     }

//     Vec3 getGyroGlobal(){
//         return getRotMat() * getGyro();
//     }

//     Vec3 getRPY(){
//         return rotMatToRPY(getRotMat());
//     }
//     Vec3 getRPYD(){
//         return getGyroGlobal();
//     }
//     // Eigen::Quaterniond get_quad()
//     // {
//     //     Quat q=imu.getQuat();
//     //     Eigen::Quaterniond m(q(0),q(1),q(2),q(3));
//     //     return m;
//     // }
//     Vec3 quatToRPY()
//     {
//         Quat QUA=imu.getQuat();
//         double as = std::min(-2. * (QUA[1] * QUA[3] - QUA[0] * QUA[2]), .99999);
//         Vec3 RPY_END;
//         RPY_END(2) =std::atan2(2 * (QUA[1] * QUA[2] + QUA[0] * QUA[3]),QUA[0]*QUA[0] + QUA[1]*QUA[1] - QUA[2]*QUA[2]- QUA[3]*QUA[3]);
//         RPY_END(1) = std::asin(as);
//         RPY_END(0) =std::atan2(2 * (QUA[2] * QUA[3] + QUA[0] * QUA[1]),QUA[0]*QUA[0] - QUA[1]*QUA[1] - QUA[2]*QUA[2] + QUA[3]*QUA[3]);
        
//         return RPY_END;       
//     }

//     double getYaw(){
//         return rotMatToRPY(getRotMat())(2);
//     }

//     double getDYaw(){
//         return getGyroGlobal()(2);
//     }

//     void setQ(Vec12 q){
//         for(int i(0); i<12; ++i){
//             state.motor_state()[i].q() = q(i);
//         }
//     }
// };

#endif  //LOWLEVELSTATE_HPP