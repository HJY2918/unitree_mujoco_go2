/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "unitreeLeg.h"
// #include "../message/LowlevelState.h"
#include "../interface/robot_interface.hpp"

// 以下是我RobotController类中的protecetd变量和函数
// unitree_go::msg::dds_::LowState_ state; 

// state = *(unitree_go::msg::dds_::LowState_ *)message;
//         {
//             std::lock_guard<std::mutex> lock(state_mutex); //mutex防止多个线程竞争
//             robot_interface.GetState(state); //获取电机和imu数据
//         }

// 以下是我robot_interface.hpp中RobotInterface类的GetState函数
// void GetState(unitree_go::msg::dds_::LowState_ &state) 
//         {
//             // imu
//             const unitree_go::msg::dds_::IMUState_ &imu = state.imu_state();
//             quat = imu.quaternion();
//             rpy = imu.rpy();
//             gyro = imu.gyroscope();
//             UpdateProjectedGravity();

//             // motor
//             const std::array<unitree_go::msg::dds_::MotorState_, 20> &motor = state.motor_state();
//             for (size_t i = 0; i < 12; ++i)
//             {
//                 const unitree_go::msg::dds_::MotorState_ &m = motor.at(i);
//                 jpos.at(i) = m.q();
//                 jvel.at(i) = m.dq();
//                 tau.at(i) = m.tau_est();
//             }
//         }

// std::array<float, 12> jpos, jvel, tau;
//         std::array<float, 4> quat;
//         std::array<float, 3> rpy, gyro, projected_gravity;
//         std::array<float, 12> jpos_des, jvel_des, kp, kd, tau_ff;

// 请逐一解释代码
// 我现在想要通过下面的函数计算第(0)腿的脚尖在身体坐标系下的位置，输入为state，但这个类型和上面的state类型不太一样
// Vec3 getX(LowlevelState &state);  

// 四足机器人*整体*的运动学和动力学计算，接口

class QuadrupedRobot{
public:
    QuadrupedRobot(const RobotInterface& robotInterface) : robotInterface(robotInterface){};
    ~QuadrupedRobot(){}

    // 返回第(0)腿的脚尖在身体坐标系下的位置
    // Vec3 getX(LowlevelState &state);   
    Vec3 getX(RobotInterface &robot_interface);       
    // 该函数计算并返回每条腿的脚尖相对于身体的偏移量（即从身体到脚尖的位置向量）。 
    // Vec34 getVecXP(LowlevelState &state);
    Vec34 getVecXP(RobotInterface &robot_interface);

    // Inverse Kinematics(Body/Hip Frame) 返回每条腿的逆运动学计算结果
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // Forward Kinematics
    // 基于关节角度，计算某条腿的脚尖位置，支持两种坐标系：身体坐标系和髋关节坐标系。
    // Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    // Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec3 getFootPosition(RobotInterface &robot_interface, int id, FrameType frame);
    Vec3 getFootVelocity(RobotInterface &robot_interface, int id);
    // 计算四条腿脚尖的速度。如果坐标系为全局（GLOBAL），则还考虑了旋转矩阵的影响。
    // Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    // Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);
    Vec34 getFeet2BPositions(RobotInterface &robot_interface, FrameType frame);
    Vec34 getFeet2BVelocities(RobotInterface &robot_interface, FrameType frame);

    // 计算某条腿的雅可比矩阵
    // Mat3 getJaco(LowlevelState &state, int legID);
    Mat3 getJaco(RobotInterface &robot_interface, int legID);

    
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    QuadrupedLeg* _Legs[4]; // "unitreeLeg.h" *腿部* 的正逆运动学&动力学
    Vec2 _robVelLimitX;     // 机器人的 X 方向速度 限制 （下限，上限）
    Vec2 _robVelLimitY;     // Y
    Vec2 _robVelLimitYaw;   // 表示机器人的 旋转速度 限制（绕 Z 轴的旋转）
    Vec34 _feetPosNormalStand; // 用于表示机器人的 四条腿的脚尖位置
    double _mass;   // 表示机器人的 质量
    Vec3 _pcb;  //表示机器人的 质心位置CoM
    Mat3 _Ib;   // 机器人的 惯性矩阵
};

class A1Robot : public QuadrupedRobot{
public:
    A1Robot();
    ~A1Robot(){}
};

class Go1Robot : public QuadrupedRobot{
public:
    Go1Robot();
    ~Go1Robot(){};
};

class Go2Robot : public QuadrupedRobot{
public:
    Go2Robot();
    ~Go2Robot(){};
};
#endif  // UNITREEROBOT_H