/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "mathTypes.h"
#include "enumClass.h"

// 四足机器人 *腿部* 运动学和动力学计算

class QuadrupedLeg{
public:
    // legID 0-3
    // abadLinkLength, hipLinkLength, kneeLinkLength 腿的各部分长度
    // pHip2B 髋部到身体坐标系的向量
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    // 正运动学
    Vec3 calcPEe2H(Vec3 q); // 计算从髋关节到脚尖的位移
    Vec3 calcPEe2B(Vec3 q); // 计算从身体坐标系到脚尖的位移。
    Vec3 calcVEe(Vec3 q, Vec3 qd);  // 计算脚尖的速度
    // 逆运动学
    Vec3 calcQ(Vec3 pEe, FrameType frame);  //该函数根据脚的位置 pEe 反向计算关节角度 q， ，并通过 FrameType 指定坐标系（髋部或身体）。
    Vec3 calcQd(Vec3 q, Vec3 vEe);  // 计算给定速度下的关节速度。
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
    // 根据足端力，计算关节力矩
    Vec3 calcTau(Vec3 q, Vec3 force);
    // 雅可比矩阵用于将关节角速度转换为脚的线速度，或者在逆动力学中用来计算关节力矩。
    Mat3 calcJaco(Vec3 q);  
    // 获取髋部到身体坐标系的向量
    Vec3 getHip2B(){return _pHip2B;}
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg{
public:
    A1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{
public:
    Go1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
    ~Go1Leg(){}
};

class Go2Leg : public QuadrupedLeg{
public:
    Go2Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0955, 0.213, 0.213, pHip2B){}
    ~Go2Leg(){}
};

#endif  // UNITREELEG_H