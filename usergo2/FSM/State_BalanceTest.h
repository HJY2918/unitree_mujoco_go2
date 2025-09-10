/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef BALANCETEST_H
#define BALANCETEST_H

#include "FSMState.h"

class State_BalanceTest final: public FSMState{
public:
    State_BalanceTest(CtrlComponents *ctrlComp);

    ~State_BalanceTest(){}

    void enter() override; // override 是 C++11 引入的一个关键字，用来显式地表明子类中的函数是对父类虚函数的重写。

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void calcTau();

    Estimator *_estimator;
    QuadrupedRobot *_robot_model;
    BalanceCtrl *_balance_ctrl;
    WaveGenerator *_wave_generator;

    VecInt4 *_contact;

    Vec3 _pcd, _pcdInit; // 机器人的期望位置信息及其初始值。
    RotMat _Rd, _RdInit; // 期望的旋转矩阵及其初始值，表示机器人的期望姿态。

    double _kpw;
    Mat3 _Kpp, _Kdp, _Kdw;
    Vec3 _ddPcd, _dWbd;

    Vec12 _q, _tau; // 机器人当前的关节角度和力矩。
    Vec3 _posBody, _velBody;    // 机器人的位置和速度。
    RotMat _B2G_RotMat, _G2B_RotMat; // 从机器人坐标系到全局坐标系的旋转矩阵，和其逆矩阵。
    Vec34 _posFeet2BGlobal; // 足端在全局和身体坐标系下的位置
    Vec34 _forceFeetGlobal, _forceFeetBody; // 、力反馈信息。

    float _xMax, _xMin; // 表示在平衡测试中，机器人在 x 方向上的位移范围。
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
};

#endif  // BALANCETEST_H