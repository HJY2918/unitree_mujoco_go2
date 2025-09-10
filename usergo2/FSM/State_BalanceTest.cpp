/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "State_BalanceTest.h"

State_BalanceTest::State_BalanceTest(CtrlComponents *ctrlComp)
                  :FSMState(FSMStateName::BALANCETEST, "balanceTest", ctrlComp), // 父类 FSMState 的初始化
                  _estimator(ctrlComp->estimator), _robot_model(ctrlComp->robotModel),   // 成员变量的初始化：估计器，机器人模型
                  _balance_ctrl(ctrlComp->balCtrl), _wave_generator_(ctrlComp.waveGen), // 平衡控制器，波
                  _contact(ctrlComp->contact){     // 接触传感器

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(150, 150, 150).asDiagonal(); // 表示三个方向（x、y、z）的控制增益，对角矩阵
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200; // 旋转控制的比例增益
    _Kdw = Vec3(30, 30, 30).asDiagonal(); // 旋转控制的微分增益
}

void State_BalanceTest::enter(){
    _pcdInit = _estimator->getPosition();
    _pcd = _pcdInit;
    _RdInit = _lowState->getRotMat();

    // _ctrlComp->setAllStance();
    _wave_generator_._waveStatus= WaveStatus::STANCE_ALL;
    //  _waveStatus = WaveStatus::STANCE_ALL;
    // _ctrlComp->ioInter->zeroCmdPanel();
}

void State_BalanceTest::run(){
    _userValue = _lowState->userValue;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.ly, _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue.lx, _yMin, _yMax);
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);

    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;

    _posBody = _estimator->getPosition();
    _velBody = _estimator->getVelocity();

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();

    _lowCmd->setStableGain();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_q);
}

void State_BalanceTest::exit(){
    // _ctrlComp->ioInter->zeroCmdPanel();
    _wave_generator_._waveStatus= WaveStatus::SWING_ALL;
}

FSMStateName State_BalanceTest::checkChange(){ // 键盘输入
    if(_lowState->userCmd == UserCommand::L2_B){
        // return FSMStateName::PASSIVE;
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::BALANCETEST;
    }
}

void State_BalanceTest::calcTau(){

    // expected body acceleration
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    // expected body angular acceleration
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

     // calculate foot force
    _posFeet2BGlobal = _estimator->getPosFeet2BGlobal();

    _forceFeetGlobal = - _balance_ctrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robot_model->getTau(_q, _forceFeetBody);
}