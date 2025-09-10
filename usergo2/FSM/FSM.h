/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

#pragma once   //一个非标准但广泛支持的预处理指令，确保头文件只会被包含一次，防止重复定义问题。

// FSM States
#include "FSMState.h"
// #include "State_FixedStand.h"
// #include "State_Passive.h"
// #include "State_FreeStand.h"
// #include "State_Trotting.h"
// #include "State_BalanceTest.h"
// #include "State_SwingTest.h"
// #include "State_StepTest.h"
#include "../common/enumClass.h"
//CtrlComponents 是一个控制组件类，负责整合机器人控制系统中的各个子系统。
#include "../control/CtrlComponents.h" 
// #ifdef COMPILE_WITH_MOVE_BASE
//     #include "tate_move_base.h"
// #endif  // COMPILE_WITH_MOVE_BASE


// struct FSMStateList{
//     FSMState *invalid;
//     State_Passive *passive;
//     State_FixedStand *fixedStand;
//     State_FreeStand *freeStand;
//     State_Trotting *trotting;
//     State_BalanceTest *balanceTest;
//     State_SwingTest *swingTest;
//     State_StepTest *stepTest;

// };

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};


#endif  // FSM_H