#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <utility>
#include "../control/CtrlComponents.h"
#include "../message/LowlevelCmd.h"
#include "../message/LowlevelState.h"
#include "../common/enumClass.h"

// 基类函数，后面各种状态动作的测试基于该函数实现
class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName &state_name, std::string state_name_string, CtrlComponent *ctrl_comp)
        : state_name(state_name),
          state_name_string(std::move(state_name_string)),
          ctrl_comp_(ctrl_comp) {
    }

    virtual void enter() = 0;

    virtual void run() = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    CtrlComponent *ctrl_comp_; // 控制组建

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;

    UserValue _userValue; // 用户输入（键盘或者手柄）
};

#endif //FSMSTATE_H
