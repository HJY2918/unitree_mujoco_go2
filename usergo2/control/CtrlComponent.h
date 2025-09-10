#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include <string>
#include <utility>
// #include "unitree/idl/go2/LowState_.hpp"
// #include "unitree/idl/go2/LowCmd_.hpp"
#include "../message/LowlevelCmd.h"
#include "../message/LowlevelState.h"

#include "../interface/robot_interface.hpp"
#include "../interface/CmdPanel.h"

#include "../common/unitreeRobotgo2.h"
#include "../Gait/WaveGenerator.h"
#include "../control/Estimator.h"
#include "../control/BalanceCtrl.h"

struct CtrlComponents{
public:
    CtrlComponents(RobotInterface *robot_interface):robot_interface(robot_interface){//构造函数使用初始化列表将传入的参数 robot_interface 赋值给成员变量 robot_interface
        lowCmd = new LowlevelCmd(); //为指针 lowCmd 分配新的 LowlevelCmd 对象。这意味着 lowCmd 是指向动态分配的 LowlevelCmd 实例的指针，通常用于控制底层命令。
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete robot_interface;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    RobotInterface *robot_interface;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;
    
    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    // void sendRecv(){
    //     robot_interface->GetState();
    // }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);
    }

// private:
//     WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};


#endif //CTRLCOMPONENTS_H