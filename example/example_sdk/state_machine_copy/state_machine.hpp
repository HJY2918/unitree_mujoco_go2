#pragma once

#include <algorithm>

namespace unitree::common
{
    enum class STATES
    {
        DAMPING = 0, // 阻尼模式
        STAND = 1,  // 站立模式
        CTRL = 2    // 控制状态
    };


    class SimpleStateMachine
    {
    public:

        STATES state;
        double pd_ratio;
        double delta_pd;

        SimpleStateMachine(double pd_ratio_init = 0.1, double delta_pd = 0.005) // 默认参数
            : state(STATES::STAND), pd_ratio(pd_ratio_init), delta_pd(delta_pd) {}
        //state: 当前状态，初始为 STATES::STAND。
        // pd_ratio: 一个比例值，用于控制某些行为，初始为 0.1。
        // delta_pd: 控制 pd_ratio 增加或减少的步长，默认值为 0.005。
        bool Stop()
        {
            state = STATES::DAMPING;
            pd_ratio = 0.0;
            return true;
        }// 结束需要进入阻尼模式

        bool Stand()   //如果当前状态是 DAMPING 或 CTRL，则切换回 STAND 状态。
        {
            if (state == STATES::DAMPING || state == STATES::CTRL)
            {
                state = STATES::STAND;
                return true;
            }
            else
            {
                return false;
            }
        }

        bool Ctrl()  //只有在当前状态为 STAND 且 pd_ratio 大于 0.95 时，才会切换到 CTRL 状态。
        {
            if (state == STATES::STAND && pd_ratio > 0.95)
            {
                state = STATES::CTRL;
                return true;
            }
            else
            {
                return false;
            }
        }

        void Standing(bool up = true)  
        {
            if (state == STATES::STAND)
            {
                if (up)
                {
                    pd_ratio += delta_pd;
                }
                else
                {
                    pd_ratio -= delta_pd;
                }
                pd_ratio = std::max(0.0, std::min(1.0, pd_ratio)); //pd_ratio 始终在 [0.0, 1.0] 之间
            }
        }

    };
} // namespace unitree
