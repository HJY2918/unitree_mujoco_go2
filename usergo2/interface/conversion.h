#pragma once

#include <array>
#include "../common/comm.h"
#include "unitree/idl/go2/LowCmd_.hpp"

namespace unitree::common {

    // 函数声明
    void motorCmd2Dds(UNITREE_LEGGED_SDK::MotorCmd *raw, std::array<unitree_go::msg::dds_::MotorCmd_, 20> &dds);

    void bmsCmd2Dds(UNITREE_LEGGED_SDK::BmsCmd &raw, unitree_go::msg::dds_::BmsCmd_ &dds);

    uint32_t crc32_core(uint32_t *ptr, uint32_t len);

    void lowCmd2Dds(UNITREE_LEGGED_SDK::LowCmd &raw, unitree_go::msg::dds_::LowCmd_ &dds);

} // namespace unitree::common
