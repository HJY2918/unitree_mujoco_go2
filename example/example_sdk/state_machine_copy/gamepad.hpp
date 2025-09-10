#pragma once

#include <cmath>

namespace unitree::common
{

    // bytecode mapping for raw joystick data
    // 16b
    typedef union
    {
        struct
        {
            uint8_t R1 : 1; //1
            uint8_t L1 : 1; //2
            uint8_t start : 1; //3
            uint8_t select : 1; //4
            uint8_t R2 : 1; //5
            uint8_t L2 : 1; //6
            uint8_t F1 : 1; //7
            uint8_t F2 : 1; //8
            uint8_t A : 1; //A
            uint8_t B : 1; //B
            uint8_t X : 1; //X
            uint8_t Y : 1; //Y
            uint8_t up : 1; //
            uint8_t right : 1; //
            uint8_t down : 1; //
            uint8_t left : 1; //
        } components;
        uint16_t value;
    } xKeySwitchUnion; // 表示游戏手柄的按钮信息。

    // 40 Byte (now used 24B)
    typedef struct
    {
        uint8_t head[2];
        xKeySwitchUnion btn; //一个 xKeySwitchUnion 类型，用来表示按钮的状态
        float lx; // 遥感状态
        float rx;
        float ry;
        float L2;
        float ly;

        uint8_t idle[16];
    } xRockerBtnDataStruct; // 表示游戏手柄的按钮信息+摇杆的状态。

    typedef union
    {
        xRockerBtnDataStruct RF_RX;  //用来表示手柄的按钮和遥感的数据
        uint8_t buff[40]; // 用于存储 40 字节的手柄原始数据。
    } REMOTE_DATA_RX;

    class Button
    {
    public:
        Button() {}

        void update(bool state) //更新on_press，on_release，pressed三个布尔量
        {
            on_press = state ? state != pressed : false; //判断是否刚刚被按下，pressed存放的是之前的状态
            on_release = state ? false : state != pressed; // 是否刚刚被释放
            pressed = state;
        }

        bool pressed = false; //当前按钮是否按下。
        bool on_press = false; // 按钮是否刚刚被按下。
        bool on_release = false; // 按钮是否刚刚被释放。
    }; // 用于存储和更新每个按钮的状态信息，包括按钮是否被按下、是否刚刚被按下、是否刚刚被释放。

    class Gamepad
    {
    public:
        Gamepad() {}

        void update(xRockerBtnDataStruct &key_data)
        {
            lx = lx * (1 - smooth) + (std::fabs(key_data.lx) < dead_zone ? 0.0 : key_data.lx) * smooth;
            rx = rx * (1 - smooth) + (std::fabs(key_data.rx) < dead_zone ? 0.0 : key_data.rx) * smooth;
            ry = ry * (1 - smooth) + (std::fabs(key_data.ry) < dead_zone ? 0.0 : key_data.ry) * smooth;
            l2 = l2 * (1 - smooth) + (std::fabs(key_data.L2) < dead_zone ? 0.0 : key_data.L2) * smooth;
            ly = ly * (1 - smooth) + (std::fabs(key_data.ly) < dead_zone ? 0.0 : key_data.ly) * smooth;

            R1.update(key_data.btn.components.R1); 
            L1.update(key_data.btn.components.L1);
            start.update(key_data.btn.components.start);
            select.update(key_data.btn.components.select);
            R2.update(key_data.btn.components.R2);
            L2.update(key_data.btn.components.L2);
            F1.update(key_data.btn.components.F1);
            F2.update(key_data.btn.components.F2);
            A.update(key_data.btn.components.A);
            B.update(key_data.btn.components.B);
            X.update(key_data.btn.components.X);
            Y.update(key_data.btn.components.Y);
            up.update(key_data.btn.components.up);
            right.update(key_data.btn.components.right);
            down.update(key_data.btn.components.down);
            left.update(key_data.btn.components.left);
        }

        float lx = 0.;
        float rx = 0.;
        float ry = 0.;
        float l2 = 0.;
        float ly = 0.;

        float smooth = 0.03;
        float dead_zone = 0.01;

        Button R1;
        Button L1;
        Button start;
        Button select;
        Button R2;
        Button L2;
        Button F1;
        Button F2;
        Button A;
        Button B;
        Button X;
        Button Y;
        Button up;
        Button right;
        Button down;
        Button left;
    }; // 表示整个游戏手柄的状态，包括所有按钮和摇杆的状态，并通过 update() 方法更新手柄数据。
} // namespace unitree::common