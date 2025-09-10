#include <iostream>
#include <cstdint>
#include <termios.h>
#include <unistd.h>


#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace unitree::common;

// 遥控器键值联合体
typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

// 进入非规范模式（无需回车直接读取输入）
void setNonCanonicalMode() {
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);  // 获取当前终端设置
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 关闭规范模式和回显模式
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 立即应用新设置
}

// 恢复终端设置
void restoreTerminalMode() {
    termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    oldt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// 处理键盘按下事件，将相应的位设置为1
void keyDown(char input, uint16_t &key) {
    switch (input) {
        case '1':
            key |= (1 << 0);
            break;
        case '2':
            key |= (1 << 1);
            break;
        case '3':
            key |= (1 << 2);
            break;
        case '4':
            key |= (1 << 3);
            break;
        case '5':
            key |= (1 << 4);
            break;
        case '6':
            key |= (1 << 5);
            break;
        case '7':
            key |= (1 << 6);
            break;
        case '8':
            key |= (1 << 7);
            break;
        case 'A': case 'a':
            key |= (1 << 8);
            break;
        case 'B': case 'b':
            key |= (1 << 9);
            break;
        case 'X': case 'x':
            key |= (1 << 10);
            break;
        case 'Y': case 'y':
            key |= (1 << 11);
            break;
        case 'I':  // 上
            key |= (1 << 12);
            break;
        case 'L':  // 下
            key |= (1 << 13);
            break;
        case 'K':  // 左
            key |= (1 << 14);
            break;
        case 'J':  // 右
            key |= (1 << 15);
            break;
        default:
            break;
    }
}

// 处理键盘松开事件，将相应的位设置为0
void keyUp(char input, uint16_t &key) {
    switch (input) {
        case '1':
            key &= ~(1 << 0);
            break;
        case '2':
            key &= ~(1 << 1);
            break;
        case '3':
            key &= ~(1 << 2);
            break;
        case '4':
            key &= ~(1 << 3);
            break;
        case '5':
            key &= ~(1 << 4);
            break;
        case '6':
            key &= ~(1 << 5);
            break;
        case '7':
            key &= ~(1 << 6);
            break;
        case '8':
            key &= ~(1 << 7);
            break;
        case 'A': case 'a':
            key &= ~(1 << 8);
            break;
        case 'B': case 'b':
            key &= ~(1 << 9);
            break;
        case 'X': case 'x':
            key &= ~(1 << 10);
            break;
        case 'Y': case 'y':
            key &= ~(1 << 11);
            break;
        case 'I':  // 上
            key &= ~(1 << 12);
            break;
        case 'L':  // 右
            key &= ~(1 << 13);
            break;
        case 'K':  // 下
            key &= ~(1 << 14);
            break;
        case 'J':  // 左
            key &= ~(1 << 15);
            break;
        default:
            break;
    }
}

void printKeyPressed(const xKeySwitchUnion &keyUnion) {
    if (keyUnion.components.R1) std::cout << "R1 pressed" << std::endl;
    if (keyUnion.components.L1) std::cout << "L1 pressed" << std::endl;
    if (keyUnion.components.start) std::cout << "Start pressed" << std::endl;
    if (keyUnion.components.select) std::cout << "Select pressed" << std::endl;
    if (keyUnion.components.R2) std::cout << "R2 pressed" << std::endl;
    if (keyUnion.components.L2) std::cout << "L2 pressed" << std::endl;
    if (keyUnion.components.F1) std::cout << "F1 pressed" << std::endl;
    if (keyUnion.components.F2) std::cout << "F2 pressed" << std::endl;
    if (keyUnion.components.A) std::cout << "A pressed" << std::endl;
    if (keyUnion.components.B) std::cout << "B pressed" << std::endl;
    if (keyUnion.components.X) std::cout << "X pressed" << std::endl;
    if (keyUnion.components.Y) std::cout << "Y pressed" << std::endl;
    if (keyUnion.components.up) std::cout << "Up pressed" << std::endl;
    if (keyUnion.components.right) std::cout << "Right pressed" << std::endl;
    if (keyUnion.components.down) std::cout << "Down pressed" << std::endl;
    if (keyUnion.components.left) std::cout << "Left pressed" << std::endl;
}

int main() {

    unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> joystick_puber;
    joystick_puber.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_puber->InitChannel();
    
    unitree_go::msg::dds_::WirelessController_  joystick;

    xKeySwitchUnion key;

    
    char input;

    setNonCanonicalMode();  // 设置为非规范模式

    std::cout << "按键输入监听 (1-8, A, B, X, Y, 上:W, 下:S, 左:A, 右:D):" << std::endl;
    
    while (true) {
        if (read(STDIN_FILENO, &input, 1) > 0) {
            if (input == '\033') { // 处理退出的 ESC 按键
                break;
            }
            keyDown(input, key.value);  // 处理按键按下
            printKeyPressed(key);
            joystick.keys() = key.value;
            //std::cout << "当前 key 的值: " << std::hex << key << std::endl;
            joystick_puber->Write(joystick);

            // 松开按键模拟，暂停等待0.5秒
            usleep(500000); // 暂停500ms模拟松开
            keyUp(input, key.value);  // 处理按键松开
            joystick.keys() = key.value;
            //std::cout << "松开后 key 的值: " << std::hex << key << std::endl;
            printKeyPressed(key);
            joystick_puber->Write(joystick);
            usleep(100000); // 暂停500ms模拟松开
        }
    }

    restoreTerminalMode();  // 恢复终端模式
    return 0;
}
