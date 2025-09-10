#include <libevdev/libevdev.h> //libevdev 库的头文件，提供了与输入设备（例如键盘、鼠标等）交互的 API。
#include <fcntl.h>  //提供文件控制操作，例如 open() 用于打开设备文件
#include <iostream> //用于标准输入输出，例如 std::cout 输出信息到终端。
#include <unistd.h> //提供 POSIX 操作系统 API 的接口，如 close() 用于关闭文件描述符

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

// 处理键盘按下事件，将相应的位设置为1
void keyDown(int code, uint16_t &key) {
    switch (code) {
        case KEY_1:
            key |= (1 << 0);
            break;
        case KEY_2:
            key |= (1 << 1);
            break;
        case KEY_3:
            key |= (1 << 2);
            break;
        case KEY_4:
            key |= (1 << 3);
            break;
        case KEY_5:
            key |= (1 << 4);
            break;
        case KEY_6:
            key |= (1 << 5);
            break;
        case KEY_7:
            key |= (1 << 6);
            break;
        case KEY_8:
            key |= (1 << 7);
            break;
        case KEY_A:
            key |= (1 << 8);
            break;
        case KEY_B:
            key |= (1 << 9);
            break;
        case KEY_X:
            key |= (1 << 10);
            break;
        case KEY_Y:
            key |= (1 << 11);
            break;
        case KEY_UP:
            key |= (1 << 12);
            break;
        case KEY_DOWN:
            key |= (1 << 13);
            break;
        case KEY_LEFT:
            key |= (1 << 14);
            break;
        case KEY_RIGHT:
            key |= (1 << 15);
            break;
        default:
            break;
    }
}

// 处理键盘松开事件，将相应的位设置为0
void keyUp(int code, uint16_t &key) {
    switch (code) {
        case KEY_1:
            key &= ~(1 << 0);
            break;
        case KEY_2:
            key &= ~(1 << 1);
            break;
        case KEY_3:
            key &= ~(1 << 2);
            break;
        case KEY_4:
            key &= ~(1 << 3);
            break;
        case KEY_5:
            key &= ~(1 << 4);
            break;
        case KEY_6:
            key &= ~(1 << 5);
            break;
        case KEY_7:
            key &= ~(1 << 6);
            break;
        case KEY_8:
            key &= ~(1 << 7);
            break;
        case KEY_A:
            key &= ~(1 << 8);
            break;
        case KEY_B:
            key &= ~(1 << 9);
            break;
        case KEY_X:
            key &= ~(1 << 10);
            break;
        case KEY_Y:
            key &= ~(1 << 11);
            break;
        case KEY_UP:
            key &= ~(1 << 12);
            break;
        case KEY_DOWN:
            key &= ~(1 << 13);
            break;
        case KEY_LEFT:
            key &= ~(1 << 14);
            break;
        case KEY_RIGHT:
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
    struct libevdev *dev = nullptr;
    int fd = open("/dev/input/event8", O_RDONLY);  // 使用 event7 作为键盘设备文件
    if (fd < 0) {
        std::cerr << "无法打开设备文件" << std::endl;
        return 1;
    }

    int rc = libevdev_new_from_fd(fd, &dev);
    if (rc < 0) {
        std::cerr << "无法初始化 evdev 设备" << std::endl;
        return 1;
    }

    std::cout << "设备名称: " << libevdev_get_name(dev) << std::endl;

    unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> joystick_puber;
    joystick_puber.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_puber->InitChannel();
    
    unitree_go::msg::dds_::WirelessController_  joystick;

    xKeySwitchUnion key;

    while (true) {
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == 0) {
            if (ev.type == EV_KEY) {
                if (ev.value == 1) {  // 按下事件
                    // std::cout << "按下: " << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                    keyDown(ev.code, key.value);  // 处理按键按下
                    printKeyPressed(key);
                    joystick.keys() = key.value;
                    joystick_puber->Write(joystick);
                } else if (ev.value == 0) {  // 松开事件
                    // std::cout << "松开: " << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                    keyUp(ev.code, key.value);  // 处理按键松开
                    // printKeyPressed(key);
                    joystick.keys() = key.value;
                    joystick_puber->Write(joystick);
                } else if (ev.value == 2) {  // 持续按住事件
                    // std::cout << "持续: " << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                    joystick_puber->Write(joystick);
                }
            }
        }
    }

    libevdev_free(dev);
    close(fd);
    return 0;
}
