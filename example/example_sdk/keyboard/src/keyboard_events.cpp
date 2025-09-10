#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

int main() {
    struct libevdev *dev = nullptr;
    int fd = open("/dev/input/event7", O_RDONLY);  // 使用 event7 作为键盘设备文件
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

    while (true) {
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == 0) {
            if (ev.type == EV_KEY) {
                if (ev.value == 1) {  // 按下事件
                    std::cout << "按下: " << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                } else if (ev.value == 0) {  // 松开事件
                    std::cout << "松开: " << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                }else if (ev.value ==2){
                    std::cout << "持续:" << libevdev_event_code_get_name(ev.type, ev.code) << std::endl;
                }
            }
        }
    }

    libevdev_free(dev);
    close(fd);
    return 0;
}
