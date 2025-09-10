#include <mutex>
#include "unitree/common/thread/thread.hpp"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include "gamepad.hpp"

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace unitree::common;
using namespace unitree::robot;


class GamepadExample
{
public:
    GamepadExample() {}

   // setup dds model
   void InitDdsModel(const std::string &networkInterface = "")
   {
       ChannelFactory::Instance()->Init(1, networkInterface);
       joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));

       joystick_subscriber->InitChannel(std::bind(&GamepadExample::MessageHandler, this, std::placeholders::_1), 1);
   }


    // set gamepad dead_zone parameter
   void SetGamepadDeadZone(float deadzone)
   {
       gamepad.dead_zone = deadzone;
   }

   // set gamepad smooth parameter
   void setGamepadSmooth(float smooth)
   {
       gamepad.smooth = smooth;
   }

   // callback function to save joystick message

   // 消息监听回调
   // callback function to save joystick message
   void MessageHandler(const void *message)
   {
       std::lock_guard<std::mutex> lock(joystick_mutex);
       joystick_msg = *(unitree_go::msg::dds_::WirelessController_ *)message;
   }

    const unitree_go::msg::dds_::WirelessController_& getJoystickMsg() const {
        return joystick_msg;
    }

protected:
   ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
   unitree_go::msg::dds_::WirelessController_ joystick_msg;

   Gamepad gamepad;
   std::mutex joystick_mutex;

};