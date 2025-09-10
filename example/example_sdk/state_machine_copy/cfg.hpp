#include "unitree/common/json/jsonize.hpp"
#include <vector>
#include <iostream>

// 配置文件configuration; 
// 继承自jsonize，能够实现将对象数据序列化为 JSON 格式，并能够从 JSON 字符串中解析出对象数据。（SDK）
namespace unitree::common
{
    class ExampleCfg : public Jsonize
    {
    public:
        ExampleCfg() : kp(0), kd(0), dt(0)
        {
        }

        void fromJson(JsonMap &json)
        {
            FromJson(json["kp"], kp);
            FromJson(json["kd"], kd);
            FromJson(json["dt"], dt);
            FromAny<float>(json["init_pos"], init_pos);
        }

        void toJson(JsonMap &json) const
        {
            ToJson(kp, json["kp"]);
            ToJson(kd, json["kd"]);
            ToJson(dt, json["dt"]);
            ToAny<float>(init_pos, json["init_pos"]);
        }

        float kp;
        float kd;
        float dt;

        std::vector<float> init_pos;
    };
}