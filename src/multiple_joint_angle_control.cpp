#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};

    ///* HOME
    msg.data_() =
        "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
        "\"mode\":1,"
        "\"angle0\":-0.6,"
        "\"angle1\":-91,"
        "\"angle2\":88.5,"
        "\"angle3\":-90.5,"
        "\"angle4\":-2,"
        "\"angle5\":89.6,"
        "\"angle6\":-5.0"
        "}}";
    //*/

    /* LOW right
    msg.data_() =
        "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
        "\"mode\":1,"
        "\"angle0\":1.3,"
        "\"angle1\":-91.0,"
        "\"angle2\":88.4,"
        "\"angle3\":-89.5,"
        "\"angle4\":96,"
        "\"angle5\":95.6,"
        "\"angle6\":-5.0"
        "}}";

    */

    /* MID right
    msg.data_() =
        "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
        "\"mode\":1,"
        "\"angle0\":-0.7,"
        "\"angle1\":-59.6,"
        "\"angle2\":35.5,"
        "\"angle3\":-87.4,"
        "\"angle4\":93.6,"
        "\"angle5\":64.9,"
        "\"angle6\":-5.0"
        "}}";
    */

    /* HIGH right
    msg.data_() =
        "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
        "\"mode\":1,"
        "\"angle0\":-0.7,"
        "\"angle1\":-5.3,"
        "\"angle2\":-88.5,"
        "\"angle3\":-90.8,"
        "\"angle4\":90.4,"
        "\"angle5\":-3.2,"
        "\"angle6\":-5.0"
        "}}";
    */

    publisher.Write(msg);

    return 0;
}
