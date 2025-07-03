#include "d2_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<D2Node> d2_node = std::make_shared<D2Node>();

    try
    {
        d2_node->connectBoostSerial();
        rclcpp::Rate rate(1000);
        while(rclcpp::ok())
        {
            d2_node->loopCygParser();
            rate.sleep();
        }

        d2_node->disconnectBoostSerial();
    }
    catch (const rclcpp::exceptions::RCLError& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[D2 NODE ERROR] : %s", e.what());
    }

    rclcpp::shutdown();
}
