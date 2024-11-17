#include "gamepad_controller/gamepad_controller_node.hpp"

namespace gamepad_controller
{
    GamepadControllerNode::GamepadControllerNode()
        : Node("gamepad_controller_node"),
          current_controller_("joint_trajectory_controller"),
          motion_enabled_(false),
          jogging_mode_(true)
    {
        // Initialize subscribers and publishers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&GamepadControllerNode::joyCallback, this, std::placeholders::_1));        
    }

    void GamepadControllerNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
       std::cout << msg << std::endl;
    }

} // gamepad_controller

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gamepad_controller::GamepadControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
