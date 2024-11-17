#include "gamepad_interface/gamepad_receiver.hpp"

namespace gamepad_interface
{
    GamepadReceiver::GamepadReceiver(const std::string &node_name)
        : Node(node_name)
    {
        // Load button mappings from parameters
        loadButtonMappings();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&GamepadReceiver::joyCallback, this, std::placeholders::_1));
    }

    void GamepadReceiver::setInputCallback(InputCallback callback)
    {
        input_callback_ = callback;
    }

    const ButtonMapping &GamepadReceiver::getButtonMapping() const
    {
        return button_mapping_;
    }

    void GamepadReceiver::loadButtonMappings()
    {
        button_mapping_.deadman_switch = this->declare_parameter<int>("button_mapping.deadman_switch", 9);
        button_mapping_.stop_motion = this->declare_parameter<int>("button_mapping.stop_motion", 10);
        button_mapping_.close_gripper = this->declare_parameter<int>("button_mapping.close_gripper", 11);
        button_mapping_.open_gripper = this->declare_parameter<int>("button_mapping.open_gripper", 12);
        button_mapping_.switch_controller = this->declare_parameter<int>("button_mapping.switch_controller", 13);
        button_mapping_.move_home = this->declare_parameter<int>("button_mapping.move_home", 5);

        auto emergency_stop_param = this->declare_parameter<std::vector<long>>("button_mapping.emergency_stop", {5, 6});
        button_mapping_.emergency_stop = std::vector<int>(emergency_stop_param.begin(), emergency_stop_param.end());

        RCLCPP_INFO(this->get_logger(), "Button mappings loaded successfully.");
    }

    void GamepadReceiver::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        GamepadInput input;
        input.buttons = msg->buttons;
        input.axes = msg->axes;

        if (input_callback_)
        {
            input_callback_(input, button_mapping_);
        }
    }
}
