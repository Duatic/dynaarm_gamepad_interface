#include "gamepad_interface/gamepad_receiver.hpp"

namespace gamepad_interface
{
    GamepadReceiver::GamepadReceiver()
        : Node("gamepad_receiver_node")
    {
        // Load mappings from parameters
        loadMappings();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&GamepadReceiver::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "GamepadReceiver initialized.");
    }

    void GamepadReceiver::setInputCallback(InputCallback callback)
    {
        input_callback_ = callback;
    }

    const ButtonMapping &GamepadReceiver::getButtonMapping() const
    {
        return button_mapping_;
    }

    const AxisMapping &GamepadReceiver::getAxisMapping() const
    {
        return axis_mapping_;
    }

    void GamepadReceiver::loadMappings()
    {
        // Load button mappings
        button_mapping_.deadman_switch = this->declare_parameter<int>("button_mapping.deadman_switch", 9);
        button_mapping_.move_home = this->declare_parameter<int>("button_mapping.move_home", 12);
        button_mapping_.wrist_rotation_left = this->declare_parameter<int>("button_mapping.wrist_rotation_left", 11);
        button_mapping_.wrist_rotation_right = this->declare_parameter<int>("button_mapping.wrist_rotation_right", 12);

        // Load axis mappings
        axis_mapping_.left_joystick.x = this->declare_parameter<int>("axis_mapping.left_joystick.x", 0);
        axis_mapping_.left_joystick.y = this->declare_parameter<int>("axis_mapping.left_joystick.y", 1);
        axis_mapping_.right_joystick.x = this->declare_parameter<int>("axis_mapping.right_joystick.x", 3);
        axis_mapping_.right_joystick.y = this->declare_parameter<int>("axis_mapping.right_joystick.y", 4);
        axis_mapping_.triggers.left = this->declare_parameter<int>("axis_mapping.triggers.left", 2);
        axis_mapping_.triggers.right = this->declare_parameter<int>("axis_mapping.triggers.right", 5);

        // Log loaded mappings
        RCLCPP_DEBUG(this->get_logger(), "Button Mappings:");
        RCLCPP_DEBUG(this->get_logger(), "  Deadman Switch: %d", button_mapping_.deadman_switch);
        RCLCPP_DEBUG(this->get_logger(), "  Move Home: %d", button_mapping_.move_home);
        RCLCPP_DEBUG(this->get_logger(), "  Wrist Rotation Left: %d", button_mapping_.wrist_rotation_left);
        RCLCPP_DEBUG(this->get_logger(), "  Wrist Rotation Right: %d", button_mapping_.wrist_rotation_right);        

        RCLCPP_DEBUG(this->get_logger(), "Axis Mappings:");
        RCLCPP_DEBUG(this->get_logger(), "  Left Joystick X: %d", axis_mapping_.left_joystick.x);
        RCLCPP_DEBUG(this->get_logger(), "  Left Joystick Y: %d", axis_mapping_.left_joystick.y);
        RCLCPP_DEBUG(this->get_logger(), "  Right Joystick X: %d", axis_mapping_.right_joystick.x);
        RCLCPP_DEBUG(this->get_logger(), "  Right Joystick Y: %d", axis_mapping_.right_joystick.y);
        RCLCPP_DEBUG(this->get_logger(), "  Left Trigger: %d", axis_mapping_.triggers.left);
        RCLCPP_DEBUG(this->get_logger(), "  Right Trigger: %d", axis_mapping_.triggers.right);

        RCLCPP_INFO(this->get_logger(), "Mappings loaded successfully.");
    }

    void GamepadReceiver::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        GamepadInput input;
        input.buttons = msg->buttons;
        input.axes = msg->axes;

        if (input_callback_)
        {
            input_callback_(input, button_mapping_, axis_mapping_);
        }
    }
}
