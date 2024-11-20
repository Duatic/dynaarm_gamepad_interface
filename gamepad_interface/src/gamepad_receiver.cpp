#include "gamepad_interface/gamepad_receiver.hpp"

namespace gamepad_interface
{
    GamepadReceiver::GamepadReceiver()
        : Node("gamepad_receiver_node")
    {
        // Load button mappings from parameters
        loadButtonMappings();

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

    void GamepadReceiver::loadButtonMappings()
    {
        button_mapping_.deadman_switch = this->declare_parameter<int>("button_mapping.deadman_switch", 9);
        button_mapping_.stop_motion = this->declare_parameter<int>("button_mapping.stop_motion", 10);
        button_mapping_.close_gripper = this->declare_parameter<int>("button_mapping.close_gripper", 11);
        button_mapping_.open_gripper = this->declare_parameter<int>("button_mapping.open_gripper", 12);
        button_mapping_.switch_controller = this->declare_parameter<int>("button_mapping.switch_controller", 13);
        button_mapping_.move_home = this->declare_parameter<int>("button_mapping.move_home", 5);
        auto emergency_stop_param = this->declare_parameter<std::vector<int>>("button_mapping.emergency_stop", {5, 6});
        button_mapping_.emergency_stop = std::vector<int>(emergency_stop_param.begin(), emergency_stop_param.end());

        // Log the loaded button mappings
        RCLCPP_DEBUG(this->get_logger(), "  Deadman Switch: %d", button_mapping_.deadman_switch);
        RCLCPP_DEBUG(this->get_logger(), "  Stop Motion: %d", button_mapping_.stop_motion);
        RCLCPP_DEBUG(this->get_logger(), "  Close Gripper: %d", button_mapping_.close_gripper);
        RCLCPP_DEBUG(this->get_logger(), "  Open Gripper: %d", button_mapping_.open_gripper);
        RCLCPP_DEBUG(this->get_logger(), "  Switch Controller: %d", button_mapping_.switch_controller);
        RCLCPP_DEBUG(this->get_logger(), "  Move Home: %d", button_mapping_.move_home);
        RCLCPP_DEBUG(this->get_logger(), "  Emergency Stop: [%s]", rcpputils::join(button_mapping_.emergency_stop, ", ").c_str());
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
