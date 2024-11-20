#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <vector>

namespace gamepad_interface
{
    struct ButtonMapping
    {
        int deadman_switch;
        int stop_motion;
        int close_gripper;
        int open_gripper;
        int switch_controller;
        int move_home;
        std::vector<int> emergency_stop;
    };

    struct GamepadInput
    {
        std::vector<int> buttons;
        std::vector<float> axes;
    };

    class GamepadReceiver : public rclcpp::Node
    {
    public:
        explicit GamepadReceiver();

        using InputCallback = std::function<void(const GamepadInput &, const ButtonMapping &)>;
        void setInputCallback(InputCallback callback);

        const ButtonMapping &getButtonMapping() const; // Provide access to mapping

    private:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void loadButtonMappings();

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        InputCallback input_callback_;
        ButtonMapping button_mapping_; // Stores the button mapping
    };
}
