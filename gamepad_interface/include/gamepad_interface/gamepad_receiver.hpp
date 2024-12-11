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
        int wrist_rotation_left;
        int wrist_rotation_right;
        int move_home;        
    };

    struct AxisMapping
    {
        struct Joystick
        {
            int x; // Index for X-axis
            int y; // Index for Y-axis
        };

        struct Triggers
        {
            int left;  // Index for Left Trigger
            int right; // Index for Right Trigger
        };

        Joystick left_joystick;
        Joystick right_joystick;
        Triggers triggers;
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

        using InputCallback = std::function<void(const GamepadInput &, const ButtonMapping &, const AxisMapping &)>;
        void setInputCallback(InputCallback callback);

        const ButtonMapping &getButtonMapping() const; // Provide access to button mapping
        const AxisMapping &getAxisMapping() const;    // Provide access to axis mapping

    private:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void loadMappings();        

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        InputCallback input_callback_;
        ButtonMapping button_mapping_; // Stores the button mapping
        AxisMapping axis_mapping_;     // Stores the axis mapping
    };
}
