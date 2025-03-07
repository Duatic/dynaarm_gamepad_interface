/*
 * Copyright 2024 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
