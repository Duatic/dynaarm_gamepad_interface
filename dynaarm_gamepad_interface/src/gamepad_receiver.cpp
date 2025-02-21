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
