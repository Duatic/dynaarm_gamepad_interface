#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "gamepad_interface/gamepad_receiver.hpp"

namespace gamepad_interface
{
    class GamepadHandler
    {
    public:
        GamepadHandler(const std::shared_ptr<rclcpp::Node> &node, const ButtonMapping &button_mapping);

        void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping);

    private:
        void moveToHome();
        void controlGripper(bool close);
        void emergencyStop();

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
        bool motion_enabled_;
        ButtonMapping button_mapping_;
    };
}
