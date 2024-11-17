#pragma once

// System

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gamepad_controller
{
    class GamepadControllerNode : public rclcpp::Node
    {
    public:
        GamepadControllerNode();

    private:
        // Callback for joystick messages
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

        // Controller switching
        void switchController(const std::string &start_controller, const std::string &stop_controller);

        // Move to home position
        void moveToHome();

        // Gripper control
        void controlGripper(bool close);

        // Publish joint or Cartesian commands
        void publishJointCommand(const sensor_msgs::msg::Joy::SharedPtr msg);
        void publishCartesianCommand(const sensor_msgs::msg::Joy::SharedPtr msg);

        // Emergency stop
        void emergencyStop();

        // Publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;

        // Service client for controller switching
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;

        // State variables
        std::string current_controller_;
        bool motion_enabled_;
        bool jogging_mode_;
    };
} // gamepad_controller
