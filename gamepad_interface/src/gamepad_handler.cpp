#include "gamepad_interface/gamepad_handler.hpp"

namespace gamepad_interface
{
    GamepadHandler::GamepadHandler(
        const std::shared_ptr<rclcpp::Node> &node,
        const ButtonMapping &button_mapping)
        : motion_enabled_(false),
          button_mapping_(button_mapping)
    {
        gripper_pub_ = node->create_publisher<std_msgs::msg::Bool>("/gripper_command", 10);
        joint_pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_command", 10);
    }

    void GamepadHandler::handleInput(const GamepadInput &input, const ButtonMapping &button_mapping)
    {
        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.deadman_switch) &&
            input.buttons[button_mapping.deadman_switch] == 1)
        {
            motion_enabled_ = true;
            RCLCPP_INFO(rclcpp::get_logger("GamepadHandler"), "Motion enabled.");
        }
    }

    void GamepadHandler::moveToHome()
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        traj_msg.points.push_back(point);

        joint_pub_->publish(traj_msg);
        RCLCPP_INFO(rclcpp::get_logger("GamepadHandler"), "Published home position trajectory.");
    }

    void GamepadHandler::controlGripper(bool close)
    {
        std_msgs::msg::Bool msg;
        msg.data = close;
        gripper_pub_->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("GamepadHandler"), close ? "Gripper closed." : "Gripper opened.");
    }

    void GamepadHandler::emergencyStop()
    {
        motion_enabled_ = false;
        RCLCPP_WARN(rclcpp::get_logger("GamepadHandler"), "Emergency stop activated.");
    }
}
