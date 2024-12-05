#pragma once

#include <rclcpp/rclcpp.hpp>
#include "controller_helper.hpp"

#include "gamepad_interface/gamepad_receiver.hpp"
#include "gamepad_interface/kinematic_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace gamepad_interface
{
    class GamepadHandler : public rclcpp::Node
    {
    public:
        explicit GamepadHandler();

        void init();
        void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping);

    private:
        void publishCartesianTarget(const GamepadInput &input);
        void publishJointTrajectory(const std::vector<double> &target_positions, double speed_percentage);

        std::shared_ptr<KinematicUtils> kinematic_utils_;
        std::shared_ptr<ControllerHelper> controller_helper_;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_publisher_;

        bool motion_enabled_;
        ButtonMapping button_mapping_;
        std::vector<std::string> joint_names_;
    };
}
