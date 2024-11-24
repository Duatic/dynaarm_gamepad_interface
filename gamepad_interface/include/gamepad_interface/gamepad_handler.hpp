#pragma once

#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "gamepad_interface/gamepad_receiver.hpp"
#include "gamepad_interface/kinematic_utils.hpp"

namespace gamepad_interface
{
    class GamepadHandler : public rclcpp::Node
    {
    public:
        explicit GamepadHandler();

        void init();
        void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping);

    private:
        void switchController(const std::string &start_controller, const std::string &stop_controller);
        void listAvailableControllers();

        void publishCartesianTarget(const GamepadInput &input);
        geometry_msgs::msg::Pose getCurrentPose();

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_publisher_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

        bool motion_enabled_;
        ButtonMapping button_mapping_;
        geometry_msgs::msg::Pose current_pose_;
        std::shared_ptr<KinematicUtils> kinematic_utils;

        std::vector<std::string> whitelisted_controllers_;
        std::vector<std::string> available_controllers_;
    };
}
