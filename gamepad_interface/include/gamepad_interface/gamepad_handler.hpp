#pragma once

#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "gamepad_interface/gamepad_receiver.hpp"

namespace gamepad_interface
{
    class GamepadHandler : public rclcpp::Node
    {
    public:
        explicit GamepadHandler();

        void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping);

    private:
        void switchController(const std::string &start_controller, const std::string &stop_controller);
        void listAvailableControllers();

        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

        bool motion_enabled_;
        ButtonMapping button_mapping_;

        std::vector<std::string> whitelisted_controllers_;
        std::vector<std::string> available_controllers_;
    };
}
