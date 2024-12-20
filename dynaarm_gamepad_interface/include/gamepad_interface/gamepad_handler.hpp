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

#include <mutex>

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
        void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping, const AxisMapping &axis_mapping);

    private:
        void moveCart();
        void moveJoints(const GamepadInput &input, const AxisMapping &axis_mapping, const ButtonMapping &button_mapping);
        void holdCurrentPosition();
        void publishCartesianTarget(const GamepadInput &input);
        void publishJointTrajectory(const std::vector<double> &target_positions, double speed_percentage);

        std::shared_ptr<KinematicUtils> kinematic_utils_;
        std::shared_ptr<ControllerHelper> controller_helper_;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_publisher_;

        bool motion_enabled_;
        bool is_moving_home_ = false;
        ButtonMapping button_mapping_;
        std::vector<std::string> joint_names_;
    };
}
