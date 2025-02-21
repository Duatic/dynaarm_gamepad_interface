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

#include "gamepad_interface/gamepad_handler.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace gamepad_interface
{
    GamepadHandler::GamepadHandler()
        : Node("gamepad_handler_node")
    {
        cartesian_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cartesian_motion_controller/target_frame", 10);
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    }

    void GamepadHandler::init()
    {
        controller_helper_ = std::make_shared<ControllerHelper>(this->shared_from_this());
        kinematic_utils_ = std::make_shared<KinematicUtils>(this->shared_from_this(), "base", "END_EFFECTOR");

        joint_names_ = kinematic_utils_->getJointNames();

        if (joint_names_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve joint names from KinematicUtils. Motion commands will fail.");
        }

        // Update the list of available and active controllers
        controller_helper_->updateControllers();

        RCLCPP_INFO(this->get_logger(), "GamepadHandler initialized.");
    }

    void GamepadHandler::publishCartesianTarget(const GamepadInput &input)
    {
        if (input.axes.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No joystick input received.");
            return;
        }

        // Fetch the current pose using KinematicUtils
        geometry_msgs::msg::PoseStamped target_pose_stamped = kinematic_utils_->getCurrentPose();

        // Scale joystick input to Cartesian displacement
        double displacement_scale = 0.01;                                          // Adjust for desired sensitivity
        target_pose_stamped.pose.position.x += input.axes[0] * displacement_scale; // X-axis (left/right)
        target_pose_stamped.pose.position.y += input.axes[1] * displacement_scale; // Y-axis (forward/backward)
        target_pose_stamped.pose.position.z += input.axes[2] * displacement_scale; // Z-axis (up/down)

        // Publish the message
        cartesian_pose_publisher_->publish(target_pose_stamped);

        RCLCPP_DEBUG(this->get_logger(), "Published Cartesian target: position(x: %f, y: %f, z: %f)",
                     target_pose_stamped.pose.position.x, target_pose_stamped.pose.position.y, target_pose_stamped.pose.position.z);
    }

    void GamepadHandler::publishJointTrajectory(const std::vector<double> &target_positions, double speed_percentage)
    {
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Joint names are not set. Cannot send joint motion.");
            return;
        }

        // Ensure the sizes match between joint names and target positions
        if (target_positions.size() != joint_names_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Size mismatch between joint names and target positions.");
            return;
        }

        // Clamp speed percentage between 1% and 100%
        speed_percentage = std::clamp(speed_percentage, 1.0, 100.0);

        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;

        // Calculate time_from_start based on speed percentage
        double time_seconds = 50.0 / speed_percentage;

        // Convert seconds to rclcpp::Duration
        int32_t sec = static_cast<int32_t>(std::floor(time_seconds));
        uint32_t nanosec = static_cast<uint32_t>((time_seconds - sec) * 1e9);
        point.time_from_start = rclcpp::Duration(sec, nanosec);

        trajectory_msg.points.push_back(point);
        joint_trajectory_publisher_->publish(trajectory_msg);
    }

    void GamepadHandler::holdCurrentPosition()
    {
        // Retrieve current joint positions from KinematicUtils or a similar source
        auto current_positions_map = kinematic_utils_->getCurrentJointPositions();

        if (current_positions_map.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet. Cannot compute target positions.");
            return;
        }

        std::vector<double> target_positions(joint_names_.size(), 0.0);

        // Map current positions to target positions
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const std::string &joint_name = joint_names_[i];
            if (current_positions_map.find(joint_name) != current_positions_map.end())
            {
                target_positions[i] = current_positions_map[joint_name];
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Joint state for '%s' not found.", joint_name.c_str());
            }
        }

        // Publish the updated joint positions
        publishJointTrajectory(target_positions, 10.0);
    }

    void GamepadHandler::moveJoints(const GamepadInput &input, const AxisMapping &axis_mapping, const ButtonMapping &button_mapping)
    {
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Joint names are not set. Cannot send joint motion.");
            return;
        }

        // Retrieve current joint positions from KinematicUtils
        auto current_positions_map = kinematic_utils_->getCurrentJointPositions();

        if (current_positions_map.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet. Cannot compute target positions.");
            return;
        }

        std::vector<double> target_positions(joint_names_.size(), 0.0);
        double displacement_scale = 1.0; // Adjust for sensitivity

        // Map joystick inputs to joint positions based on axis mapping
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const std::string &joint_name = joint_names_[i];

            if (current_positions_map.find(joint_name) == current_positions_map.end())
            {
                RCLCPP_WARN(this->get_logger(), "Joint state for '%s' not found.", joint_name.c_str());
                continue;
            }

            if (i == 0 && input.axes.size() > static_cast<size_t>(axis_mapping.left_joystick.x))
            {
                target_positions[i] = current_positions_map[joint_name] + (input.axes[axis_mapping.left_joystick.x] * displacement_scale);
            }
            else if (i == 1 && input.axes.size() > static_cast<size_t>(axis_mapping.left_joystick.y))
            {
                target_positions[i] = current_positions_map[joint_name] + (input.axes[axis_mapping.left_joystick.y] * displacement_scale);
            }
            else if (i == 2 && input.axes.size() > static_cast<size_t>(axis_mapping.right_joystick.y))
            {
                target_positions[i] = current_positions_map[joint_name] + (input.axes[axis_mapping.right_joystick.y] * displacement_scale);
            }
            else if (i == 3 && input.axes.size() > static_cast<size_t>(axis_mapping.right_joystick.x))
            {
                target_positions[i] = current_positions_map[joint_name] + (input.axes[axis_mapping.right_joystick.x] * displacement_scale);
            }
            else if (i == 4)
            {
                double left_trigger = (input.axes.size() > static_cast<size_t>(axis_mapping.triggers.left)) ? input.axes[axis_mapping.triggers.left] : 0.0;
                double right_trigger = (input.axes.size() > static_cast<size_t>(axis_mapping.triggers.right)) ? input.axes[axis_mapping.triggers.right] : 0.0;
                target_positions[i] = current_positions_map[joint_name] + ((right_trigger - left_trigger) * displacement_scale);
            }
            else if (i == 5)
            {
                bool move_left = (input.buttons.size() > static_cast<size_t>(button_mapping.wrist_rotation_left)) && input.buttons[button_mapping.wrist_rotation_left];
                bool move_right = (input.buttons.size() > static_cast<size_t>(button_mapping.wrist_rotation_right)) && input.buttons[button_mapping.wrist_rotation_right];

                if (move_left)
                {
                    target_positions[i] = current_positions_map[joint_name] - displacement_scale;
                }
                else if (move_right)
                {
                    target_positions[i] = current_positions_map[joint_name] + displacement_scale;
                }
                else
                {
                    target_positions[i] = current_positions_map[joint_name];
                }
            }
        }

        // Publish the updated joint positions
        publishJointTrajectory(target_positions, 50.0);
    }

    void GamepadHandler::handleInput(const GamepadInput &input, const ButtonMapping &button_mapping, const AxisMapping &axis_mapping)
    {
        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.move_home) &&
            input.buttons[button_mapping.move_home] == 1)
        {
            // Move to home position
            publishJointTrajectory(std::vector<double>(joint_names_.size(), 0.0), 10.0);
            return;
        }

        if (controller_helper_->isControllerActive("joint_trajectory_controller") && !input.axes.empty())
        {
            moveJoints(input, axis_mapping, button_mapping);
        }
    }
}
