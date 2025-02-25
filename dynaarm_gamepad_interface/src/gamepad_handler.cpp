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
 #include <sstream>
 #include <cmath>
 
 using namespace std::chrono_literals;
 
 namespace gamepad_interface
 {
     GamepadHandler::GamepadHandler()
         : Node("gamepad_handler_node"), is_joystick_idle_(true)
     {
         joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
             "/joint_trajectory_controller/joint_trajectory", 10);
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
 
         // Update the list of available and active controllers.
         controller_helper_->updateControllers();
 
         RCLCPP_INFO(this->get_logger(), "GamepadHandler initialized.");
     }
 
     void GamepadHandler::publishJointTrajectory(const std::vector<double> &target_positions, double speed_percentage)
     {
         if (joint_names_.empty())
         {
             RCLCPP_ERROR(this->get_logger(), "Joint names are not set. Cannot send joint motion.");
             return;
         }
 
         if (target_positions.size() != joint_names_.size())
         {
             RCLCPP_ERROR(this->get_logger(), "Size mismatch between joint names and target positions.");
             return;
         }
 
         // Clamp speed percentage between 1 and 100.
         speed_percentage = std::clamp(speed_percentage, 1.0, 100.0);
 
         trajectory_msgs::msg::JointTrajectory trajectory_msg;
         trajectory_msg.joint_names = joint_names_;
 
         trajectory_msgs::msg::JointTrajectoryPoint point;
         point.positions = target_positions;
 
         // Compute time_from_start based on speed_percentage.
         double time_seconds = 50.0 / speed_percentage;
         int32_t sec = static_cast<int32_t>(std::floor(time_seconds));
         uint32_t nanosec = static_cast<uint32_t>((time_seconds - sec) * 1e9);
         point.time_from_start = rclcpp::Duration(sec, nanosec);
 
         trajectory_msg.points.push_back(point);
         joint_trajectory_publisher_->publish(trajectory_msg);
     }
 
     void GamepadHandler::moveJoints(const GamepadInput &input, const AxisMapping &axis_mapping, const ButtonMapping &button_mapping)
     {
         if (joint_names_.empty())
         {
             RCLCPP_ERROR(this->get_logger(), "Joint names are not set. Cannot send joint motion.");
             return;
         }
 
         auto current_positions_map = kinematic_utils_->getCurrentJointPositions();
         if (current_positions_map.empty())
         {
             RCLCPP_WARN(this->get_logger(), "No joint states received yet. Cannot compute target positions.");
             return;
         }
 
         // Lazy initialization: populate persistent vectors on first run.
         if (last_commanded_positions_.empty())
         {
             last_commanded_positions_.resize(joint_names_.size());
             last_joint_positions_.resize(joint_names_.size());
             for (size_t i = 0; i < joint_names_.size(); ++i)
             {
                 const std::string &joint_name = joint_names_[i];
                 last_commanded_positions_[i] = current_positions_map[joint_name];
                 last_joint_positions_[i] = current_positions_map[joint_name];
                 RCLCPP_DEBUG(this->get_logger(), "%s: Initial commanded position: %f", joint_name.c_str(), last_commanded_positions_[i]);
             }
         }
 
         bool any_axis_active = false;
         double displacement_scale = 1.0; // Adjust for sensitivity
 
         // Process each joint input.
         for (size_t i = 0; i < joint_names_.size(); ++i)
         {
             const std::string &joint_name = joint_names_[i];
             double cur_joint_val = current_positions_map[joint_name];
             double axis_val = 0.0;
 
             // Select the appropriate axis value for each joint.
             if (i == 0 && input.axes.size() > static_cast<size_t>(axis_mapping.left_joystick.x))
             {
                 axis_val = input.axes[axis_mapping.left_joystick.x];
             }
             else if (i == 1 && input.axes.size() > static_cast<size_t>(axis_mapping.left_joystick.y))
             {
                 axis_val = input.axes[axis_mapping.left_joystick.y];
             }
             else if (i == 2 && input.axes.size() > static_cast<size_t>(axis_mapping.right_joystick.y))
             {
                 axis_val = input.axes[axis_mapping.right_joystick.y];
             }
             else if (i == 3 && input.axes.size() > static_cast<size_t>(axis_mapping.right_joystick.x))
             {
                 axis_val = input.axes[axis_mapping.right_joystick.x];
             }
             else if (i == 4)
             {
                 double left_trigger  = (input.axes.size() > static_cast<size_t>(axis_mapping.triggers.left)) ? input.axes[axis_mapping.triggers.left] : 0.0;
                 double right_trigger = (input.axes.size() > static_cast<size_t>(axis_mapping.triggers.right)) ? input.axes[axis_mapping.triggers.right] : 0.0;
                 axis_val = right_trigger - left_trigger;
             }
             else if (i == 5)
             {
                 bool move_left  = (input.buttons.size() > static_cast<size_t>(button_mapping.wrist_rotation_left)) && input.buttons[button_mapping.wrist_rotation_left];
                 bool move_right = (input.buttons.size() > static_cast<size_t>(button_mapping.wrist_rotation_right)) && input.buttons[button_mapping.wrist_rotation_right];
                 if (move_left && !move_right)
                 {
                     axis_val = -1.0;
                 }
                 else if (move_right && !move_left)
                 {
                     axis_val = 1.0;
                 }
             }
 
             // Update command if the axis is active.
             if (axis_val != 0.0)
             {
                 last_commanded_positions_[i] = cur_joint_val + axis_val * displacement_scale;
                 last_joint_positions_[i] = cur_joint_val;
                 any_axis_active = true;
             }
             else
             {
                 // No input on this axis: hold the last measures position.
                 last_commanded_positions_[i] = last_joint_positions_[i];
             }
         }
 
         // Publish commands:
         // - If any axis is active, publish the new command and mark joystick as non-idle.
         // - Otherwise, if joystick was not already idle, send the last measured joint values (once).
         if (any_axis_active)
         {
             is_joystick_idle_ = false;
             publishJointTrajectory(last_commanded_positions_, 50.0);
         }
         else if (!is_joystick_idle_)
         {
             publishJointTrajectory(last_commanded_positions_, 50.0);
             is_joystick_idle_ = true;
         }
     }
 
     void GamepadHandler::handleInput(const GamepadInput &input, const ButtonMapping &button_mapping, const AxisMapping &axis_mapping)
     {
         if (!controller_helper_->isControllerActive("joint_trajectory_controller"))
         {
             return;
         }
 
         if (input.buttons.size() > static_cast<size_t>(button_mapping.move_home) && input.buttons[button_mapping.move_home] == 1)
         {
             publishJointTrajectory(std::vector<double>(joint_names_.size(), 0.0), 10.0);
             return;
         }
 
         if (input.axes.empty())
         {
             return;
         }
 
         moveJoints(input, axis_mapping, button_mapping);
     }
 } // namespace gamepad_interface
 