/*
 * Copyright 2025 Duatic AG
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

 #include <vector>
 #include <string>
 #include <rclcpp/rclcpp.hpp>
 #include "controller_helper.hpp"
 #include "gamepad_interface/gamepad_receiver.hpp"
 #include "gamepad_interface/kinematic_utils.hpp"
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <trajectory_msgs/msg/joint_trajectory.hpp>
 
 namespace gamepad_interface
 {
     /**
      * @brief The GamepadHandler node processes joystick input and sends joint trajectory commands to control the JTC arm.
      *
      * When at least one joystick axis is nonzero, a motion command is published. When no axis is active, a single stop (hold)
      * command is published.
      */
     class GamepadHandler : public rclcpp::Node
     {
     public:
         explicit GamepadHandler();
 
         /**
          * @brief Initializes the helper classes and retrieves joint names.
          */
         void init();
 
         /**
          * @brief Handles the input from the gamepad.
          *
          * @param input The gamepad input.
          * @param button_mapping The mapping for gamepad buttons.
          * @param axis_mapping The mapping for gamepad axes.
          */
         void handleInput(const GamepadInput &input, const ButtonMapping &button_mapping, const AxisMapping &axis_mapping);
 
     private:
         /**
          * @brief Processes joystick input for joint motion and publishes the corresponding trajectory.
          *
          * When any axis is active, a command is published. Otherwise, a stop (hold) command is published once.
          *
          * @param input The gamepad input.
          * @param axis_mapping Mapping for axes.
          * @param button_mapping Mapping for buttons.
          */
         void moveJoints(const GamepadInput &input, const AxisMapping &axis_mapping, const ButtonMapping &button_mapping);
 
         /**
          * @brief Publishes a JointTrajectory message using the provided target positions and speed percentage.
          *
          * @param target_positions The desired joint positions.
          * @param speed_percentage The speed as a percentage (clamped between 1 and 100).
          */
         void publishJointTrajectory(const std::vector<double> &target_positions, double speed_percentage);
 
         // Helper objects
         std::shared_ptr<KinematicUtils> kinematic_utils_;
         std::shared_ptr<ControllerHelper> controller_helper_;
 
         // ROS publishers
         rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
         rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_publisher_;
 
         // Persistent storage for joint positions
         std::vector<double> last_commanded_positions_;
         std::vector<double> last_joint_positions_;
 
         // Flag to indicate if joystick is currently idle (no moving axis)
         bool is_joystick_idle_ = true;
 
         // Flag to indicate if the arm is moving home (not used in current implementation)
         bool is_moving_home_ = false;
 
         // Gamepad button mapping (structure defined in gamepad_interface/gamepad_receiver.hpp)
         ButtonMapping button_mapping_;
 
         // Names of the joints to control
         std::vector<std::string> joint_names_;
     };
 }
 