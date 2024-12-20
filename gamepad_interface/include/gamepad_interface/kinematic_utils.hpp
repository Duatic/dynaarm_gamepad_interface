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

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class KinematicUtils
{
public:
    KinematicUtils(const std::shared_ptr<rclcpp::Node> &node,
                   const std::string &base_frame,
                   const std::string &ee_frame);

    geometry_msgs::msg::PoseStamped getCurrentPose();
    geometry_msgs::msg::PoseStamped calculateForwardKinematics(const std::vector<double> &joint_positions);
    std::vector<std::string> getJointNames() const;
    std::unordered_map<std::string, double> getCurrentJointPositions() const;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string base_frame_;
    std::string ee_frame_;

    std::string fetchParameter(const std::shared_ptr<rclcpp::Node> &node, const std::string &parameter_name);

    // Joint state handling
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
    std::unordered_map<std::string, double> current_joint_positions_;
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // TF Buffer and Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Pinocchio Model and Data
    pinocchio::Model pinocchio_model_;
    pinocchio::Data pinocchio_data_;
};
