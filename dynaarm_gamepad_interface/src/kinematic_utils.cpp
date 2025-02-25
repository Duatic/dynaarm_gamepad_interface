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

#include "gamepad_interface/kinematic_utils.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <rcpputils/join.hpp> // Include this header at the top of your file

KinematicUtils::KinematicUtils(const std::shared_ptr<rclcpp::Node> &node,
                               const std::string &base_frame,
                               const std::string &ee_frame)
    : node_(node), base_frame_(base_frame), ee_frame_(ee_frame)
{
    // Initialize the TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(node_->get_logger(), "TF listener initialized successfully.");

    // Load Pinocchio model from URDF
    std::string urdf_path = fetchParameter(node_, "/robot_state_publisher/get_parameters");
    pinocchio::urdf::buildModelFromXML(urdf_path, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    RCLCPP_INFO(node_->get_logger(), "Pinocchio model loaded. Number of joints: %d", pinocchio_model_.nq);

    // Subscribe to the joint_states topic
    joint_states_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&KinematicUtils::jointStatesCallback, this, std::placeholders::_1));
}

std::string KinematicUtils::fetchParameter(const std::shared_ptr<rclcpp::Node> &node, const std::string &parameter_name)
{
    auto client = node->create_client<rcl_interfaces::srv::GetParameters>(parameter_name);
    client->wait_for_service();

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description"};

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
        {
            return response->values[0].string_value;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "robot_description is not a string parameter");
        }
    }
    return "";
}

void KinematicUtils::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        current_joint_positions_[msg->name[i]] = msg->position[i];
    }
}

std::unordered_map<std::string, double> KinematicUtils::getCurrentJointPositions() const
{
    if (current_joint_positions_.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No joint positions available yet.");
    }
    return current_joint_positions_;
}

std::vector<std::string> KinematicUtils::getJointNames() const
{
    std::vector<std::string> joint_names;

    // Iterate through all joints in the Pinocchio model
    for (const auto &frame : pinocchio_model_.frames)
    {
        // Check if the frame type is a JOINT
        if (frame.type == pinocchio::FrameType::JOINT)
        {
            joint_names.push_back(frame.name); // Use the frame's name
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Retrieved joint names: %s", rcpputils::join(joint_names, ", ").c_str());
    return joint_names;
}

geometry_msgs::msg::PoseStamped KinematicUtils::getCurrentPose()
{
    geometry_msgs::msg::PoseStamped pose_msg;

    try
    {
        // Lookup the transform from base_frame to ee_frame
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(base_frame_, ee_frame_, rclcpp::Time(0), rclcpp::Duration(1, 0));

        // Convert TransformStamped to PoseStamped
        pose_msg.header = transform_stamped.header;
        pose_msg.pose.position.x = transform_stamped.transform.translation.x;
        pose_msg.pose.position.y = transform_stamped.transform.translation.y;
        pose_msg.pose.position.z = transform_stamped.transform.translation.z;
        pose_msg.pose.orientation = transform_stamped.transform.rotation;

        RCLCPP_DEBUG(node_->get_logger(),
                     "Retrieved pose from TF: x=%f, y=%f, z=%f",
                     pose_msg.pose.position.x,
                     pose_msg.pose.position.y,
                     pose_msg.pose.position.z);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s",
                    ee_frame_.c_str(), base_frame_.c_str(), ex.what());
    }

    return pose_msg;
}

geometry_msgs::msg::PoseStamped KinematicUtils::calculateForwardKinematics(const std::vector<double> &joint_positions)
{
    geometry_msgs::msg::PoseStamped pose_msg;

    if (joint_positions.size() != static_cast<size_t>(pinocchio_model_.nq))
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint positions size (%zu) does not match model size (%d).",
                     joint_positions.size(), pinocchio_model_.nq);
        return pose_msg;
    }

    // Map joint positions to Pinocchio configuration vector
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        q[i] = joint_positions[i];
    }

    // Perform forward kinematics
    pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q);
    pinocchio::SE3 ee_pose = pinocchio_data_.oMi[pinocchio_model_.getFrameId(ee_frame_)];

    // Convert SE3 to geometry_msgs::PoseStamped
    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.header.frame_id = base_frame_;

    pose_msg.pose.position.x = ee_pose.translation()[0];
    pose_msg.pose.position.y = ee_pose.translation()[1];
    pose_msg.pose.position.z = ee_pose.translation()[2];

    Eigen::Quaterniond quat(ee_pose.rotation());
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    RCLCPP_INFO(node_->get_logger(),
                "Forward kinematics result: x=%f, y=%f, z=%f",
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z);

    return pose_msg;
}
