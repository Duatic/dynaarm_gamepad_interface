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