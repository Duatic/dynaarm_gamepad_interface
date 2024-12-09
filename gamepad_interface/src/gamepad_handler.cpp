#include "gamepad_interface/gamepad_handler.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace gamepad_interface
{
    GamepadHandler::GamepadHandler()
        : Node("gamepad_handler_node"),
          motion_enabled_(false)
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

    void GamepadHandler::handleInput(const GamepadInput &input, const ButtonMapping &button_mapping)
    {
        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.deadman_switch) &&
            input.buttons[button_mapping.deadman_switch] == 1)
        {   
            if (input.buttons[button_mapping.deadman_switch] == 1 && motion_enabled_ == false)
            {
                RCLCPP_INFO(this->get_logger(), "Motion enabled.");
            }
            
            motion_enabled_ = true;
        }
        else 
        {
            if (motion_enabled_) 
            {
                RCLCPP_INFO(this->get_logger(), "Motion disabled.");
            }

            motion_enabled_ = false;
        }    

        // // Handle controller switching
        // if (input.buttons.size() > static_cast<std::size_t>(button_mapping.switch_controller) &&
        //     input.buttons[button_mapping.switch_controller] == 1)
        // {
        //     if (is_switching_controller_) 
        //     {
        //         return;
        //     }

        //     controller_helper_->updateControllers();

        //     is_switching_controller_ = true;
        //     auto active_controllers = controller_helper_->getActiveControllers();
        //     auto available_controllers = controller_helper_->getAvailableControllers();

        //     if (active_controllers.empty() && available_controllers.size() > 1)
        //     {                
        //         RCLCPP_INFO(this->get_logger(), "Activating controller: %s", available_controllers[current_index].c_str());
        //         controller_helper_->activateController(available_controllers[current_index]);
        //     }
        //     else if (!active_controllers.empty() && available_controllers.size() > 1)
        //     {                
        //         size_t next_index = (current_index + 1) % available_controllers.size();

        //         RCLCPP_INFO(this->get_logger(), "Switching to controller: %s", available_controllers[current_index].c_str());
        //         controller_helper_->switchController(available_controllers[next_index], active_controllers[current_index]);
        //         current_index = next_index;
        //     }
        //     else 
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Can't switch to any controller.");
        //         is_switching_controller_ = false;
        //         return;
        //     }

        //     RCLCPP_INFO(this->get_logger(), "Controller switch: success");
        // }
        
        // if (input.buttons.size() > static_cast<std::size_t>(button_mapping.switch_controller) &&
        //     input.buttons[button_mapping.switch_controller] == 0 && 
        //     is_switching_controller_)
        // {
        //     is_switching_controller_ = false;
        // }
        

        if (motion_enabled_ == false)
        {
            return;
        }

        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.move_home) &&
            input.buttons[button_mapping.move_home] == 1)
        {
            controller_helper_->activateController("joint_trajectory_controller");
            // Move to home position
            publishJointTrajectory(std::vector<double>(joint_names_.size(), 0.0), 10.0); // Very slow speed

            // If we are moving to home, we don't need to do anything else.
            return;
        }

        if (controller_helper_->isControllerActive("joint_trajectory_controller") && !input.axes.empty())
        {
            // General joystick-based joint control
            std::vector<double> target_positions(joint_names_.size(), 0.0);
            double displacement_scale = 1.0; // Adjust sensitivity

            // Retrieve current joint positions from KinematicUtils or a similar source
            auto current_positions_map = kinematic_utils_->getCurrentJointPositions(); // Assuming such a method exists

            if (current_positions_map.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No joint states received yet. Cannot compute target positions.");
                return;
            }

            // Map current positions to target positions
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                const std::string &joint_name = joint_names_[i];
                if (current_positions_map.find(joint_name) != current_positions_map.end())
                {
                    target_positions[i] = current_positions_map[joint_name] + (input.axes[i] * displacement_scale);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Joint state for '%s' not found.", joint_name.c_str());
                }
            }

            // Publish the updated joint positions
            publishJointTrajectory(target_positions, 30.0); // Moderate speed
        }
    }
}
