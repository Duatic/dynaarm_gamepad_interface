#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <string>
#include <vector>
#include <memory>

class ControllerHelper
{
public:
    explicit ControllerHelper(const rclcpp::Node::SharedPtr &node);

    // Update the list of available and active controllers
    void updateControllers();

    // Get the current active controllers
    std::vector<std::string> getActiveControllers() const;

    // Get the list of available controllers
    std::vector<std::string> getAvailableControllers() const;

    // Get the whitelisted controllers
    const std::vector<std::string>& getWhitelistedControllers() const;

    // Switch from one controller to another
    bool switchController(const std::string &start_controller, const std::string &stop_controller);

    // Check if a specific controller is active
    bool isControllerActive(const std::string &controller_name) const;

    // Check if a specific controller is available
    bool isControllerAvailable(const std::string &controller_name) const;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

    std::vector<std::string> available_controllers_;
    std::vector<std::string> active_controllers_;
    std::vector<std::string> whitelisted_controllers_;
};
