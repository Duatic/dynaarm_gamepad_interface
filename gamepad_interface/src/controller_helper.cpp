#include "gamepad_interface/controller_helper.hpp"

ControllerHelper::ControllerHelper(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    // Load the whitelist parameter
    node_->declare_parameter<std::vector<std::string>>("controller_whitelist", {});
    node_->get_parameter("controller_whitelist", whitelisted_controllers_);

    if (whitelisted_controllers_.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "No controllers are whitelisted. All controllers will be ignored.");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Whitelisted controllers: %zu", whitelisted_controllers_.size());
        for (const auto &controller : whitelisted_controllers_)
        {
            RCLCPP_INFO(node_->get_logger(), "- %s", controller.c_str());
        }
    }
}

void ControllerHelper::updateControllers()
{
    if (!list_controllers_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "List controllers service not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = list_controllers_client_->async_send_request(request);

    // Wait for the service response
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call list controllers service.");
        return;
    }

    try
    {
        auto response = future.get();
        available_controllers_.clear();
        active_controllers_.clear();

        for (const auto &controller : response->controller)
        {
            if (std::find(whitelisted_controllers_.begin(), whitelisted_controllers_.end(), controller.name) != whitelisted_controllers_.end())
            {
                available_controllers_.push_back(controller.name);
                if (controller.state == "active")
                {
                    active_controllers_.push_back(controller.name);
                }
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Updated controllers. Available: %zu, Active: %zu",
                    available_controllers_.size(), active_controllers_.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error processing response: %s", e.what());
    }
}

std::vector<std::string> ControllerHelper::getActiveControllers() const
{
    return active_controllers_;
}

std::vector<std::string> ControllerHelper::getAvailableControllers() const
{
    return available_controllers_;
}

const std::vector<std::string>& ControllerHelper::getWhitelistedControllers() const
{
    return whitelisted_controllers_;
}

bool ControllerHelper::isControllerActive(const std::string &controller_name) const
{
    return std::find(active_controllers_.begin(), active_controllers_.end(), controller_name) != active_controllers_.end();
}

bool ControllerHelper::isControllerAvailable(const std::string &controller_name) const
{
    return std::find(available_controllers_.begin(), available_controllers_.end(), controller_name) != available_controllers_.end();
}

bool ControllerHelper::switchController(const std::string &start_controller, const std::string &stop_controller)
{
    if (!switch_controller_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Switch controller service not available.");
        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {start_controller};
    request->deactivate_controllers = {stop_controller};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto future = switch_controller_client_->async_send_request(request);

    try
    {
        auto response = future.get();
        if (response->ok)
        {
            RCLCPP_INFO(node_->get_logger(), "Switched controllers successfully: activated '%s', deactivated '%s'.",
                        start_controller.c_str(), stop_controller.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to switch controllers.");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error switching controllers: %s", e.what());
    }

    return false;
}