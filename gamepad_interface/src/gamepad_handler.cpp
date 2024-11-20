#include "gamepad_interface/gamepad_handler.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace gamepad_interface
{
    GamepadHandler::GamepadHandler()
        : Node("gamepad_handler_node"),
          motion_enabled_(false)
    {
        // Declare and load the whitelist parameter
        this->declare_parameter<std::vector<std::string>>("controller_whitelist", {}, rcl_interfaces::msg::ParameterDescriptor{});
        this->get_parameter("controller_whitelist", whitelisted_controllers_);

        switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
        list_controllers_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");

        listAvailableControllers();

        RCLCPP_INFO(this->get_logger(), "GamepadHandler initialized.");
    }

    void GamepadHandler::listAvailableControllers()
    {
        if (!list_controllers_client_->wait_for_service(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "List controllers service not available.");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto result = list_controllers_client_->async_send_request(request);

        // Spin until the future is complete
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            try
            {
                auto response = result.get();
                available_controllers_.clear();

                for (const auto &controller : response->controller)
                {
                    if (std::find(whitelisted_controllers_.begin(), whitelisted_controllers_.end(), controller.name) != whitelisted_controllers_.end())
                    {
                        available_controllers_.push_back(controller.name);
                        RCLCPP_DEBUG(this->get_logger(), "Switchable controller: %s", controller.name.c_str());
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "Ignored controller: %s", controller.name.c_str());
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error processing response: %s", e.what());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call list controllers service.");
        }
    }

    void GamepadHandler::switchController(const std::string &start_controller, const std::string &stop_controller)
    {
        if (!switch_controller_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Switch controller service not available");
            return;
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
                RCLCPP_INFO(this->get_logger(), "Switched controllers successfully.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to switch controllers.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while switching controllers: %s", e.what());
        }
    }

    void GamepadHandler::handleInput(const GamepadInput &input, const ButtonMapping &button_mapping)
    {
        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.deadman_switch) &&
            input.buttons[button_mapping.deadman_switch] == 1)
        {
            motion_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Motion enabled.");
        }

        if (input.buttons.size() > static_cast<std::size_t>(button_mapping.switch_controller) &&
            input.buttons[button_mapping.switch_controller] == 1)
        {
            if (available_controllers_.size() > 1)
            {
                static size_t current_index = 0;
                size_t next_index = (current_index + 1) % available_controllers_.size();
                switchController(available_controllers_[next_index], available_controllers_[current_index]);
                current_index = next_index;

                RCLCPP_INFO(this->get_logger(),
                            "Switched to controller: %s", available_controllers_[current_index].c_str());
            }
        }
    }
}
