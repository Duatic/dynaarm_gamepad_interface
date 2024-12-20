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

    // Activate controller
    bool activateController(const std::string &start_controller);

    // Activate controller
    bool deactivateController(const std::string &stop_controller);

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
