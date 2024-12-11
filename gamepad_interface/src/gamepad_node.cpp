#pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"

#include "gamepad_interface/gamepad_receiver.hpp"
#include "gamepad_interface/gamepad_handler.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto receiver = std::make_shared<gamepad_interface::GamepadReceiver>();
    auto handler = std::make_shared<gamepad_interface::GamepadHandler>();
    handler->init();

    // Set the input callback to pass data from receiver to handler
    receiver->setInputCallback(
        [handler](const gamepad_interface::GamepadInput &input, 
                    const gamepad_interface::ButtonMapping &button_mapping,
                    const gamepad_interface::AxisMapping &axis_mapping)
        {
            handler->handleInput(input, button_mapping, axis_mapping);
        });

    // Run the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(receiver);
    executor.add_node(handler);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
