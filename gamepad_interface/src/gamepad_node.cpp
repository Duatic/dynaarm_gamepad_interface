#include "gamepad_interface/gamepad_receiver.hpp"
#include "gamepad_interface/gamepad_handler.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto receiver_node = std::make_shared<gamepad_interface::GamepadReceiver>("gamepad_receiver_node");
    const auto button_mapping = receiver_node->getButtonMapping();

    // Create the handler with the loaded button mappings
    auto handler = std::make_shared<gamepad_interface::GamepadHandler>(receiver_node, button_mapping);

    // Set the input callback
    receiver_node->setInputCallback(
        [handler](const gamepad_interface::GamepadInput &input, const gamepad_interface::ButtonMapping &button_mapping)
        {
            handler->handleInput(input, button_mapping);
        });

    // Spin the node
    rclcpp::spin(receiver_node);

    rclcpp::shutdown();
    return 0;
}
