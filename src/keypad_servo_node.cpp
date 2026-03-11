#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hardware_team_robot/keypad_servo.h"
#include <pigpio.h>
using std::placeholders::_1;

class KeypadServoNode : public rclcpp::Node
{

public:

    KeypadServoNode::KeypadServoNode()
: Node("keypad_servo_node"),
  servo_(12, 13)
{
    if (gpioInitialise() < 0) {
        RCLCPP_FATAL(this->get_logger(), "pigpio initialization failed");
        rclcpp::shutdown(); 
        return;             
    }

    servo_.initialize();    
    initialized_ = true;     

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/keypad_command",
        10,
        std::bind(&KeypadServoNode::command_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Dual Servo Keypad Node Started");
}

    KeypadServoNode::~KeypadServoNode()
{
    if (initialized_) {
        servo_.home();
        gpioTerminate();
    }
}

private:

    KeypadServo servo_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    bool initialized_ = false; 
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(),"Executing keypad command: %s",command.c_str());
        for(char digit : command)
{

    if(!isdigit(digit) && digit != '#')
        continue;

    servo_.press_digit(digit);

    rclcpp::sleep_for(std::chrono::milliseconds(250));
}
        servo_.home();
    }
};

int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);
    auto node = std::make_shared<KeypadServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
