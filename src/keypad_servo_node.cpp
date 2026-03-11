#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "hardware_team_robot/keypad_servo.hpp"
#include <pigpio.h>
using std::placeholders::_1;

class KeypadServoNode : public rclcpp::Node
{

public:

    KeypadServoNode()
    : Node("keypad_servo_node"),
      servo_(12,13)
    {

        if(gpioInitialise() < 0)
        {
            RCLCPP_FATAL(this->get_logger(),"pigpio initialization failed");
            rclcpp::shutdown();
        }

        servo_.initialize();

        subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/keypad_command",
            10,
            std::bind(&KeypadServoNode::command_callback,this,_1)
        );

        RCLCPP_INFO(this->get_logger(),"Dual Servo Keypad Node Started");
    }

    ~KeypadServoNode()
    {

        servo_.home();

        gpioTerminate();
    }

private:

    KeypadServo servo_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {

        std::string command = msg->data;

        RCLCPP_INFO(this->get_logger(),"Executing keypad command: %s",command.c_str());

        for(char digit : command)
        {

            if(!isdigit(digit))
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
