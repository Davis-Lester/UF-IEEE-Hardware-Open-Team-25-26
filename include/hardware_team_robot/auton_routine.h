#ifndef AUTON_ROUTINE_H
#define AUTON_ROUTINE_H

// Name: Davis Lester
// Date: 2/5/2026
// Description: Creating a Chassis PID Control Library for ROS2 Robot

//╭━━━╮╱╱╭╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━━━╮╱╱╱╱╱╭╮╱╱╱╱╱╭╮╱╭╮╱╱╱╭╮
//┃╭━╮┃╱╭╯╰╮╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱┃╭━╮┃╱╱╱╱╭╯╰╮╱╱╱╱┃┃╱┃┃╱╱╱┃┃
//┃┃╱┃┣╮┣╮╭╋━━┳━╮╭━━┳╮╭┳━━┳╮╭┳━━╮┃┃╱╰╋━━┳━╋╮╭╋━┳━━┫┃╱┃┃╱╱╭┫╰━┳━┳━━┳━┳╮╱╭╮
//┃╰━╯┃┃┃┃┃┃╭╮┃╭╮┫╭╮┃╰╯┃╭╮┃┃┃┃━━┫┃┃╱╭┫╭╮┃╭╮┫┃┃╭┫╭╮┃┃╱┃┃╱╭╋┫╭╮┃╭┫╭╮┃╭┫┃╱┃┃
//┃╭━╮┃╰╯┃╰┫╰╯┃┃┃┃╰╯┃┃┃┃╰╯┃╰╯┣━━┃┃╰━╯┃╰╯┃┃┃┃╰┫┃┃╰╯┃╰╮┃╰━╯┃┃╰╯┃┃┃╭╮┃┃┃╰━╯┃
//╰╯╱╰┻━━┻━┻━━┻╯╰┻━━┻┻┻┻━━┻━━┻━━╯╰━━━┻━━┻╯╰┻━┻╯╰━━┻━╯╰━━━┻┻━━┻╯╰╯╰┻╯╰━╮╭╯
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╭━╯┃
//╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╱╰━━╯

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hardware_team_robot/action/drive.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>

class AutonRoutine : public rclcpp::Node {
public:
    using Drive = hardware_team_robot::action::Drive;
    using GoalHandleDrive = rclcpp_action::ClientGoalHandle<Drive>;

    AutonRoutine();

private:
    // Clients & Publishers
    rclcpp_action::Client<Drive>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ir_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Routine Logic
    void run_routine();
    
    // Helper to mimic EZ-Template blocking calls
    void wait_for_drive(double ticks, std::string mode);
};

#endif // AUTON_ROUTINE_H