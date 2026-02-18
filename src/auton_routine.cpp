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

using Drive = hardware_team_robot::action::Drive;

class AutonRoutine : public rclcpp::Node {
public:
    AutonRoutine() : Node("auton_routine") {
        this->client_ptr_ = rclcpp_action::create_client<Drive>(this, "drive_command");
        this->ir_pub_ = this->create_publisher<std_msgs::msg::UInt8>("ir_command", 10);
        
        // Start the routine on a timer so the constructor finishes quickly
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&AutonRoutine::run_routine, this));
    }

private:
    rclcpp_action::Client<Drive>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ir_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- The "EZ" Wrapper Function ---
    void wait_for_drive(double ticks, std::string mode) {
        auto goal_msg = Drive::Goal();
        goal_msg.target_ticks = ticks;
        goal_msg.mode = mode;
        goal_msg.max_speed = 100;

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto send_goal_options = rclcpp_action::Client<Drive>::SendGoalOptions();
        
        // This is the "Blocking" part
        // In standard ROS 2, we usually use callbacks. 
        // For a linear script, we can use std::future to wait.
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        
        // Wait for server to accept
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Send goal failed");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        auto result_future = this->client_ptr_->async_get_result(goal_handle);

        // Wait for drive to complete
        RCLCPP_INFO(this->get_logger(), "Driving...");
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
        RCLCPP_INFO(this->get_logger(), "Drive Complete.");
    }

    void run_routine() {
        timer_->cancel(); // Only run once

        RCLCPP_INFO(this->get_logger(), "--- STARTING AUTONOMOUS ---");

        // 1. Drive Forward
        wait_for_drive(1000.0, "DRIVE");

        // 2. Fire IR (Antenna 3, Blue)
        auto msg = std_msgs::msg::UInt8();
        msg.data = 0x5C; 
        ir_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "IR Fired.");

        // 3. Turn 
        wait_for_drive(90.0, "TURN");

        // 4. Drive Backward
        wait_for_drive(-1000.0, "DRIVE");

        RCLCPP_INFO(this->get_logger(), "--- AUTONOMOUS FINISHED ---");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonRoutine>();
    rclcpp::spin(node); // This spin handles the "wait_for_drive" inner spins
    rclcpp::shutdown();
    return 0;
}