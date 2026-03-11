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

#include "hardware_team_robot/auton_routine.h"


AutonRoutine::AutonRoutine() : Node("auton_routine") {
    this->client_ptr_ = rclcpp_action::create_client<Drive>(this, "drive_command");
    this->ir_pub_ = this->create_publisher<std_msgs::msg::UInt8>("ir_command", 10);
    
    this->start_light_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/start_light_detected", 10,
    std::bind(&AutonRoutine::start_light_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for competition start light...");

    // Check for start signal every 50ms
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), 
    std::bind(&AutonRoutine::check_and_run, this));
}

void AutonRoutine::start_light_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !start_detected_) {
        start_detected_ = true;
        RCLCPP_INFO(this->get_logger(), "START LIGHT DETECTED!");
    }
}

void AutonRoutine::check_and_run() {
    if (start_detected_) {
        timer_->cancel();  // Stop checking
        run_routine();     // Start autonomous
    }
}


// Updated to use the standardized 'target_value'
void AutonRoutine::wait_for_drive(std::string mode, double target_value, double max_speed) {
        auto goal_msg = Drive::Goal();
        goal_msg.mode = mode;
        goal_msg.target_value = target_value; 
        goal_msg.max_speed = max_speed;

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto send_goal_options = rclcpp_action::Client<Drive>::SendGoalOptions();
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Send goal failed");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        auto result_future = this->client_ptr_->async_get_result(goal_handle);

        RCLCPP_INFO(this->get_logger(), "Executing %s...", mode.c_str());
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
        RCLCPP_INFO(this->get_logger(), "Action Complete.");
}

void AutonRoutine::run_routine() {
    timer_->cancel(); // Only run once

    RCLCPP_INFO(this->get_logger(), "--- STARTING AUTONOMOUS ---");

    // 1. Drive Forward
    wait_for_drive("DRIVE", 1000.0);

    // 2. Fire IR (Antenna 3, Blue)
    auto msg = std_msgs::msg::UInt8();
    msg.data = 0x5C; 
    ir_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "IR Fired.");

    // 3. Turn 
    wait_for_drive("TURN", 90.0);

    // 4. Drive Backward
    wait_for_drive("DRIVE", -1000.0);

    RCLCPP_INFO(this->get_logger(), "--- AUTONOMOUS FINISHED ---");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonRoutine>();
    rclcpp::spin(node); // This spin handles the "wait_for_drive" inner spins
    rclcpp::shutdown();
    return 0;
}