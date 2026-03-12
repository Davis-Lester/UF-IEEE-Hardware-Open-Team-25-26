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
    
    // Create a robust QoS profile to ensure commands aren't lost during startup
    rclcpp::QoS intake_qos(10);
    intake_qos.transient_local();
    intake_qos.reliable();
    this->intake_pub_ = this->create_publisher<std_msgs::msg::Int8>("/intake_cmd", intake_qos);

    this->start_light_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/start_light_detected", 10,
    std::bind(&AutonRoutine::start_light_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for competition start light...");

    // Check for start signal every 50ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&AutonRoutine::check_and_run, this));

    // --- NEW: Continuous Intake Publisher Timer (10Hz) ---
    intake_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AutonRoutine::intake_publish_callback, this));
}

void AutonRoutine::start_light_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !start_detected_) {
        start_detected_ = true;
        RCLCPP_INFO(this->get_logger(), "START LIGHT DETECTED!");
    }
}

AutonRoutine::~AutonRoutine() {
    // stop the intake if the auton routine stops for safety
    set_intake(0);

    if (routine_thread_.joinable()) {
        routine_thread_.join();
    }
}

void AutonRoutine::check_and_run() {
    if (start_detected_) {
        timer_->cancel();  // Stop checking
        
        // Spawn a new thread to run the blocking sequence off the main executor
        routine_thread_ = std::thread(&AutonRoutine::run_routine, this);
    }
}

// --- NEW: Update the state and let the timer handle the publishing ---
void AutonRoutine::set_intake(int state){
    current_intake_state_.store(state);
    intake_publish_callback(); // Publish immediately for zero-latency response
}

void AutonRoutine::intake_publish_callback() {
    auto msg = std_msgs::msg::Int8();
    msg.data = current_intake_state_.load();
    intake_pub_->publish(msg);
}
// ---------------------------------------------------------------------

// Updated to use the standardized 'target_value' and handle clean shutdowns
bool AutonRoutine::wait_for_drive(std::string mode, double target_value, double max_speed) {
        if (!rclcpp::ok()) return false; // Immediate safety check

        auto goal_msg = Drive::Goal();
        goal_msg.mode = mode;
        goal_msg.target_value = target_value; 
        goal_msg.max_speed = max_speed;

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return false;
        }

        auto send_goal_options = rclcpp_action::Client<Drive>::SendGoalOptions();
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        
        // Block the worker thread in small chunks, checking rclcpp::ok()
        auto timeout_time = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        std::future_status status;
        do {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(this->get_logger(), "Node shutting down, aborting wait for goal handle.");
                return false;
            }
            status = goal_handle_future.wait_for(std::chrono::milliseconds(100));
        } while (status != std::future_status::ready && std::chrono::steady_clock::now() < timeout_time);

        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for goal handle");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = this->client_ptr_->async_get_result(goal_handle);

        RCLCPP_INFO(this->get_logger(), "Executing %s...", mode.c_str());
        
        // Block the worker thread in small chunks, checking rclcpp::ok()
        timeout_time = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        do {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(this->get_logger(), "Node shutting down, aborting wait for action result.");
                return false;
            }
            status = result_future.wait_for(std::chrono::milliseconds(100));
        } while (status != std::future_status::ready && std::chrono::steady_clock::now() < timeout_time);

        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for action result");
            return false;
        }

        // Inspect the actual result code to ensure the action didn't abort/cancel
        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Action Complete.");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Action failed with result code: %d", static_cast<int>(result.code));
            return false;
        }
}

void AutonRoutine::run_routine() {
    // Note: timer is already canceled in check_and_run
    RCLCPP_INFO(this->get_logger(), "--- STARTING AUTONOMOUS ---");

    // 1. Drive Forward
    if (!wait_for_drive("DRIVE", 1000.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to drive forward. Aborting routine.");
        return;
    }

    // Turn on the intake (1 = forward)
    set_intake(1);
    RCLCPP_INFO(this->get_logger(), "Intake ON.");
    
    // 2. Fire IR (Antenna 3, Blue)
    auto msg = std_msgs::msg::UInt8();
    msg.data = 0x5C; 
    ir_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "IR Fired.");

    // 3. Turn 
    if (!wait_for_drive("TURN", 90.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to turn. Aborting routine.");
        set_intake(0); // Safety shutdown
        return;
    }

    // 4. Drive Backward
    if (!wait_for_drive("DRIVE", -1000.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to drive backward. Aborting routine.");
        set_intake(0); // Safety shutdown
        return;
    }

    // Stop the intake (0 = stop) at the end of the routine
    set_intake(0);
    RCLCPP_INFO(this->get_logger(), "--- AUTONOMOUS FINISHED ---");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonRoutine>();
    
    // Main executor spins here, processing callbacks and action server responses
    rclcpp::spin(node); 
    
    rclcpp::shutdown();
    return 0;
}