#include "hardware_team_robot/camera_node.h"

CameraProcessor::CameraProcessor() : Node("camera_processor") {
    // 1. Publisher to the IR Node
    ir_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("ir_command", 10);

    // 2. Action Server for Auton Routine
    action_server_ = rclcpp_action::create_server<FindColor>(
        this, "find_color_action",
        std::bind(&CameraProcessor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CameraProcessor::handle_cancel, this, std::placeholders::_1),
        std::bind(&CameraProcessor::handle_accepted, this, std::placeholders::_1));

    // 3. Subscription to Camera (Always running, but logic only active during Goal)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&CameraProcessor::image_callback, this, std::placeholders::_1));
    
    // intialize vector to check colors
    // color name, hsv range for color, hex code to be transmitted based on color
    targets_ = {
        {"RED", {
            {cv::Scalar(0, 100, 50),   cv::Scalar(10, 255, 255)},  
            {cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255)} 
        }, 0x09},
        {"GREEN", {{cv::Scalar(36, 100, 50),  cv::Scalar(85, 255, 255)}}, 0x0A},
        {"PURPLE", {{cv::Scalar(135, 100, 50), cv::Scalar(160, 255, 255)}}, 0x0F
        },
        {"BLUE", {{cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255)}}, 0x0C},
    };

    goal_active_ = false;

    // Create the standalone timeout timer (checks every 200ms)
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), 
        std::bind(&CameraProcessor::check_timeout, this)
    );
}

void CameraProcessor::check_timeout() {
    // safely exit if no goal is currently active or the handle is null
    if (!goal_active_ || active_goal_handle_ == nullptr) return;

    auto current_time = this->now();
    double elapsed_time = (current_time - start_time_).seconds();

    if (elapsed_time > timeout_seconds_) {
        RCLCPP_WARN(this->get_logger(), "FindColor Action timed out after %.1f seconds", timeout_seconds_);
        
        auto result = std::make_shared<FindColor::Result>();
        result->success = false; // indicate failure
        result->detected_color = "TIMEOUT";
        
        active_goal_handle_->abort(result); // use 'abort' for failure cases
        
        goal_active_ = false;
        active_goal_handle_ = nullptr;
    }
}

void CameraProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!goal_active_) return; // Only process if Auton asked for it
        
    try{

        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat hsv_frame;
        cv::cvtColor(cv_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);

        for (const auto& target : targets_) {
            cv::Mat combined_mask = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);

            for (const auto& r : target.ranges) {
                cv::Mat temp_mask;
                cv::inRange(hsv_frame, r.lower, r.upper, temp_mask);
                // conbines masks (this is necessary for red since it wraps around)
                cv::bitwise_or(combined_mask, temp_mask, combined_mask);
            }

            int pixel_count = cv::countNonZero(combined_mask);

            if (pixel_count > 500) {
                // 1. Create and Publish a String message for the IR Node
                auto ir_msg = std_msgs::msg::UInt8();
                ir_msg.data = target.colorCode; // e.g., "RED" or "GREEN"
                ir_publisher_->publish(ir_msg);
                RCLCPP_INFO(this->get_logger(), "Published detected color: %s", target.name.c_str());

                // 2. Prepare the Action Result
                auto result = std::make_shared<FindColor::Result>();
                result->success = true;
                
                result->detected_color = target.name;

                // 3. Send Success signal to the client (Auton Routine)
                active_goal_handle_->succeed(result);

                // 4. Reset state to stop processing images until the next goal
                goal_active_ = false;
                active_goal_handle_ = nullptr; // Clear the pointer for safety

                RCLCPP_INFO(this->get_logger(), "Goal Succeeded and state reset.");
                break; // Exit the loop since we found a match
            }
        }
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

// --- Action Server Boilerplate ---
rclcpp_action::GoalResponse CameraProcessor::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FindColor::Goal>) {
    if (goal_active_) {
        RCLCPP_WARN(this->get_logger(), "Rejecting FindColor goal while another goal is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void CameraProcessor::handle_accepted(const std::shared_ptr<GoalHandleFindColor> goal_handle) {
    goal_handle->execute();

    active_goal_handle_ = goal_handle;
    goal_active_ = true;
    start_time_ = this->now(); // Mark the start time
}

rclcpp_action::CancelResponse CameraProcessor::handle_cancel(const std::shared_ptr<GoalHandleFindColor> goal_handle) {
    // build the result object
    auto result = std::make_shared<FindColor::Result>();
    result->success = false;
    result->detected_color = "CANCELED"; // Give the client clear feedback
    
    // send the terminal canceled state to the client
    goal_handle->canceled(result);
    
    // reset internal state
    goal_active_ = false;
    active_goal_handle_ = nullptr;
    
    // acknowledge the cancel request
    return rclcpp_action::CancelResponse::ACCEPT;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // This line creates the node and keeps it alive
  auto node = std::make_shared<CameraProcessor>();
  
  RCLCPP_INFO(node->get_logger(), "Camera Node is spinning...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
