#include "hardware_team_robot/camera_node.h"

CameraProcessor::CameraProcessor() : Node("camera_processor") {
    // 1. Publisher to the IR Node
    ir_publisher_ = this->create_publisher<std_msgs::msg::String>("/ir_transmit_data", 10);

    // 2. Action Server for Auton Routine
    action_server_ = rclcpp_action::create_server<FindColor>(
        this, "find_color_action",
        std::bind(&CameraProcessor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CameraProcessor::handle_cancel, this, std::placeholders::_1),
        std::bind(&CameraProcessor::handle_accepted, this, std::placeholders::_1));

    // 3. Subscription to Camera (Always running, but logic only active during Goal)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&CameraProcessor::image_callback, this, std::placeholders::_1));
    
    goal_active_ = false;
}

void CameraProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!goal_active_) return; // Only process if Auton asked for it
    
    // check for timeout
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
        return;
    }

    
    try{

        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat hsv_frame;
        cv::cvtColor(cv_ptr->image, hsv_frame, cv::COLOR_BGR2HSV);

        std::vector<ColorTargets> targets = {
            {"RED", {
                {cv::Scalar(0, 100, 50),   cv::Scalar(10, 255, 255)},  // Lower Red
                {cv::Scalar(160, 100, 50), cv::Scalar(180, 255, 255)} // Upper Red
            }},
            {"GREEN", {{cv::Scalar(36, 100, 50),  cv::Scalar(85, 255, 255)}   // Single Green Range
            }},
            {"PURPLE", {{cv::Scalar(135, 100, 50), cv::Scalar(160, 255, 255)}
            }},
            {"BLUE", {{cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255)}
            }},
        };

        for (const auto& target : targets) {
            cv::Mat combined_mask = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);

            for (const auto& r : target.ranges) {
                cv::Mat temp_mask;
                cv::inRange(hsv_frame, r.lower, r.upper, temp_mask);
                // Combine masks: If a pixel is in ANY of the ranges, it stays white
                cv::bitwise_or(combined_mask, temp_mask, combined_mask);
            }

            int pixel_count = cv::countNonZero(combined_mask);

            if (pixel_count > 500) {
                // 1. Create and Publish a String message for the IR Node
                auto ir_msg = std_msgs::msg::String();
                ir_msg.data = target.name; // e.g., "RED" or "GREEN"
                ir_publisher_->publish(ir_msg);
                RCLCPP_INFO(this->get_logger(), "Published detected color: %s", target.name.c_str());

                // 2. Prepare the Action Result
                // Use the namespace defined in your header: hardware_team_robot::action::FindColor
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
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void CameraProcessor::handle_accepted(const std::shared_ptr<GoalHandleFindColor> goal_handle) {
    active_goal_handle_ = goal_handle;
    goal_active_ = true;
    start_time_ = this->now(); // Mark the start time
}

rclcpp_action::CancelResponse CameraProcessor::handle_cancel(const std::shared_ptr<GoalHandleFindColor>) {
    goal_active_ = false;
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
