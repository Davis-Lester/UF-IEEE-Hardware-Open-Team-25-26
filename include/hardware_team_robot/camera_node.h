#pragma once
// THIS IS AN ACTION SERVER TO SEND LED COLOR TO IR NODE WHEN REQUESTED

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "hardware_team_robot/action/find_color.hpp"

struct HSVRange {
    cv::Scalar lower;
    cv::Scalar upper;
};

struct ColorTargets {
    std::string name;
    std::vector<HSVRange> ranges;
    uint8_t colorCode;
};

class CameraProcessor : public rclcpp::Node {
public:
    using FindColor = hardware_team_robot::action::FindColor;
    using GoalHandleFindColor = rclcpp_action::ServerGoalHandle<FindColor>;

    CameraProcessor();

private:
    // to time out if unresponsive
    rclcpp::Time start_time_;
    double timeout_seconds_ = 10.0; // 10 second limit
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    void check_timeout();

    // vector of colors to be checked by camera
    std::vector<ColorTargets> targets_;

    // Image processing
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Action Server Callbacks (Matches your .cpp signatures)
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FindColor::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFindColor> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleFindColor> goal_handle);

    // ROS Members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ir_publisher_;
    rclcpp_action::Server<FindColor>::SharedPtr action_server_;

    // State management
    std::shared_ptr<GoalHandleFindColor> active_goal_handle_;
    bool goal_active_;
};