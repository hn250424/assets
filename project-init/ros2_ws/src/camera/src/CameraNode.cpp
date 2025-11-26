#include "camera/CameraNode.hpp"

using namespace std::chrono_literals;

CameraNode::CameraNode()
    : Node("camera_node"), frame_count_(0), loop_rate_(1s), startGenerator_(false)
{
    pub_ = this->create_publisher<std_msgs::msg::String>("camera/image", 10);

    action_client_ = rclcpp_action::create_client<Generator>(
        this,
        "measure_weight"
    );

    RCLCPP_INFO(this->get_logger(), "Camera node started");
}

void CameraNode::start()
{
    if (!startGenerator_)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        if (action_client_->wait_for_action_server(5s))
        {
            RCLCPP_INFO(this->get_logger(), "Action server connected!");
            startGenerator_ = true;
            send_weight_measurement();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }
    }

    // Use a wall timer to periodically publish image frames.
    // This approach is preferred over calling spin_some in a loop,
    // because spin_some only processes callbacks while it is running, 
    // which may cause delays or missed updates for periodic tasks.
    // Wall timers integrate better with ROS 2's executor, avoid busy-waiting,
    // and ensure callbacks are called at consistent intervals.
    timer_ = this->create_wall_timer(
        loop_rate_.period(),
        std::bind(&CameraNode::publish_image_frame, this));
}

void CameraNode::publish_image_frame()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Image Frame_" + std::to_string(frame_count_++);

    RCLCPP_INFO(this->get_logger(), "Capture and Publishing Image: '%s'", msg.data.c_str());
    pub_->publish(msg);
}

void CameraNode::send_weight_measurement()
{
    if (!action_client_->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Action server not ready, skipping measurement");
        return;
    }

    auto goal_msg = Generator::Goal();
    goal_msg.goal = 0.9;
    auto send_goal_options = rclcpp_action::Client<Generator>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleGenerator::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Goal rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "✅ Goal accepted by server");
            this->current_goal_handle_ = goal_handle;
        }
    };

    send_goal_options.feedback_callback =
        [this](GoalHandleGenerator::SharedPtr goal_handle,
               const std::shared_ptr<const Generator::Feedback> feedback)
    {
        auto w = feedback->weight;
        RCLCPP_INFO(this->get_logger(), "⚖️  Weight feedback: %.3f", w);

        if (w < 0.1 && this->current_goal_handle_)
        {
            // We launch a separate thread to call async_cancel_goal to avoid blocking the feedback callback
            // and to safely handle any potential executor/threading issues.
            std::thread([this]() {
                RCLCPP_WARN(this->get_logger(), "⚠️ Weight < 0.1 → sending cancel request (async thread)");
                action_client_->async_cancel_goal(this->current_goal_handle_);
             }).detach();
        }
    };

    send_goal_options.result_callback =
        [this](const GoalHandleGenerator::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "🎉 Measurement completed! Success: %s",
                        result.result->success ? "YES ✓" : "NO ✗");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "❌ Measurement was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "⚠️  Measurement was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "❓ Unknown result code");
            break;
        }
    };

    auto goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}