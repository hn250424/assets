#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor/MotorNode.hpp"

using namespace std::chrono_literals;
using namespace std;

MotorNode::MotorNode() : 
    Node("motor_node"),
    gen_(std::random_device{}()),
    dist_(0, 10)
{
    srv_ = this->create_service<interfaces::srv::MoveMotor>(
        "move_motor_service",
        std::bind(&MotorNode::handleMotorRequest, this, std::placeholders::_1, std::placeholders::_2)
    );

    action_client_ = rclcpp_action::create_client<Generator>(
        this,
        "measure_weight"
    );
    
    RCLCPP_INFO(this->get_logger(), "Motor node service started");
}

void MotorNode::handleMotorRequest(
    const std::shared_ptr<interfaces::srv::MoveMotor::Request> request,
    std::shared_ptr<interfaces::srv::MoveMotor::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received request for image_no: %s", request->image_no.c_str());

    response->success = true;
    response->move_amount = dist_(gen_);

    RCLCPP_INFO(this->get_logger(), "Responding with success=true, move_amount=%d", response->move_amount);

    if (response->move_amount >= 8) {
        send_weight_measurement();
    }
}

void MotorNode::send_weight_measurement() {
    if (!action_client_->action_server_is_ready()) {
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
        }
    };

    send_goal_options.feedback_callback =
        [this](GoalHandleGenerator::SharedPtr goal_handle,
               const std::shared_ptr<const Generator::Feedback> feedback)
    {
        auto w = feedback->weight;
        RCLCPP_INFO(this->get_logger(), "⚖️  Weight feedback: %.3f", w);

        if (w < 0.1)
        {
            // We launch a separate thread to call async_cancel_goal to avoid blocking the feedback callback
            // and to safely handle any potential executor/threading issues.
            std::thread([this, goal_handle]() {
                RCLCPP_WARN(this->get_logger(), "⚠️ Weight < 0.1 → sending cancel request (async thread)");

                try {
                    auto status = goal_handle->get_status();
                    
                    if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        status == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                    {
                        action_client_->async_cancel_goal(goal_handle);
                        RCLCPP_INFO(this->get_logger(), "✅ Cancel request sent");
                    }
                    else {
                        RCLCPP_INFO(this->get_logger(), "ℹ️  Goal already in terminal state (status: %d), skipping cancel", static_cast<int>(status));
                    }

                } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e) {
                    RCLCPP_INFO(this->get_logger(), "ℹ️  Goal already completed before cancel: %s", e.what());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "❌ Error during cancel: %s", e.what());
                }
                
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