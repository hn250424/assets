#include <chrono>
#include <memory>
#include <random>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/generator.hpp"

using namespace std::chrono_literals;
using Generator = interfaces::action::Generator;
using GoalHandleGenerator = rclcpp_action::ServerGoalHandle<Generator>;

class GeneratorNode : public rclcpp::Node
{
public:
    GeneratorNode() : Node("Generator_node")
    {
        action_server_ = rclcpp_action::create_server<Generator>(
            this,
            "measure_weight",
            // Goal callback
            [this](const rclcpp_action::GoalUUID& uuid,
                   std::shared_ptr<const Generator::Goal> goal) {
                return handle_goal(uuid, goal);
            },
            // Cancel callback
            [this](const std::shared_ptr<GoalHandleGenerator> goal_handle) {
                return handle_cancel(goal_handle);
            },
            // Accepted callback
            [this](const std::shared_ptr<GoalHandleGenerator> goal_handle) {
                handle_accepted(goal_handle);
            }
        );

        rng_.seed(std::random_device{}());
        dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

        RCLCPP_INFO(this->get_logger(), "Generator action server started");
    }

private:
    rclcpp_action::Server<Generator>::SharedPtr action_server_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Generator::Goal> goal)
    {
        (void) uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal: %f", goal->goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGenerator> goal_handle)
    {
        (void) goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGenerator> goal_handle)
    {
        std::thread{[this](auto gh){ execute(gh); }, goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGenerator> goal_handle)
    {
        auto result = std::make_shared<Generator::Result>();
        auto feedback = std::make_shared<Generator::Feedback>();

        std::stringstream ss;
        for (auto c : goal_handle->get_goal_id()) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int) c;
        }
        std::string goal_id_str = ss.str();
        
        double threshold = goal_handle->get_goal()->goal;
        int attempts = 0;
        int status = 0;

        while (rclcpp::ok() && attempts < 20) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled by client.");
                status = 2;
                break;
            }

            attempts++;
            feedback->weight = dist_(rng_);
            
            RCLCPP_INFO(
                this->get_logger(),
                "[%s] Attempts: %d, Feedback weight: %f",
                goal_id_str.c_str(),
                attempts,
                feedback->weight
            );

            goal_handle->publish_feedback(feedback);

            if (feedback->weight >= threshold) {
                status = 1;
                break;
            }

            std::this_thread::sleep_for(1s);
        }

        result->success = (status == 1);
        switch (status) {
            case 1:
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Result: SUCCESS");
                break;
            case 2:
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Result: CANCELED");
                break;
            default:
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Result: FAILURE");
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
