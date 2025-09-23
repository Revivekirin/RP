#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_exam/action/Primenumber.hpp"

using namespace std::chrono_literals;

class PrimenumberActionClient : public rclcpp::Node
{
public:
  using Primenumber = action_exam::action::Primenumber;
  using GoalHandlePrimenumber = rclcpp_action::ClientGoalHandle<Primenumber>;

  PrimenumberActionClient()
  : rclcpp::Node("action_exam_client")
  {
    client_ = rclcpp_action::create_client<Primenumber>(this, "action_exam_action");
  }

  void send_goal(int target)
  {
    if (!client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
    }

    Primenumber::Goal goal_msg;
    goal_msg.target = target;

    rclcpp_action::Client<Primenumber>::SendGoalOptions opts;
    opts.goal_response_callback = 
      [this](std::shared_ptr<GoalHandlePrimenumber> handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    };

    opts.feedback_callback = 
      [this](GoalHandlePrimenumber::SharedPtr,
             const std::shared_ptr<const Primenumber::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback: cur_num=%d", feedback->cur_num);
             };
    
    opts.result_callback = 
      [this](const GoalHandlePrimenumber::WrappedResult & result) {
        using rclcpp_action::ResultCode;
        if(result.code == ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "succeeded, %zu primes:",
                        result.result->sequence.size());
            for (size_t i=0; i< result.result->sequence.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "%zu: %d", i+1, result.result->sequence[i]);
            }
        } else if (result.code == ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        rclcpp::shutdown();
      };
      
    client_->async_send_goal(goal_msg, opts);

    rclcpp::spin(node);
    return 0;
  }