#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_exam/action/primenumber.hpp"

using namespace std::chrono_literals;

class PrimenumberActionClient : public rclcpp::Node
{
public:
  using ActionT = action_exam::action::Primenumber;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;

  PrimenumberActionClient() : rclcpp::Node("action_exam_client") {
    client_ = rclcpp_action::create_client<ActionT>(this, "action_exam_action");
  }

  void send_goal(int target) {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    ActionT::Goal goal;
    goal.target = target;

    rclcpp_action::Client<ActionT>::SendGoalOptions opts;

    // ★ Foxy 요구 시그니처: shared_future 인자로 받아야 함
    opts.goal_response_callback =
      [this](std::shared_future<typename GoalHandleT::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
      };

    opts.feedback_callback =
      [this](GoalHandleT::SharedPtr,
             const std::shared_ptr<const ActionT::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: cur_num=%d", feedback->cur_num);
      };

    opts.result_callback =
      [this](const GoalHandleT::WrappedResult & result) {
        using rclcpp_action::ResultCode;
        if (result.code == ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Succeeded. %zu primes:",
                      result.result->sequence.size());
          for (size_t i = 0; i < result.result->sequence.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "%zu: %d", i + 1, result.result->sequence[i]);
          }
        } else if (result.code == ResultCode::CANCELED) {
          RCLCPP_WARN(this->get_logger(), "Canceled");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal, opts);
  }

private:
  rclcpp_action::Client<ActionT>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrimenumberActionClient>();

  int target = 20;
  if (argc == 2) target = std::atoi(argv[1]);

  node->send_goal(target);
  rclcpp::spin(node);
  return 0;
}

