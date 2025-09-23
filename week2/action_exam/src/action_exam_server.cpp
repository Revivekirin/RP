#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_exam/action/Primenumber.hpp"
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;


class PrimenumberAction : public rclcpp::Node
{
public:
  using Primenumber = action_exam::action::Primenumber;
  using GoalHandlePrimenumber = rclcpp_action::ServerGoalHandle<Primenumber>;

  PrimenumberActionServer()
  : rclcpp::Node("action_exam_server")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    server = rlcpp_action::create_server<Primenumber>(
        this,
        "action_exam_action",
        std::bind(&PrimenumberActionServer::handle_goal, this, _1, _2),
        std::bind(&PrimenumberActionServer::handle_cancel, this, _1),
        std::bind(&PrimenumberActionServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "action_exam_action: ready");
  }

private:
  rclcpp_action::Server<Primenumber>::SharedPtr server_;

  rclcpp_action::Goalresponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Primenumber::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal: target=%d", goal->target);
        if (goal->target <2) {
            RCLCPP_WARN(this->get_looger(), "target < 2: reject");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePrimenumber>  /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

  void handle_accepted(const std::shared_ptr<GoalHandlePrimenumber> goal_handle)
  {
    std::thread{std::bind(&PrimenumberActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shard_ptr<GoalHandlePrimenumber> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PrimenumberAction::Result>();

    for (int i=2; i<= goal->target; ++i) {
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            goal_handle->canceled(result);
            return;
        }
        bool is_prime=true;
        for (int j=2;j*j<=i; ++j) {
            if (i%j==0) (is_prime=false; break;)
        }

        if (is_prime) {
            result->sequence.push_back(i);
            auto feedback = std::make_shared<Primenumber::Feedback>();
            feedback->cur_num=i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Prime: %d", i);
        }

        std::this_thread::sleep_for(100ms);
    }

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded (found %zu prime).",
                    result->sequence.size());
    }
  }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrimenumberActionServer>());
    rclcpp::shutdown();
    return 0;
}