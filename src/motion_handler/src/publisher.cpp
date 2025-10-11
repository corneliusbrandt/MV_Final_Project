#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "motion_handler/msg/motion_command.hpp"

using namespace std::chrono_literals;

constexpr float GRIPPER_MIN_POSITION = -0.010;
constexpr float GRIPPER_MAX_POSITION = 0.019;

constexpr float GRIPPER_POS_DIFF = GRIPPER_MAX_POSITION - GRIPPER_MIN_POSITION;
constexpr float GRIPPER_POS_ONE_PERCENT = GRIPPER_POS_DIFF / 100;

class MotionHandler : public rclcpp::Node {
private:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;
    using MotionCommand = motion_handler::msg::MotionCommand;

    rclcpp::TimerBase::SharedPtr m_InitMoveGroupsTimer;
    rclcpp_action::Client<GripperCommand>::SharedPtr m_GripperClient;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_ArmGroup;
    rclcpp::Subscription<MotionCommand>::SharedPtr m_MotionCmdSub;

public:
    MotionHandler() : Node("motion_handler") {
        m_InitMoveGroupsTimer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MotionHandler::initMoveGroups, this));

        m_MotionCmdSub = this->create_subscription<MotionCommand>(
            "/motion_command", 10,
            std::bind(&MotionHandler::motionCallback, this, std::placeholders::_1));

        m_GripperClient = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            this, "/gripper_controller/gripper_cmd");
        while (!m_GripperClient->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
        }
        RCLCPP_INFO(get_logger(), "Gripper action server connected.");
    }

    void moveGripper(float position) {
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 1.0;

        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GripperGoalHandle::WrappedResult& result) {
            if (result.result->reached_goal) {
                RCLCPP_INFO(this->get_logger(), "Gripper reached goal (final position: %f)",
                            result.result->position);
            } else {
                RCLCPP_INFO(this->get_logger(), "Gripper did not reach goal (final position: %f)",
                            result.result->position);
            }
        };

        RCLCPP_INFO(get_logger(), "Sending command to set gripper to position %f", position);
        m_GripperClient->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void motionCallback(const MotionCommand::SharedPtr msg) {
        if (msg->gripper_percentage > 100) {
            RCLCPP_WARN(get_logger(), "Invalid gripper percentage (%u) gripper will not be moved",
                        msg->gripper_percentage);
        } else {
            float new_gripper_pos =
                GRIPPER_MIN_POSITION + (msg->gripper_percentage * GRIPPER_POS_ONE_PERCENT);
            moveGripper(new_gripper_pos);
        }
    }

    void initMoveGroups() {
        if (m_ArmGroup) return;

        RCLCPP_INFO(get_logger(), "Initializing MoveIt interfaces...");
        m_ArmGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized!");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionHandler>());
    rclcpp::shutdown();

    return 0;
}
