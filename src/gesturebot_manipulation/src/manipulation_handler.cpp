#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <gesturebot_msgs/msg/arm_command.hpp>
#include <gesturebot_msgs/msg/drive_command.hpp>
#include <gesturebot_msgs/msg/gripper_command.hpp>

using namespace std::chrono_literals;

constexpr float GRIPPER_MIN_POSITION = -0.010;
constexpr float GRIPPER_MAX_POSITION = 0.019;

constexpr float GRIPPER_POS_DIFF = GRIPPER_MAX_POSITION - GRIPPER_MIN_POSITION;
constexpr float GRIPPER_POS_ONE_PERCENT = GRIPPER_POS_DIFF / 100;

class MotionHandler : public rclcpp::Node {
private:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

    rclcpp::TimerBase::SharedPtr m_InitMoveGroupsTimer;
    rclcpp::TimerBase::SharedPtr m_InitGripperActionServerTimer;

    rclcpp_action::Client<GripperCommand>::SharedPtr m_GripperClient;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_ArmGroup;

    rclcpp::Subscription<gesturebot_msgs::msg::ArmCommand>::SharedPtr m_ArmMotionSub;
    rclcpp::Subscription<gesturebot_msgs::msg::DriveCommand>::SharedPtr m_DriveMotionSub;
    rclcpp::Subscription<gesturebot_msgs::msg::GripperCommand>::SharedPtr m_GripperMotionSub;

public:
    MotionHandler() : Node("motion_handler") {
        m_ArmMotionSub = this->create_subscription<gesturebot_msgs::msg::ArmCommand>(
            "/arm_motion", 10,
            std::bind(&MotionHandler::armMotionCallback, this, std::placeholders::_1));

        m_DriveMotionSub = this->create_subscription<gesturebot_msgs::msg::DriveCommand>(
            "/drive_motion", 10,
            std::bind(&MotionHandler::driveMotionCallback, this, std::placeholders::_1));

        m_GripperMotionSub = this->create_subscription<gesturebot_msgs::msg::GripperCommand>(
            "/gripper_motion", 10,
            std::bind(&MotionHandler::gripperMotionCallback, this, std::placeholders::_1));

        m_GripperClient = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            this, "/gripper_controller/gripper_cmd");

        m_InitMoveGroupsTimer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MotionHandler::initMoveGroups, this));

        m_InitGripperActionServerTimer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MotionHandler::initGripperActionServer, this));
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
    void armMotionCallback(const gesturebot_msgs::msg::ArmCommand::SharedPtr msg) {
        geometry_msgs::msg::Pose target_pose = msg->target_pose;

        geometry_msgs::msg::Pose current_pose = m_ArmGroup->getCurrentPose().pose;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose);

        geometry_msgs::msg::Pose waypoint;
        tf2::Transform current_pose_transform, target_pose_transform;
        tf2::fromMsg(current_pose, current_pose_transform);
        tf2::fromMsg(target_pose, target_pose_transform);
        tf2::Transform result_transform = current_pose_transform * target_pose_transform;

        tf2::Vector3 t = result_transform.getOrigin();
        tf2::Quaternion q = result_transform.getRotation();

        waypoint.position.x = t.x();
        waypoint.position.y = t.y();
        waypoint.position.z = t.z();
        waypoint.orientation.x = q.x();
        waypoint.orientation.y = q.y();
        waypoint.orientation.z = q.z();
        waypoint.orientation.w = q.w();

        waypoints.push_back(waypoint);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.1;
        const double jump_threshold = 0.0;
        double fraction =
            m_ArmGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.9) {
            RCLCPP_INFO(get_logger(), "Path computed with %.2f%% success", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            m_ArmGroup->execute(plan);
        } else {
            RCLCPP_WARN(get_logger(), "Trajectory could not be planned completely (%.2f%%)",
                        fraction * 100.0);
        }
    }

    void driveMotionCallback(const gesturebot_msgs::msg::DriveCommand::SharedPtr msg) {}

    void gripperMotionCallback(const gesturebot_msgs::msg::GripperCommand::SharedPtr msg) {
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

        RCLCPP_INFO(get_logger(), "Arm Group using planning frame: %s",
                    m_ArmGroup->getPlanningFrame().c_str());

        auto current_pose = m_ArmGroup->getCurrentPose();
        RCLCPP_INFO(get_logger(), "Arm Group current pose:");
        RCLCPP_INFO(get_logger(), "  Position -> x: %.3f, y: %.3f, z: %.3f",
                    current_pose.pose.position.x, current_pose.pose.position.y,
                    current_pose.pose.position.z);
        RCLCPP_INFO(get_logger(), "  Orientation -> x: %.3f, y: %.3f, z: %.3f, w: %.3f",
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z, current_pose.pose.orientation.w);

        RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized!");
        m_InitMoveGroupsTimer->cancel();
    }

    void initGripperActionServer() {
        if (!m_GripperClient->action_server_is_ready()) {
            RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");
        } else {
            RCLCPP_INFO(get_logger(), "Gripper action server connected.");
            m_InitGripperActionServerTimer->cancel();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionHandler>());
    rclcpp::shutdown();

    return 0;
}
