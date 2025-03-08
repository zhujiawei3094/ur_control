//
// Created by zhu on 25-2-26.
//

#ifndef UR_CONTROLLER_H
#define UR_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <utility>
#include <oneapi/tbb/task_arena.h>

#include "fstream"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class URController : public rclcpp::Node {
public:
    URController() : Node("ur_controller") {
        // 初始化Action客户端
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        );

        // 等待Action服务器上线
        this->connect_to_server();
    }

    //assert infile not nullptr
    void register_data(std::queue<std::vector<double> > &data_) {
        positions.swap(data_);
        execute_next(10);
    }

    void send_joint_goal(const std::vector<double> &positions, double duration) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = compose_goal(positions, duration);
        auto send_goal_options = create_send_goal_options();
        // 异步发送目标
        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "Sending goal %s %s %s %s %s %s", goal_msg.trajectory.joint_names[0].c_str(),
                    goal_msg.trajectory.joint_names[1].c_str(), goal_msg.trajectory.joint_names[2].c_str(),
                    goal_msg.trajectory.joint_names[3].c_str(), goal_msg.trajectory.joint_names[4].c_str(),goal_msg.trajectory.joint_names[5].c_str());
        RCLCPP_INFO(get_logger(), "Sending goal %f %f %f %f %f %f", positions[0], positions[1], positions[2],
                    positions[3], positions[4], positions[5]);
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    std::queue<std::vector<double> > positions;

    void execute_next(double duration) {
        if (!positions.empty()) {
            send_joint_goal(positions.front(), duration);
            positions.pop();
        } else {
            RCLCPP_ERROR(get_logger(), "No goal was received");
        }
    }

    void connect_to_server() {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
            std::thread([this]() {
                rclcpp::spin_some(this->get_node_base_interface());
                if (action_client_->wait_for_action_server()) {
                    RCLCPP_INFO(get_logger(), "Action server connected!");
                }
            }).detach();
        }
    }

    static FollowJointTrajectory::Goal compose_goal(
        const std::vector<double> &positions,
        double sec_duration
    ) {
        FollowJointTrajectory::Goal goal;

        // 设置关节名称（根据实际机型调整）
        goal.trajectory.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        // 构建轨迹点
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = rclcpp::Duration::from_seconds(sec_duration);

        // 添加轨迹点
        goal.trajectory.points.push_back(point);

        // 设置轨迹容差
        int i =0;
        goal.path_tolerance.resize(6);
        for (auto &tolerance: goal.path_tolerance) {
            tolerance.name = goal.trajectory.joint_names[i];
            ++i;
            tolerance.position = 1000;
            tolerance.velocity = 1000;
        }

        return goal;
    }

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions create_send_goal_options() {
        using FullGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
        options.goal_response_callback =
                [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowJointTrajectory> > &goal_handle) {
                    if (!goal_handle) {
                        RCLCPP_ERROR(get_logger(), "Goal rejected by server");
                    } else {
                        RCLCPP_INFO(get_logger(), "Goal accepted!");
                    }
                };
        options.feedback_callback =
                [this](
            FullGoalHandle::SharedPtr,
            const std::shared_ptr<const FollowJointTrajectory::Feedback> &feedback
        ) {
                    RCLCPP_DEBUG(
                        get_logger(),
                        "Current progress: "
                    );
                };
        options.result_callback =
                [this](const FullGoalHandle::WrappedResult &result) {
                    switch (result.code) {
                        case rclcpp_action::ResultCode::SUCCEEDED: {
                            RCLCPP_INFO(get_logger(), "Goal achieved!");
                            execute_next(2);
                            break;
                        }
                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(get_logger(), "Goal aborted");
                            // execute_next(0.1);
                            break;
                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_WARN(get_logger(), "Goal canceled");
                            break;
                        default:
                            RCLCPP_ERROR(get_logger(), "Unknown result code");
                            break;
                    }
                };
        return options;
    }
};


#endif //UR_CONTROLLER_H
