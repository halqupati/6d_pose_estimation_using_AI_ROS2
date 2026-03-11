#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shapes.h>
#include <array>
#include <mutex>

#define PI 3.14159265359

// ANSI escape codes for cleaner logging
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_MAGENTA "\033[35m"

class ControlRobot : public rclcpp::Node {
public:
    ControlRobot() : Node("robot_controller") {
        pose_angle_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions pose_angle_options;
        pose_angle_options.callback_group = pose_angle_group_;

        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle", 10, std::bind(&ControlRobot::pose_angleCallback, this, std::placeholders::_1), pose_angle_options);

        bool_pub = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(1).transient_local());

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
            std::bind(&ControlRobot::timerCallback, this), timer_group_);

        timer_collision_ = this->create_wall_timer(std::chrono::milliseconds(500), 
            std::bind(&ControlRobot::addCollisionMesh, this));

        RCLCPP_INFO(this->get_logger(), "Fixed & Optimized Jaka Node Started.");
    }

    void initializeMoveGroup() {
        arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "jaka_zu5");
        gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        arm_move_group->setMaxVelocityScalingFactor(0.8); 
        arm_move_group->setMaxAccelerationScalingFactor(0.6);
    }

private:
    void timerCallback() {
        if (!arm_move_group || planning) return;

        if (!nut_pose_angle_received_ && i < nut_places.size()) {
            std_msgs::msg::Bool trigger_msg;
            trigger_msg.data = true;
            bool_pub->publish(trigger_msg);
            return;
        }

        switch (sequence) {
            case 0: { // BRACES ADDED HERE
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_CYAN "SEQ 0: Transit to hover" COLOR_RESET);
                
                {
                    std::lock_guard<std::mutex> lock(pose_angle_mutex);
                    local_target_nut = nut_pose_angle_;
                }

                gripper_move_group->setJointValueTarget({0.015}); 
                gripper_move_group->asyncMove(); 

                tf2::Quaternion q;
                q.setRPY(0, PI, local_target_nut[3]);

                target_pose.position.x = local_target_nut[0];
                target_pose.position.y = local_target_nut[1];
                target_pose.position.z = local_target_nut[2] + 0.10;
                target_pose.orientation = tf2::toMsg(q);

                if (pose_target_plan_and_execute(target_pose)) sequence = 1;
                planning = false;
                break;
            }

            case 1: { // BRACES ADDED HERE
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "SEQ 1: Linear descent" COLOR_RESET);
                if (perform_cartesian_move(local_target_nut[0], local_target_nut[1], local_target_nut[2], 0.3)) {
                    sequence = 2;
                }
                planning = false;
                break;
            }

            case 2: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_MAGENTA "SEQ 2: Closing Gripper" COLOR_RESET);
                if (joint_value_target_plan_and_execute(0.003f)) sequence = 3;
                planning = false;
                break;
            }

            case 3: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_CYAN "SEQ 3: Lifting + Pipelining" COLOR_RESET);
                
                std_msgs::msg::Bool trigger_msg; trigger_msg.data = true;
                bool_pub->publish(trigger_msg);
                nut_pose_angle_received_ = false; 

                if (perform_cartesian_move(local_target_nut[0], local_target_nut[1], local_target_nut[2] + 0.15, 0.6)) {
                    sequence = 4;
                }
                planning = false;
                break;
            }

            case 4: { // BRACES ADDED HERE
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_CYAN "SEQ 4: Moving to Bin" COLOR_RESET);
                
                tf2::Quaternion q_bin;
                q_bin.setRPY(0, PI, -PI/4);

                target_pose.position.x = nut_places[i][0];
                target_pose.position.y = nut_places[i][1];
                target_pose.position.z = nut_places[i][2] + 0.10;
                target_pose.orientation = tf2::toMsg(q_bin);

                if (pose_target_plan_and_execute(target_pose)) sequence = 5;
                planning = false;
                break;
            }

            case 5: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "SEQ 5: Drop and Return" COLOR_RESET);
                
                perform_cartesian_move(nut_places[i][0], nut_places[i][1], nut_places[i][2], 0.4);
                joint_value_target_plan_and_execute(0.015f);
                perform_cartesian_move(nut_places[i][0], nut_places[i][1], nut_places[i][2] + 0.10, 0.6);
                
                i++;
                sequence = (i < nut_places.size()) ? 0 : 10;
                planning = false;
                break;
            }

            case 10: {
                RCLCPP_INFO_ONCE(this->get_logger(), COLOR_GREEN "SIMULATION COMPLETE" COLOR_RESET);
                break;
            }
        }
    }

    bool perform_cartesian_move(double x, double y, double z, double speed) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose p = arm_move_group->getCurrentPose().pose;
        p.position.x = x; p.position.y = y; p.position.z = z;
        waypoints.push_back(p);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

        if (fraction > 0.9) {
            robot_trajectory::RobotTrajectory rt(arm_move_group->getRobotModel(), arm_move_group->getName());
            rt.setRobotTrajectoryMsg(*arm_move_group->getCurrentState(), trajectory);
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(rt, speed, speed/2.0);
            rt.getRobotTrajectoryMsg(trajectory);
            return (arm_move_group->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool pose_target_plan_and_execute(const geometry_msgs::msg::Pose &target) {
        arm_move_group->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (arm_move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool joint_value_target_plan_and_execute(float displacement) {
        gripper_move_group->setJointValueTarget({displacement});
        return (gripper_move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);
    }

    void pose_angleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_angle_mutex);
        std::copy_n(msg->data.begin(), 4, nut_pose_angle_.begin());
        nut_pose_angle_received_ = true;
    }

    void addCollisionMesh() {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "world";
        table.id = "simulation_table";
        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX; box.dimensions = {1.5, 0.8, 1.06};
        geometry_msgs::msg::Pose p; p.position.z = -0.56; p.orientation.w = 1.0;
        table.primitives.push_back(box); table.primitive_poses.push_back(p);
        table.operation = table.ADD;
        psi.applyCollisionObject(table);
        timer_collision_->cancel();
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_, timer_collision_;
    rclcpp::CallbackGroup::SharedPtr pose_angle_group_, timer_group_;

    std::array<float, 4> nut_pose_angle_, local_target_nut;
    bool nut_pose_angle_received_ = false, planning = false;
    int sequence = 0;
    size_t i = 0;
    geometry_msgs::msg::Pose target_pose;
    std::mutex pose_angle_mutex;

    std::array<std::array<double, 3>, 3> nut_places = {{{0.3, -0.24, 1.07}, {0.3, -0.30, 1.07}, {0.3, -0.36, 1.07}}};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlRobot>();
    
    std::thread spinning_thread([node]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });

    node->initializeMoveGroup();
    spinning_thread.join();
    rclcpp::shutdown();
    return 0;
}
