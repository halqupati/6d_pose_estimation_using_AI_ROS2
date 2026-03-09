#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
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
#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>

#define PI 3.14159265359

// ANSI escape codes
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RESET   "\033[0m"

class ControlRobot : public rclcpp::Node {
public:
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Quaternion q_current;
    std_msgs::msg::Bool msg;

    int sequence = 0;
    bool planning = false;
    bool nut_pose_received_ = false;
    std::array<float, 4> nut_pose_angle_;
    bool nut_pose_angle_received_ = false;
    int step_retry_count_ = 0;
    static constexpr int kMaxStepRetries = 3;
    double roll_current, pitch_current, yaw_current;
    size_t i = 0;
    std::mutex pose_angle_mutex;

    // coordinates to place nuts. x, y, z
    std::array<std::array<double, 3>, 3> nut_places = {{
        {0.3, -0.24, 0.05},
        {0.3, -0.30, 0.05},
        {0.3, -0.36, 0.05}
    }};

    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    ControlRobot() : Node("robot_controller")
    {
        pose_angle_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions pose_angle_options;
        pose_angle_options.callback_group = pose_angle_group_;

        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle", 10, std::bind(&ControlRobot::pose_angleCallback, this, std::placeholders::_1), pose_angle_options);

        bool_pub = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&ControlRobot::timerCallback, this), timer_group_);

        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(1).transient_local());

        timer_collision_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&ControlRobot::addCollisionMesh, this));

        RCLCPP_INFO(this->get_logger(), "Jaka Control Node Started.");
    }

    void initializeMoveGroup()
    {
        arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "jaka_zu5");
        gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        arm_move_group->setPlanningTime(5.0);
        arm_move_group->setMaxVelocityScalingFactor(0.5);
        RCLCPP_INFO(this->get_logger(), "MoveGroups Initialized.");
    }

private:
    void apply_sleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

    void addCollisionMesh()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject table_obj;
        table_obj.header.frame_id = "world";
        table_obj.id = "simulation_table";

        shape_msgs::msg::SolidPrimitive table_primitive;
        table_primitive.type = table_primitive.BOX;
        table_primitive.dimensions = {1.5, 0.8, 1.030}; // Table size

        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.575;
        table_pose.orientation.w = 1.0;

        table_obj.primitives.push_back(table_primitive);
        table_obj.primitive_poses.push_back(table_pose);
        table_obj.operation = table_obj.ADD;

        psi.applyCollisionObject(table_obj);
        timer_collision_->cancel();
    }

    void pose_angleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /pos_angle: expected 4 values [x,y,z,yaw], got %zu",
                msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lock(pose_angle_mutex);
        std::copy_n(msg->data.begin(), 4, nut_pose_angle_.begin());
        nut_pose_angle_received_ = true;
    }

    void request_new_detection(const char *reason)
    {
        RCLCPP_WARN(this->get_logger(), "%s. Resetting to sequence 0 and requesting a new pose.", reason);
        sequence = 0;
        planning = false;
        nut_pose_angle_received_ = false;
        step_retry_count_ = 0;
        msg.data = true;
        bool_pub->publish(msg);
    }

    bool try_or_recover(bool success, const char *reason)
    {
        if (success) {
            step_retry_count_ = 0;
            return true;
        }

        step_retry_count_++;
        if (step_retry_count_ >= kMaxStepRetries) {
            request_new_detection(reason);
            return false;
        }

        RCLCPP_WARN(this->get_logger(), "%s. Retry %d/%d.", reason, step_retry_count_, kMaxStepRetries);
        return false;
    }

    void timerCallback()
    {
        if (!arm_move_group) return;
        if (i >= nut_places.size()) return;

        if (!nut_pose_angle_received_ && i < nut_places.size())
        {
            msg.data = true;
            bool_pub->publish(msg);
            return;
        }

        if (planning) return; // Prevent re-entry

        switch (sequence) {
            case 0: { // Move to Hover
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 0: Moving to HOVER" COLOR_RESET);

                std::array<float, 4> local_nut_pose_angle;
                {
                    std::lock_guard<std::mutex> lock(pose_angle_mutex);
                    local_nut_pose_angle = nut_pose_angle_;
                }

                const bool valid_pose =
                    std::isfinite(local_nut_pose_angle[0]) &&
                    std::isfinite(local_nut_pose_angle[1]) &&
                    std::isfinite(local_nut_pose_angle[2]) &&
                    std::isfinite(local_nut_pose_angle[3]);
                if (!valid_pose) {
                    request_new_detection("Received invalid nut pose values (NaN/Inf)");
                    break;
                }

                msg.data = false;
                bool_pub->publish(msg);
                arm_move_group->setStartStateToCurrentState();

                // Move 15 cm above nut center.
                geometry_msgs::msg::Pose hover_pose;
                hover_pose.position.x = local_nut_pose_angle[0];
                hover_pose.position.y = local_nut_pose_angle[1];
                hover_pose.position.z = local_nut_pose_angle[2] + 0.15;

                // Face Down
                tf2::Quaternion q_hover;
                q_hover.setRPY(0, PI, local_nut_pose_angle[3]);
                hover_pose.orientation = tf2::toMsg(q_hover);

                RCLCPP_INFO(this->get_logger(), "Hover Target Z: %f", hover_pose.position.z);

                arm_move_group->setPoseTarget(hover_pose);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (arm_move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    const bool executed = (arm_move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    if (try_or_recover(executed, "Hover execute failed")) {
                        sequence = 1;
                    }
                } else {
                    try_or_recover(false, "Hover plan failed");
                }
                planning = false;
                break;
            }

            case 1: { // Align Rotation
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 1: Aligning Rotation" COLOR_RESET);

                current_pose = arm_move_group->getCurrentPose("Link_6");
                std::array<float, 4> local_nut;
                { std::lock_guard<std::mutex> lock(pose_angle_mutex); local_nut = nut_pose_angle_; }

                tf2::Quaternion q_align;
                q_align.setRPY(0, PI, local_nut[3]);

                target_pose = current_pose.pose;
                target_pose.orientation = tf2::toMsg(q_align);

                if (try_or_recover(pose_target_plan_and_execute(target_pose), "Align rotation failed")) {
                    sequence = 2;
                }
                planning = false;
                break;
            }

            case 2: { // Open Gripper
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 2: Opening Gripper" COLOR_RESET);
                if (try_or_recover(joint_value_target_plan_and_execute(0.015f), "Open gripper failed")) {
                    sequence = 3;
                }
                planning = false;
                break;
            }

            case 3: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 3: Descending to Pick" COLOR_RESET);

                std::array<float, 4> local_nut;
                { std::lock_guard<std::mutex> lock(pose_angle_mutex); local_nut = nut_pose_angle_; }

                // Link_6 must stop above nut so fingers reach target contact point.
                double gripper_length = 0.155;
                double target_z = local_nut[2] + gripper_length;

                RCLCPP_INFO(this->get_logger(), "Nut Z: %f | Gripper Len: %f | Target Link_6 Z: %f",
                        local_nut[2], gripper_length, target_z);

                if (try_or_recover(perform_cartesian_move(local_nut[0], local_nut[1], target_z), "Descent blocked")) {
                    sequence = 4;
                }
                planning = false;
                break;
            }

            case 4: { // CLOSE GRIPPER (Grasp)
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 4: Grasping" COLOR_RESET);

                if (try_or_recover(joint_value_target_plan_and_execute(-0.001f), "Close gripper failed")) {
                    apply_sleep(1.0);
                    sequence = 5;
                }
                planning = false;
                break;
            }

            case 5: { // Move Up
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 5: Lifting" COLOR_RESET);
                current_pose = arm_move_group->getCurrentPose("Link_6");
                if (try_or_recover(
                        perform_cartesian_move(
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z + 0.15),
                        "Lift failed")) {
                    sequence = 6;
                }
                planning = false;
                break;
            }

            case 6: { // Move to Bin
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 6: Moving to Bin" COLOR_RESET);

                tf2::Quaternion q_bin;
                q_bin.setRPY(0, PI, -PI/4);

                target_pose.position.x = nut_places[i][0];
                target_pose.position.y = nut_places[i][1];
                target_pose.position.z = nut_places[i][2] + 0.10;
                target_pose.orientation = tf2::toMsg(q_bin);

                if (try_or_recover(pose_target_plan_and_execute(target_pose), "Move to bin failed")) {
                    sequence = 7;
                }
                planning = false;
                break;
            }

            case 7: { // Drop (Descent)
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 7: Dropping" COLOR_RESET);
                if (try_or_recover(
                        perform_cartesian_move(nut_places[i][0], nut_places[i][1], nut_places[i][2]),
                        "Drop descent failed")) {
                    sequence = 8;
                }
                planning = false;
                break;
            }

            case 8: { // Open Gripper
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 8: Releasing" COLOR_RESET);
                if (try_or_recover(joint_value_target_plan_and_execute(0.015f), "Release gripper failed")) {
                    sequence = 9;
                }
                planning = false;
                break;
            }

            case 9: { // Return to Ready
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 9: Returning to Start" COLOR_RESET);

                std::vector<double> ready_joints = {0.0, 1.57, -1.57, 1.57, 1.57, 0.0};
                if (try_or_recover(move_arm_to_joint_target(ready_joints), "Return to ready failed")) {
                    sequence = 10;
                }
                planning = false;
                break;
            }

            case 10: {
                planning = true;
                i++;
                if (i < nut_places.size()) {
                    RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Cycle complete. Next nut." COLOR_RESET);
                    sequence = 0;
                    nut_pose_angle_received_ = false;
                    step_retry_count_ = 0;
                    msg.data = true;
                    bool_pub->publish(msg);
                } else {
                    RCLCPP_INFO(this->get_logger(), COLOR_CYAN "All nuts processed!" COLOR_RESET);
                }
                planning = false;
                break;
            }
        }
    }

    bool pose_target_plan_and_execute(const geometry_msgs::msg::Pose &target) {
        arm_move_group->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            const bool executed = (arm_move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            arm_move_group->clearPoseTargets();
            return executed;
        }
        arm_move_group->clearPoseTargets();
        RCLCPP_WARN(this->get_logger(), "Pose planning failed.");
        return false;
    }

    bool perform_cartesian_move(double x, double y, double z) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose p = arm_move_group->getCurrentPose().pose;
        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        waypoints.push_back(p);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

        if (fraction > 0.8) {
            return (arm_move_group->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        RCLCPP_WARN(this->get_logger(), "Cartesian path fraction too low: %.3f", fraction);
        return false;
    }

    bool joint_value_target_plan_and_execute(float displacement) {
        std::vector<double> joint_values = gripper_move_group->getCurrentJointValues();
        if (!joint_values.empty()) {
            joint_values[0] = displacement;
            gripper_move_group->setJointValueTarget(joint_values);
            return (gripper_move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);
        }
        RCLCPP_WARN(this->get_logger(), "Gripper joint state unavailable.");
        return false;
    }

    bool move_arm_to_joint_target(const std::vector<double> &joints)
    {
        arm_move_group->setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Joint-space planning failed.");
            return false;
        }
        return (arm_move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_, timer_collision_;
    rclcpp::CallbackGroup::SharedPtr pose_angle_group_, timer_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group, gripper_move_group;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });
    node->initializeMoveGroup();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
