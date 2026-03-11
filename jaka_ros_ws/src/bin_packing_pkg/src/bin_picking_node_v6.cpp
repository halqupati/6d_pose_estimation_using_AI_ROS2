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
#include <chrono>
#include <mutex>
#include <thread>
#include <limits>

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
    std::array<float, 4> nut_pose_angle_;
    bool nut_pose_angle_received_ = false;
    int step_retry_count_ = 0;
    static constexpr int kMaxStepRetries = 3;
    double gripper_open_cmd_ = 0.0;   // Robotiq 2F-85: 0.0 is fully open
    double gripper_close_cmd_ = 0.80; // Fully close for stronger grasp in sim
    double gripper_max_effort_ = 80.0;
    double tool_center_offset_ = 0.155;     // Link_6 -> finger contact center
    double seat_down_distance_ = 0.008;     // Extra downward move before closing
    double gripper_result_timeout_sec_ = 2.0;
    double pregrasp_height_ = 0.20;         // Hover height over nut
    double place_hover_height_ = 0.40;      // Hover height above place point
    double preclose_cmd_ = 0.55;            // Intermediate close before final squeeze
    double empty_close_threshold_ = 0.77;   // Near-full close means likely empty grasp
    double table_top_z_ = -0.06;            // Conservative table top estimate
    double pick_clearance_z_ = 0.01;       // Minimum TCP clearance above table
    double roll_current, pitch_current, yaw_current;
    size_t i = 0;
    std::mutex pose_angle_mutex;

    // coordinates to place nuts. x, y, z
    std::array<std::array<double, 3>, 3> nut_places = {{
        {0.5, 0.50, 0.05},
        {0.5, 0.55, 0.05},
        {0.3, -0.60, 0.05}
    }};

    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    ControlRobot()
        : Node("robot_controller")
    {
        pose_angle_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions pose_angle_options;
        pose_angle_options.callback_group = pose_angle_group_;

        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle", 10,
            std::bind(&ControlRobot::pose_angleCallback, this, std::placeholders::_1),
            pose_angle_options
        );

        bool_pub = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

        gripper_open_cmd_ = this->declare_parameter<double>("gripper_open_cmd", 0.0);
        gripper_close_cmd_ = this->declare_parameter<double>("gripper_close_cmd", 0.80);
        gripper_max_effort_ = this->declare_parameter<double>("gripper_max_effort", 80.0);
        tool_center_offset_ = this->declare_parameter<double>("tool_center_offset", 0.155);
        seat_down_distance_ = this->declare_parameter<double>("seat_down_distance", 0.008);
        gripper_result_timeout_sec_ = this->declare_parameter<double>("gripper_result_timeout_sec", 2.0);
        pregrasp_height_ = this->declare_parameter<double>("pregrasp_height", 0.20);
        place_hover_height_ = this->declare_parameter<double>("place_hover_height", 0.40);
        preclose_cmd_ = this->declare_parameter<double>("preclose_cmd", 0.55);
        empty_close_threshold_ = this->declare_parameter<double>("empty_close_threshold", 0.77);
        table_top_z_ = this->declare_parameter<double>("table_top_z", -0.06);
        pick_clearance_z_ = this->declare_parameter<double>("pick_clearance_z", 0.006);
        gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/robotiq_gripper_controller/gripper_cmd");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlRobot::timerCallback, this),
            timer_group_
        );

        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(1).transient_local());

        timer_collision_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ControlRobot::addCollisionMesh, this));

        RCLCPP_INFO(this->get_logger(), "Jaka_zu5 Control node created. Waiting for MoveGroup init...");
    }

    bool waitForRobotState(double timeout_sec = 2.0)
    {
        auto start = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);

        while ((this->now() - start) < timeout)
        {
            auto current_state = arm_move_group->getCurrentState(0.1);
            if (current_state)
            {
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

    void initializeMoveGroup()
    {
        arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "jaka_zu5");

        gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        arm_move_group->setPlanningTime(8.0);
        arm_move_group->setNumPlanningAttempts(20);
        arm_move_group->setMaxVelocityScalingFactor(0.5);

        if (!waitForRobotState(2.0))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive robot state");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Robot state ready");
        }

        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(),
                "Gripper action server not ready yet: /robotiq_gripper_controller/gripper_cmd");
        } else {
            RCLCPP_INFO(this->get_logger(), "Gripper action server is ready.");
        }
    }

private:
    void apply_sleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

    bool is_pose_close_to_current(const geometry_msgs::msg::Pose &target, double pos_tol = 0.003, double ang_tol = 0.03)
    {
        const auto current = arm_move_group->getCurrentPose("Link_6").pose;
        const double dx = current.position.x - target.position.x;
        const double dy = current.position.y - target.position.y;
        const double dz = current.position.z - target.position.z;
        const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (pos_err > pos_tol) {
            return false;
        }

        tf2::Quaternion q_cur, q_tgt;
        tf2::fromMsg(current.orientation, q_cur);
        tf2::fromMsg(target.orientation, q_tgt);
        q_cur.normalize();
        q_tgt.normalize();
        const double ang_err = q_cur.angleShortestPath(q_tgt);
        return ang_err <= ang_tol;
    }

    void addCollisionMesh()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        moveit_msgs::msg::CollisionObject table_obj;
        table_obj.header.frame_id = "world";
        table_obj.id = "simulation_table";

        shape_msgs::msg::SolidPrimitive table_primitive;
        table_primitive.type = table_primitive.BOX;
        table_primitive.dimensions = {1.5, 0.8, 1.007};

        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.507;
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
        RCLCPP_INFO(this->get_logger(), "Nut pose received.");
    }

    void request_new_detection(const char *reason)
    {
        RCLCPP_WARN(this->get_logger(), "%s. Resetting sequence and requesting a new pose.", reason);
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
        if (!arm_move_group) {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        if (i >= nut_places.size()) {
            return;
        }

        if (!nut_pose_angle_received_)
        {
            msg.data = true;
            bool_pub->publish(msg);
            return;
        }

        if (planning) {
            return;
        }

        switch (sequence) {
            case 0: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 0: Pregrasp (hover + yaw align)" COLOR_RESET);

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

                // Force a stable seed posture to avoid folded IK branches near the table.
                std::vector<double> ready_joints = {-1.57, 1.57, -1.57, 1.57, 1.57, 0.0};
                if (!try_or_recover(move_arm_to_joint_target(ready_joints), "Pregrasp seed failed")) {
                    planning = false;
                    break;
                }

                geometry_msgs::msg::Pose hover_pose;
                hover_pose.position.x = local_nut_pose_angle[0];
                hover_pose.position.y = local_nut_pose_angle[1];
                hover_pose.position.z = local_nut_pose_angle[2] + pregrasp_height_;

                tf2::Quaternion q_hover;
                q_hover.setRPY(PI, 0, local_nut_pose_angle[3]);
                hover_pose.orientation = tf2::toMsg(q_hover);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                arm_move_group->setPoseTarget(hover_pose);
                if (arm_move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    const bool executed = (arm_move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    arm_move_group->clearPoseTargets();
                    if (try_or_recover(executed, "Hover execute failed")) {
                        sequence = 1;
                    }
                } else {
                    arm_move_group->clearPoseTargets();
                    try_or_recover(false, "Hover plan failed");
                }

                planning = false;
                break;
            }

            case 1: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 1: Open gripper" COLOR_RESET);

                if (try_or_recover(joint_value_target_plan_and_execute(gripper_open_cmd_), "Open gripper failed")) {
                    sequence = 2;
                }

                planning = false;
                break;
            }

            case 2: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 2: Descend to pick" COLOR_RESET);

                std::array<float, 4> local_nut;
                { std::lock_guard<std::mutex> lock(pose_angle_mutex); local_nut = nut_pose_angle_; }

                const double requested_target_z = local_nut[2] + tool_center_offset_;
                const double min_safe_z = table_top_z_ + pick_clearance_z_;
                const double target_z = std::max(requested_target_z, min_safe_z);
                current_pose = arm_move_group->getCurrentPose("Link_6");
                const double current_z = current_pose.pose.position.z;

                if (!try_or_recover(
                        perform_cartesian_move(local_nut[0], local_nut[1], current_z),
                        "XY align before descent failed")) {
                    planning = false;
                    break;
                }

                if (try_or_recover(
                        perform_cartesian_move(local_nut[0], local_nut[1], target_z),
                        "Descent blocked")) {
                    sequence = 3;
                }

                planning = false;
                break;
            }

            case 3: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 3: Preclose + seat + final close" COLOR_RESET);

                if (!try_or_recover(joint_value_target_plan_and_execute(preclose_cmd_), "Preclose gripper failed")) {
                    planning = false;
                    break;
                }

                // Small downward nudge improves contact before closing in Gazebo.
                current_pose = arm_move_group->getCurrentPose("Link_6");
                const double min_safe_z = table_top_z_ + pick_clearance_z_;
                const double seat_z = std::max(current_pose.pose.position.z - seat_down_distance_, min_safe_z);
                if (!perform_cartesian_move(current_pose.pose.position.x, current_pose.pose.position.y, seat_z)) {
                    RCLCPP_WARN(this->get_logger(), "Seat-down move skipped (collision/path limit).");
                }

                if (!try_or_recover(joint_value_target_plan_and_execute(gripper_close_cmd_), "Final close gripper failed")) {
                    planning = false;
                    break;
                }

                double final_gripper_pos = 0.0;
                if (get_current_gripper_position(final_gripper_pos) &&
                    final_gripper_pos >= empty_close_threshold_) {
                    try_or_recover(false, "Likely missed grasp (gripper fully closed)");
                    planning = false;
                    break;
                }

                apply_sleep(0.5);
                sequence = 4;

                planning = false;
                break;
            }

            case 4: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 4: Lift" COLOR_RESET);

                current_pose = arm_move_group->getCurrentPose("Link_6");
                if (try_or_recover(
                        perform_cartesian_move(
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z + 0.10),
                        "Lift failed")) {
                    sequence = 5;
                }

                planning = false;
                break;
            }

            case 5: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 5: Move to place hover" COLOR_RESET);

                target_pose.position.x = nut_places[i][0];
                target_pose.position.y = nut_places[i][1];
                target_pose.position.z = nut_places[i][2] + place_hover_height_;

                bool moved_to_place = false;
                current_pose = arm_move_group->getCurrentPose("Link_6");
                target_pose.orientation = current_pose.pose.orientation;

                // Primary attempt: cartesian move to place hover with current orientation.
                moved_to_place = perform_cartesian_move(
                    target_pose.position.x, target_pose.position.y, target_pose.position.z, 0.30);

                // Fallback 1: normal pose planning with current orientation.
                if (!moved_to_place) {
                    moved_to_place = pose_target_plan_and_execute(target_pose);
                }

                // Fallback 2: go through a known stable ready joint posture, then retry.
                if (!moved_to_place) {
                    std::vector<double> ready_joints = {0.0, 1.57, -1.57, 1.57, 1.57, 0.0};
                    if (move_arm_to_joint_target(ready_joints)) {
                        moved_to_place = pose_target_plan_and_execute(target_pose);
                    }
                }

                if (try_or_recover(moved_to_place, "Move to place failed")) {
                    sequence = 6;
                }

                planning = false;
                break;
            }

            case 6: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 6: Descend to place" COLOR_RESET);

                current_pose = arm_move_group->getCurrentPose("Link_6");
                const double place_link6_z = nut_places[i][2] + tool_center_offset_;
                if (try_or_recover(
                        perform_cartesian_move(
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            place_link6_z,
                            0.25),
                        "Place descent failed")) {
                    sequence = 7;
                }

                planning = false;
                break;
            }

            case 7: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 7: Release" COLOR_RESET);

                if (try_or_recover(joint_value_target_plan_and_execute(gripper_open_cmd_), "Release gripper failed")) {
                    sequence = 8;
                }

                planning = false;
                break;
            }

            case 8: {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 8: Close gripper + return to ready" COLOR_RESET);

                std::vector<double> ready_joints = {0.0, 1.57, -1.57, 1.57, 1.57, 0.0};
                bool closed_ok = joint_value_target_plan_and_execute(gripper_close_cmd_);
                bool ready_ok = false;
                if (closed_ok) {
                    ready_ok = move_arm_to_joint_target(ready_joints);
                }
                if (try_or_recover(closed_ok && ready_ok, "Close gripper or return to ready failed")) {
                    sequence = 9;
                }

                planning = false;
                break;
            }

            case 9: {
                planning = true;
                i++;

                if (i < nut_places.size()) {
                    RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Cycle complete. Requesting next nut pose." COLOR_RESET);
                    sequence = 0;
                    nut_pose_angle_received_ = false;
                    step_retry_count_ = 0;
                    msg.data = true;
                    bool_pub->publish(msg);
                } else {
                    RCLCPP_INFO(this->get_logger(), COLOR_CYAN "The simulation is completed!" COLOR_RESET);
                }

                planning = false;
                break;
            }
        }
    }

    bool execute_timed_trajectory(moveit_msgs::msg::RobotTrajectory trajectory_msg)
    {
        if (trajectory_msg.joint_trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Refusing to execute empty trajectory.");
            return false;
        }

        robot_trajectory::RobotTrajectory rt(arm_move_group->getRobotModel(), "jaka_zu5");
        auto current_state = arm_move_group->getCurrentState(0.2);
        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current robot state unavailable for trajectory retiming.");
            return false;
        }

        rt.setRobotTrajectoryMsg(*current_state, trajectory_msg);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, 0.5, 0.5)) {
            RCLCPP_WARN(this->get_logger(), "Trajectory time parameterization failed.");
            return false;
        }

        rt.getRobotTrajectoryMsg(trajectory_msg);
        auto &pts = trajectory_msg.joint_trajectory.points;
        auto duration_to_ns = [](const auto &d) -> int64_t {
            return static_cast<int64_t>(d.sec) * 1000000000LL + static_cast<int64_t>(d.nanosec);
        };
        auto ns_to_duration = [](int64_t ns, auto &d) {
            if (ns < 0) {
                ns = 0;
            }
            d.sec = static_cast<int32_t>(ns / 1000000000LL);
            d.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        };

        // If this is effectively a no-op motion, avoid sending a degenerate trajectory.
        bool has_motion = false;
        if (pts.size() >= 2) {
            const auto &p0 = pts.front().positions;
            const auto &p1 = pts.back().positions;
            if (p0.size() == p1.size()) {
                for (size_t j = 0; j < p0.size(); ++j) {
                    if (std::abs(p1[j] - p0[j]) > 1e-5) {
                        has_motion = true;
                        break;
                    }
                }
            }
        } else {
            has_motion = false;
        }
        if (!has_motion) {
            return true;
        }

        for (size_t i = 1; i < pts.size(); ++i) {
            const int64_t prev_ns = duration_to_ns(pts[i - 1].time_from_start);
            int64_t curr_ns = duration_to_ns(pts[i].time_from_start);
            if (curr_ns <= prev_ns) {
                // JointTrajectoryController requires strictly increasing times.
                curr_ns = prev_ns + 1000000LL; // 1 ms minimum increment
                ns_to_duration(curr_ns, pts[i].time_from_start);
            }
        }

        return (arm_move_group->execute(trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool pose_target_plan_and_execute(const geometry_msgs::msg::Pose &target)
    {
        if (is_pose_close_to_current(target)) {
            return true;
        }

        arm_move_group->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (arm_move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            arm_move_group->clearPoseTargets();
            RCLCPP_WARN(this->get_logger(), "Pose planning failed.");
            return false;
        }

        const bool executed = execute_timed_trajectory(plan.trajectory_);
        arm_move_group->clearPoseTargets();
        return executed;
    }

    bool move_arm_to_joint_target(const std::vector<double> &joints)
    {
        const auto current = arm_move_group->getCurrentJointValues();
        if (current.size() == joints.size()) {
            double max_abs_err = 0.0;
            for (size_t idx = 0; idx < joints.size(); ++idx) {
                max_abs_err = std::max(max_abs_err, std::abs(current[idx] - joints[idx]));
            }
            if (max_abs_err < 1e-3) {
                // Skip no-op goals; they can produce zero-duration trajectories rejected by JTC.
                return true;
            }
        }

        arm_move_group->setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (arm_move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Joint-space planning failed.");
            return false;
        }

        return execute_timed_trajectory(plan.trajectory_);
    }

    bool get_current_gripper_position(double &position_out)
    {
        const auto vals = gripper_move_group->getCurrentJointValues();
        if (vals.empty()) {
            return false;
        }
        position_out = vals[0];
        return true;
    }

    bool wait_for_gripper_settle(double start_pos, double target_pos, bool closing, double timeout_sec)
    {
        const auto start_t = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_t).count() < timeout_sec) {
            const auto vals = gripper_move_group->getCurrentJointValues();
            if (!vals.empty()) {
                const double pos = vals[0];
                const double moved = std::abs(pos - start_pos);

                if (closing) {
                    // Either reached near target, or moved enough to indicate real closure/contact.
                    if (pos >= target_pos - 0.03 || moved >= 0.03) {
                        return true;
                    }
                } else {
                    // Opening should move toward lower values.
                    if (pos <= target_pos + 0.03 || moved >= 0.03) {
                        return true;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_WARN(this->get_logger(), "Gripper did not settle in %.2f s.", timeout_sec);
        return false;
    }

    bool perform_cartesian_move(double x, double y, double z, double min_fraction = 0.8)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose p = arm_move_group->getCurrentPose().pose;
        const double dx = p.position.x - x;
        const double dy = p.position.y - y;
        const double dz = p.position.z - z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.002) {
            return true;
        }

        p.position.x = x;
        p.position.y = y;
        p.position.z = z;
        waypoints.push_back(p);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm_move_group->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

        if (fraction >= min_fraction) {
            return execute_timed_trajectory(trajectory);
        }

        RCLCPP_WARN(this->get_logger(), "Cartesian path fraction too low: %.3f (need %.3f)", fraction, min_fraction);
        return false;
    }

    bool joint_value_target_plan_and_execute(double displacement)
    {
        const auto before_vals = gripper_move_group->getCurrentJointValues();
        if (before_vals.empty()) {
            RCLCPP_WARN(this->get_logger(), "Gripper joint state unavailable before command.");
            return false;
        }
        const double start_pos = before_vals[0];

        if (!gripper_action_client_->wait_for_action_server(std::chrono::milliseconds(300))) {
            RCLCPP_WARN(this->get_logger(), "Gripper action server unavailable.");
            return false;
        }

        // For this gripper model: 0.0=open, 0.8=closed.
        const double clamped = std::clamp(displacement, 0.0, 0.8);
        if (std::abs(clamped - displacement) > 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Requested gripper cmd %.4f out of range, clamped to %.4f", displacement, clamped);
        }

        GripperCommand::Goal goal;
        goal.command.position = clamped;
        goal.command.max_effort = gripper_max_effort_;

        auto goal_future = gripper_action_client_->async_send_goal(goal);
        if (goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Timeout sending gripper goal.");
            return false;
        }

        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Gripper goal rejected.");
            return false;
        }

        auto result_future = gripper_action_client_->async_get_result(goal_handle);
        const auto timeout_ms = static_cast<int>(gripper_result_timeout_sec_ * 1000.0);
        const bool closing = (clamped > start_pos);
        if (result_future.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
            // Result can be late in Gazebo; require observed joint motion before succeeding.
            RCLCPP_WARN(this->get_logger(), "Timeout waiting gripper result; waiting for joint settle.");
            return wait_for_gripper_settle(start_pos, clamped, closing, 1.2);
        }

        auto result = result_future.get();
        const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (ok) {
            RCLCPP_INFO(this->get_logger(), "Gripper action done: %.4f (0=open, 0.8=closed)", clamped);
        } else {
            RCLCPP_WARN(this->get_logger(), "Gripper action failed with code: %d", static_cast<int>(result.code));
            return false;
        }
        return wait_for_gripper_settle(start_pos, clamped, closing, 1.0);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_collision_;

    rclcpp::CallbackGroup::SharedPtr pose_angle_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
};

int main(int argc, char **argv)
{
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