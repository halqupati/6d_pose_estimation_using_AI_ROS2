#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#define PI 3.14159265359
#define COLOR_GREEN "\033[32m"
#define COLOR_CYAN  "\033[36m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED   "\033[31m"
#define COLOR_RESET "\033[0m"

class ControlRobot : public rclcpp::Node {
public:
    ControlRobot()
        : Node("bin_picking_node_v7_to_control_jaka_s12")
    {
        pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        arm_group_name_ = this->declare_parameter<std::string>("arm_group", "jaka_s12");
        gripper_group_name_ = this->declare_parameter<std::string>("gripper_group", "gripper");
        flange_link_name_ = this->declare_parameter<std::string>("flange_link", "tcp");
        ready_joints_ = this->declare_parameter<std::vector<double>>(
            "ready_joints", std::vector<double>{-1.57, 1.57, 1.57, 1.57, -1.57, 0.0});

        pregrasp_height_ = this->declare_parameter<double>("pregrasp_height", 0.20);
        place_hover_height_ = this->declare_parameter<double>("place_hover_height", 0.40);
        lift_height_ = this->declare_parameter<double>("lift_height", 0.10);
        tool_center_offset_ = this->declare_parameter<double>("tool_center_offset", 0.019);
        pick_clearance_z_ = this->declare_parameter<double>("pick_clearance_z", 0.006);
        table_top_z_ = this->declare_parameter<double>("table_top_z", -0.06);

        arm_velocity_scaling_ = this->declare_parameter<double>("arm_velocity_scaling", 0.30);
        arm_acceleration_scaling_ = this->declare_parameter<double>("arm_acceleration_scaling", 0.30);
        arm_planning_time_ = this->declare_parameter<double>("arm_planning_time", 5.0);

        gripper_open_cmd_ = this->declare_parameter<double>("gripper_open_cmd", 0.0);
        gripper_close_cmd_ = this->declare_parameter<double>("gripper_close_cmd", -0.025);
        gripper_min_position_ = this->declare_parameter<double>("gripper_min_position", -0.026);
        gripper_max_position_ = this->declare_parameter<double>("gripper_max_position", 0.001);
        empty_close_threshold_ = this->declare_parameter<double>("empty_close_threshold", -0.023);
        gripper_motion_duration_ = this->declare_parameter<double>("gripper_motion_duration", 0.6);
        gripper_settle_time_ = this->declare_parameter<double>("gripper_settle_time", 0.5);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = pose_callback_group_;
        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle",
            10,
            std::bind(&ControlRobot::poseCallback, this, std::placeholders::_1),
            sub_options);

        trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);
        pose_received_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pose_received", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlRobot::mainLoop, this),
            timer_callback_group_);

        collision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ControlRobot::addCollisionObjects, this));

        RCLCPP_INFO(this->get_logger(), "Jaka S12 v5-style bin-picking node created. Waiting for MoveGroup init...");
    }

    void initializeMoveGroup()
    {
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_group_name_);
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), gripper_group_name_);

        arm_group_->setPlanningTime(arm_planning_time_);
        arm_group_->setNumPlanningAttempts(5);
        arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
        arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);
        gripper_group_->setPlanningTime(2.0);
        gripper_group_->setNumPlanningAttempts(3);
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);

        if (!arm_group_->setEndEffectorLink(flange_link_name_)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to set arm end effector link to '%s'. MoveIt kept '%s'.",
                flange_link_name_.c_str(),
                arm_group_->getEndEffectorLink().c_str());
        }

        if (!waitForRobotState(2.0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive robot state");
        } else {
            RCLCPP_INFO(this->get_logger(), "Robot state ready");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Move groups ready. arm_group=%s gripper_group=%s flange_link=%s end_effector=%s",
            arm_group_name_.c_str(),
            gripper_group_name_.c_str(),
            flange_link_name_.c_str(),
            arm_group_->getEndEffectorLink().c_str());
    }

private:
    int sequence_ = 0;
    bool is_busy_ = false;
    bool nut_pose_received_ = false;
    std::array<float, 4> nut_pose_angle_{};
    std::mutex pose_mutex_;
    size_t bin_index_ = 0;

    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::string flange_link_name_;
    std::vector<double> ready_joints_;

    double pregrasp_height_ = 0.20;
    double place_hover_height_ = 0.40;
    double lift_height_ = 0.10;
    double tool_center_offset_ = 0.019;  // World-Z offset for the tcp in top-down grasping
    double pick_clearance_z_ = 0.006;
    double table_top_z_ = -0.06;
    double arm_velocity_scaling_ = 0.30;
    double arm_acceleration_scaling_ = 0.30;
    double arm_planning_time_ = 5.0;
    double gripper_open_cmd_ = 0.0;
    double gripper_close_cmd_ = -0.025;
    double gripper_min_position_ = -0.026;
    double gripper_max_position_ = 0.001;
    double empty_close_threshold_ = -0.023;
    double gripper_motion_duration_ = 0.6;
    double gripper_settle_time_ = 0.5;

    std::array<std::array<double, 3>, 3> bin_locations_ = {{
        {0.5, 0.50, 0.05},
        {0.5, 0.55, 0.05},
        {0.3, -0.60, 0.05},
    }};

    rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pose_received_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr collision_timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    bool waitForRobotState(double timeout_sec)
    {
        const auto start = this->now();
        const auto timeout = rclcpp::Duration::from_seconds(timeout_sec);
        while ((this->now() - start) < timeout) {
            if (arm_group_->getCurrentState(0.1)) {
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

    void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /pos_angle: expected 4 or 7 values, got %zu", msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lock(pose_mutex_);
        nut_pose_angle_[0] = msg->data[0];
        nut_pose_angle_[1] = msg->data[1];
        nut_pose_angle_[2] = msg->data[2];

        if (msg->data.size() >= 7) {
            tf2::Quaternion q(msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
            q.normalize();
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            nut_pose_angle_[3] = static_cast<float>(yaw);
        } else {
            nut_pose_angle_[3] = msg->data[3];
        }

        nut_pose_received_ = true;
        std_msgs::msg::Bool ack;
        ack.data = true;
        pose_received_pub_->publish(ack);
        RCLCPP_INFO(this->get_logger(), "Nut pose received.");
    }

    bool getNutPose(std::array<float, 4> &local_nut)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        local_nut = nut_pose_angle_;
        return std::isfinite(local_nut[0]) && std::isfinite(local_nut[1]) &&
               std::isfinite(local_nut[2]) && std::isfinite(local_nut[3]);
    }

    void requestNewDetection(const char *reason)
    {
        RCLCPP_WARN(this->get_logger(), "%s. Resetting sequence and requesting a new pose.", reason);
        sequence_ = 0;
        is_busy_ = false;
        nut_pose_received_ = false;
        std_msgs::msg::Bool trigger;
        trigger.data = true;
        trigger_pub_->publish(trigger);
    }

    geometry_msgs::msg::Pose buildTopDownPose(double x, double y, double z, double yaw)
    {
        geometry_msgs::msg::Pose target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        tf2::Quaternion q;
        q.setRPY(PI, 0.0, yaw);
        q.normalize();
        target.orientation = tf2::toMsg(q);
        return target;
    }

    bool executeTimedTrajectory(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group,
        const std::string &planning_group,
        moveit_msgs::msg::RobotTrajectory trajectory_msg,
        double velocity_scaling,
        double acceleration_scaling)
    {
        if (trajectory_msg.joint_trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Refusing to execute empty trajectory.");
            return false;
        }

        robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), planning_group);
        auto current_state = move_group->getCurrentState(0.2);
        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current robot state unavailable for trajectory retiming.");
            return false;
        }

        rt.setRobotTrajectoryMsg(*current_state, trajectory_msg);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling)) {
            RCLCPP_WARN(this->get_logger(), "Trajectory time parameterization failed.");
            return false;
        }

        rt.getRobotTrajectoryMsg(trajectory_msg);
        auto &pts = trajectory_msg.joint_trajectory.points;
        auto duration_to_ns = [](const builtin_interfaces::msg::Duration &d) -> int64_t {
            return static_cast<int64_t>(d.sec) * 1000000000LL + static_cast<int64_t>(d.nanosec);
        };
        auto ns_to_duration = [](int64_t ns, builtin_interfaces::msg::Duration &d) {
            if (ns < 0) {
                ns = 0;
            }
            d.sec = static_cast<int32_t>(ns / 1000000000LL);
            d.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        };

        for (size_t idx = 1; idx < pts.size(); ++idx) {
            const int64_t prev_ns = duration_to_ns(pts[idx - 1].time_from_start);
            int64_t curr_ns = duration_to_ns(pts[idx].time_from_start);
            if (curr_ns <= prev_ns) {
                curr_ns = prev_ns + 1000000LL;
                ns_to_duration(curr_ns, pts[idx].time_from_start);
            }
        }

        return (move_group->execute(trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool planAndExecutePose(const geometry_msgs::msg::Pose &target)
    {
        arm_group_->setStartStateToCurrentState();
        arm_group_->setPoseTarget(target, flange_link_name_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = arm_group_->plan(plan);
        arm_group_->clearPoseTargets();
        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Pose planning failed.");
            return false;
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, plan.trajectory_, arm_velocity_scaling_, arm_acceleration_scaling_);
    }

    bool moveArmToJointTarget(const std::vector<double> &joints)
    {
        arm_group_->setStartStateToCurrentState();
        arm_group_->setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Joint-space planning failed.");
            return false;
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, plan.trajectory_, arm_velocity_scaling_, arm_acceleration_scaling_);
    }

    bool moveCartesian(double x, double y, double z, double min_fraction = 0.90)
    {
        geometry_msgs::msg::Pose target = arm_group_->getCurrentPose(flange_link_name_).pose;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        return moveCartesianPose(target, min_fraction);
    }

    bool moveCartesianPose(const geometry_msgs::msg::Pose &target, double min_fraction = 0.90)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
        if (fraction < min_fraction) {
            RCLCPP_WARN(
                this->get_logger(),
                "Cartesian path fraction too low: %.3f (need %.3f)",
                fraction,
                min_fraction);
            return false;
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, trajectory, arm_velocity_scaling_, arm_acceleration_scaling_);
    }

    bool moveToPoseWithFallback(const geometry_msgs::msg::Pose &target, double min_fraction = 0.90)
    {
        if (moveCartesianPose(target, min_fraction)) {
            return true;
        }

        RCLCPP_WARN(this->get_logger(), "Cartesian move failed, falling back to pose planning.");
        return planAndExecutePose(target);
    }

    bool getCurrentGripperPosition(double &position_out)
    {
        const auto values = gripper_group_->getCurrentJointValues();
        if (values.empty()) {
            return false;
        }
        position_out = values[0];
        return true;
    }

    bool waitForGripperSettle(double start_pos, double target_pos, double timeout_sec)
    {
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < timeout_sec) {
            double current_pos = 0.0;
            if (getCurrentGripperPosition(current_pos)) {
                const double moved = std::abs(current_pos - start_pos);
                const bool toward_lower = target_pos < start_pos;
                constexpr double kTargetTolerance = 0.0015;
                constexpr double kMovedEnough = 0.002;

                if ((toward_lower && current_pos <= target_pos + kTargetTolerance) ||
                    (!toward_lower && current_pos >= target_pos - kTargetTolerance) ||
                    moved >= kMovedEnough) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_WARN(this->get_logger(), "Gripper did not settle in %.2f s.", timeout_sec);
        return false;
    }

    bool executeDirectGripperMotion(double target_position)
    {
        auto current_state = gripper_group_->getCurrentState(0.2);
        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current gripper state unavailable.");
            return false;
        }

        const auto *joint_model_group = current_state->getRobotModel()->getJointModelGroup(gripper_group_name_);
        if (!joint_model_group) {
            RCLCPP_WARN(this->get_logger(), "Gripper joint model group '%s' not found.", gripper_group_name_.c_str());
            return false;
        }

        std::vector<double> start_positions;
        current_state->copyJointGroupPositions(joint_model_group, start_positions);
        if (start_positions.size() != 1) {
            RCLCPP_WARN(this->get_logger(), "Unexpected gripper joint group size: %zu", start_positions.size());
            return false;
        }

        moveit::core::RobotState goal_state(*current_state);
        goal_state.setJointGroupPositions(joint_model_group, std::vector<double>{target_position});

        robot_trajectory::RobotTrajectory trajectory(gripper_group_->getRobotModel(), gripper_group_name_);
        trajectory.addSuffixWayPoint(*current_state, 0.0);
        trajectory.addSuffixWayPoint(goal_state, gripper_motion_duration_);

        moveit_msgs::msg::RobotTrajectory trajectory_msg;
        trajectory.getRobotTrajectoryMsg(trajectory_msg);
        return executeTimedTrajectory(gripper_group_, gripper_group_name_, trajectory_msg, 1.0, 1.0);
    }

    bool moveGripper(double command)
    {
        double start_pos = 0.0;
        if (!getCurrentGripperPosition(start_pos)) {
            RCLCPP_WARN(this->get_logger(), "Gripper state unavailable before command.");
            return false;
        }

        const double target_pos = std::clamp(command, gripper_min_position_, gripper_max_position_);
        if (std::abs(target_pos - start_pos) < 5e-4) {
            return true;
        }

        if (!executeDirectGripperMotion(target_pos)) {
            RCLCPP_WARN(this->get_logger(), "Gripper execute failed.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Gripper move done: %.4f", target_pos);
        return waitForGripperSettle(start_pos, target_pos, 1.0);
    }

    bool likelyMissedGrasp() const
    {
        const bool closes_toward_max = gripper_close_cmd_ >= gripper_open_cmd_;
        double final_pos = 0.0;
        if (!const_cast<ControlRobot *>(this)->getCurrentGripperPosition(final_pos)) {
            return false;
        }

        return closes_toward_max ? (final_pos >= empty_close_threshold_) : (final_pos <= empty_close_threshold_);
    }

    void addCollisionObjects()
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
        collision_timer_->cancel();
    }

    void completeCycleAndRequestNext()
    {
        ++bin_index_;
        nut_pose_received_ = false;
        sequence_ = 0;
        is_busy_ = false;

        std_msgs::msg::Bool trigger;
        trigger.data = true;
        trigger_pub_->publish(trigger);

        if (bin_index_ >= bin_locations_.size()) {
            RCLCPP_INFO(this->get_logger(), COLOR_CYAN "The simulation is completed!" COLOR_RESET);
        } else {
            RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Cycle complete. Requesting next nut pose." COLOR_RESET);
        }
    }

    void mainLoop()
    {
        if (!arm_group_ || !gripper_group_ || is_busy_) {
            if (!arm_group_ || !gripper_group_) {
                RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet.");
            }
            return;
        }

        if (bin_index_ >= bin_locations_.size()) {
            return;
        }

        if (!nut_pose_received_) {
            std_msgs::msg::Bool trigger;
            trigger.data = true;
            trigger_pub_->publish(trigger);
            return;
        }

        std::array<float, 4> local_nut{};
        if (!getNutPose(local_nut)) {
            requestNewDetection("Received invalid nut pose values (NaN/Inf)");
            return;
        }

        is_busy_ = true;
        std_msgs::msg::Bool trigger;
        trigger.data = false;
        trigger_pub_->publish(trigger);

        switch (sequence_) {
            case 0: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 0: Hover and align" COLOR_RESET);
                if (!moveArmToJointTarget(ready_joints_)) {
                    requestNewDetection("Failed to move to ready seed");
                    break;
                }

                const auto hover_pose = buildTopDownPose(
                    local_nut[0],
                    local_nut[1],
                    local_nut[2] + pregrasp_height_,
                    local_nut[3]);
                if (planAndExecutePose(hover_pose)) {
                    sequence_ = 1;
                } else {
                    requestNewDetection("Hover move failed");
                }
                break;
            }

            case 1: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 1: Open gripper" COLOR_RESET);
                if (moveGripper(gripper_open_cmd_)) {
                    sequence_ = 2;
                } else {
                    requestNewDetection("Open gripper failed");
                }
                break;
            }

            case 2: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 2: Descend to pick" COLOR_RESET);
                const double pick_tcp_z = std::max(local_nut[2] + tool_center_offset_, table_top_z_ + pick_clearance_z_);
                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Pick target using link '%s': nut_xyz=(%.4f, %.4f, %.4f) target_tcp_z=%.4f current_tcp_z=%.4f",
                    flange_link_name_.c_str(),
                    local_nut[0],
                    local_nut[1],
                    local_nut[2],
                    pick_tcp_z,
                    current_pose.position.z);

                if (!moveCartesian(local_nut[0], local_nut[1], current_pose.position.z, 0.30)) {
                    requestNewDetection("XY align before pick failed");
                    break;
                }

                auto pick_pose = buildTopDownPose(local_nut[0], local_nut[1], pick_tcp_z, local_nut[3]);
                if (moveToPoseWithFallback(pick_pose, 0.80)) {
                    sequence_ = 3;
                } else {
                    requestNewDetection("Pick descent failed");
                }
                break;
            }

            case 3: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 3: Close gripper" COLOR_RESET);
                if (!moveGripper(gripper_close_cmd_)) {
                    requestNewDetection("Close gripper failed");
                    break;
                }

                std::this_thread::sleep_for(std::chrono::duration<double>(gripper_settle_time_));
                if (likelyMissedGrasp()) {
                    requestNewDetection("Likely missed grasp (gripper reached empty-close threshold)");
                    break;
                }
                sequence_ = 4;
                break;
            }

            case 4: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 4: Lift" COLOR_RESET);
                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                if (moveCartesian(current_pose.position.x, current_pose.position.y, current_pose.position.z + lift_height_, 0.80)) {
                    sequence_ = 5;
                } else {
                    requestNewDetection("Lift failed");
                }
                break;
            }

            case 5: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 5: Move to place hover" COLOR_RESET);
                const auto place_hover_pose = buildTopDownPose(
                    bin_locations_[bin_index_][0],
                    bin_locations_[bin_index_][1],
                    bin_locations_[bin_index_][2] + place_hover_height_,
                    0.0);
                if (planAndExecutePose(place_hover_pose)) {
                    sequence_ = 6;
                } else {
                    requestNewDetection("Move to place failed");
                }
                break;
            }

            case 6: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 6: Descend to place" COLOR_RESET);
                const double place_tcp_z = std::max(
                    bin_locations_[bin_index_][2] + tool_center_offset_,
                    table_top_z_ + pick_clearance_z_);
                auto place_pose = buildTopDownPose(
                    bin_locations_[bin_index_][0],
                    bin_locations_[bin_index_][1],
                    place_tcp_z,
                    0.0);
                if (moveToPoseWithFallback(place_pose, 0.80)) {
                    sequence_ = 7;
                } else {
                    requestNewDetection("Place descent failed");
                }
                break;
            }

            case 7: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 7: Release" COLOR_RESET);
                if (!moveGripper(gripper_open_cmd_)) {
                    requestNewDetection("Release gripper failed");
                    break;
                }
                sequence_ = 8;
                break;
            }

            case 8: {
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 8: Retreat and reset" COLOR_RESET);
                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                const bool retreat_ok = moveCartesian(
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z + lift_height_,
                    0.80);
                const bool ready_ok = retreat_ok && moveArmToJointTarget(ready_joints_);
                if (!ready_ok) {
                    requestNewDetection("Retreat or return-to-ready failed");
                    break;
                }
                completeCycleAndRequestNext();
                break;
            }

            default:
                requestNewDetection("Unknown sequence state");
                break;
        }

        is_busy_ = false;
    }
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
