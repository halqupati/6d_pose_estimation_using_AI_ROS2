#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#define PI 3.14159265359
#define COLOR_GREEN "\033[32m"
#define COLOR_CYAN  "\033[36m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED   "\033[31m"
#define COLOR_RESET "\033[0m"

// Motion envelope enforced when robot_mode == "real" (physical JAKA S12).
// These are the conservative project defaults, NOT hardware-validated maxima.
// Relaxing them requires a documented test on the real cell (reduced-speed
// dry runs, gripper close-time measurement, descent-overshoot check), then a
// rebuild. The same numbers are mirrored in
// launch/run_bin_picking_nodes_to_control_jaka_s12.launch.py for early,
// pre-launch rejection; keep both in sync.
namespace real_robot_limits {
constexpr double kMaxArmVelocityScaling = 0.30;
constexpr double kMaxArmAccelerationScaling = 0.30;
constexpr double kMaxGraspDescentStep = 0.002;       // [m]
constexpr double kMinGripperMotionDuration = 1.20;   // [s]
}  // namespace real_robot_limits

class ControlRobot : public rclcpp::Node {
public:
    ControlRobot()
        : Node("bin_picking_node_v5_to_control_jaka_s12")
    {
        pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        {
            rcl_interfaces::msg::ParameterDescriptor mode_descriptor;
            mode_descriptor.description =
                "'real' (physical JAKA S12, conservative limits enforced) or 'sim' (Gazebo only). Read-only.";
            mode_descriptor.read_only = true;
            robot_mode_ = this->declare_parameter<std::string>("robot_mode", "real", mode_descriptor);
        }
        if (robot_mode_ != "real" && robot_mode_ != "sim") {
            RCLCPP_FATAL(this->get_logger(), "Invalid robot_mode '%s'. Expected 'real' or 'sim'.", robot_mode_.c_str());
            throw std::invalid_argument("invalid robot_mode: " + robot_mode_);
        }
        is_real_robot_ = (robot_mode_ == "real");

        arm_group_name_ = this->declare_parameter<std::string>("arm_group", "jaka_s12");
        gripper_group_name_ = this->declare_parameter<std::string>("gripper_group", "gripper");
        flange_link_name_ = this->declare_parameter<std::string>("flange_link", "grasp_tcp");
        ready_joints_ = this->declare_parameter<std::vector<double>>(
            "ready_joints", std::vector<double>{-1.57, 1.57, 1.57, 1.57, -1.57, 0.0});

        pregrasp_height_ = this->declare_parameter<double>("pregrasp_height", 0.20);
        place_hover_height_ = this->declare_parameter<double>("place_hover_height", 0.40);
        lift_height_ = this->declare_parameter<double>("lift_height", 0.10);
        tool_center_offset_ = this->declare_parameter<double>("tool_center_offset", 0.019);
        grasp_final_offset_ = this->declare_parameter<double>("grasp_final_offset", 0.0);
        grasp_descent_step_ = this->declare_parameter<double>("grasp_descent_step", 0.002);
        pick_approach_offset_ = this->declare_parameter<double>("pick_approach_offset", 0.020);
        pick_safe_gap_z_ = this->declare_parameter<double>("pick_safe_gap_z", 0.003);
        detected_nut_height_ = this->declare_parameter<double>("detected_nut_height", 0.008);
        min_grasp_tcp_above_table_ = this->declare_parameter<double>("min_grasp_tcp_above_table", 0.020);
        max_pick_descent_ = this->declare_parameter<double>("max_pick_descent", 0.25);
        pick_xy_tolerance_ = this->declare_parameter<double>("pick_xy_tolerance", 0.008);
        pick_z_tolerance_ = this->declare_parameter<double>("pick_z_tolerance", 0.010);
        pick_clearance_z_ = this->declare_parameter<double>("pick_clearance_z", 0.01);
        table_top_z_ = this->declare_parameter<double>("table_top_z", -0.06);

        arm_velocity_scaling_ = this->declare_parameter<double>("arm_velocity_scaling", 0.30);
        arm_acceleration_scaling_ = this->declare_parameter<double>("arm_acceleration_scaling", 0.30);
        arm_planning_time_ = this->declare_parameter<double>("arm_planning_time", 5.0);

        gripper_open_cmd_ = this->declare_parameter<double>("gripper_open_cmd", 0.0);
        gripper_close_cmd_ = this->declare_parameter<double>("gripper_close_cmd", -0.025);
        gripper_min_position_ = this->declare_parameter<double>("gripper_min_position", -0.026);
        gripper_max_position_ = this->declare_parameter<double>("gripper_max_position", 0.001);
        empty_close_threshold_ = this->declare_parameter<double>("empty_close_threshold", -0.023);
        gripper_motion_duration_ = this->declare_parameter<double>("gripper_motion_duration", 1.20);
        gripper_settle_time_ = this->declare_parameter<double>("gripper_settle_time", 0.1);
        // Added to the perception yaw (hex edge direction) so the jaws close on flats.
        grasp_yaw_offset_ = this->declare_parameter<double>("grasp_yaw_offset", PI / 6.0);

        // ---------------- Nut tightening tasks (docs/tightening/) ----------------
        //   "pick_place"   = the original bin-picking cycle
        //   "tighten"      = a nut already threaded on the bolt is run down to snug
        //   "pick_tighten" = pick a free nut, place it on the bolt tip, then tighten it
        task_mode_ = this->declare_parameter<std::string>("task_mode", "pick_place");
        // Bolt axis and nut, in the robot base frame. Defaults = worlds/bolt_m20 fixture
        // at world (-0.30, 0.75) on the table (z offset: base frame is 1.018 m below world).
        tighten_bolt_x_ = this->declare_parameter<double>("tighten_bolt_x", -0.30);
        tighten_bolt_y_ = this->declare_parameter<double>("tighten_bolt_y", 0.75);
        tighten_seat_z_ = this->declare_parameter<double>("tighten_seat_z", 0.017);          // plate top [m]
        tighten_bolt_tip_z_ = this->declare_parameter<double>("tighten_bolt_tip_z", 0.037);  // bolt end [m]
        tighten_nut_bottom_z_ = this->declare_parameter<double>("tighten_nut_bottom_z", 0.0245);  // start [m]
        tighten_nut_height_ = this->declare_parameter<double>("tighten_nut_height", 0.016);  // M20 [m]
        tighten_nut_yaw_ = this->declare_parameter<double>("tighten_nut_yaw", 0.0);          // corner axis [rad]
        thread_pitch_ = this->declare_parameter<double>("thread_pitch", 0.0025);             // M20 coarse [m/rev]
        // Turn per grip. A multiple of 60 deg, so after turning back the open jaws meet
        // two flats again; at most 180 deg so the rotation direction is unambiguous.
        tighten_stroke_deg_ = this->declare_parameter<double>("tighten_stroke_deg", 120.0);
        // Snug threshold, measured on the wrist F/T sensor. 1.5 N*m: ~150x the running torque
        // (0.01 N*m) and below the ~2.6 N*m at which the parallel jaws slip on a seated M20 nut
        // (docs/tightening/phase1_tighten_nut_on_bolt.md). 5 N*m is not reachable with this gripper.
        tighten_snug_torque_ = this->declare_parameter<double>("tighten_snug_torque", 1.5);  // [N*m]
        tighten_max_turns_ = this->declare_parameter<double>("tighten_max_turns", 4.0);      // safety stop
        tighten_velocity_scaling_ = this->declare_parameter<double>("tighten_velocity_scaling", 0.5);
        // Fingertip height above the nut's bottom face while gripping, and the distance
        // from grasp_tcp down to the fingertips (jaw pads end 22.5 mm below grasp_tcp).
        tighten_finger_clearance_ = this->declare_parameter<double>("tighten_finger_clearance", 0.003);
        jaw_tip_offset_ = this->declare_parameter<double>("jaw_tip_offset", 0.0225);
        wrist_ft_topic_ = this->declare_parameter<std::string>("wrist_ft_topic", "/wrist_ft");

        validateModeAtStartup();

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = pose_callback_group_;
        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle",
            10,
            std::bind(&ControlRobot::poseCallback, this, std::placeholders::_1),
            sub_options);

        // Wrist force-torque sensor. Own callback group so the torque monitor keeps running
        // (and can stop the arm) while mainLoop is blocked executing a turning stroke.
        wrench_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions wrench_options;
        wrench_options.callback_group = wrench_callback_group_;
        wrist_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrist_ft_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ControlRobot::onWristWrench, this, std::placeholders::_1),
            wrench_options);

        // Simulation-only services of bolt_fixture_manager.py that give a nut its thread.
        // Own reentrant group so their responses arrive while mainLoop waits for them.
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        preload_nut_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/bolt_fixture/preload_nut", rmw_qos_profile_services_default, service_callback_group_);
        engage_nut_client_ = this->create_client<std_srvs::srv::Trigger>(
            "/bolt_fixture/engage_nut", rmw_qos_profile_services_default, service_callback_group_);

        trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);
        pose_received_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pose_received", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControlRobot::mainLoop, this),
            timer_callback_group_);

        collision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ControlRobot::addCollisionObjects, this));

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControlRobot::onParametersChanged, this, std::placeholders::_1));

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
    std::string robot_mode_ = "real";
    bool is_real_robot_ = true;
    bool sim_clock_block_logged_ = false;
    // mainLoop runs on a Reentrant timer group under a MultiThreadedExecutor, so a
    // new tick can fire while a long MoveIt call is in progress. This mutex keeps
    // mainLoop single-entry (try_lock) instead of relying on the unsynchronised
    // is_busy_ flag alone.
    std::mutex main_loop_mutex_;

    int sequence_ = 0;
    bool is_busy_ = false;
    std::atomic<bool> nut_pose_received_{false};  // written from poseCallback thread, read in mainLoop
    bool move_group_wait_logged_ = false;
    std::array<float, 4> nut_pose_angle_{};
    std::mutex pose_mutex_;
    size_t bin_index_ = 0;
    std::string last_launcher_case_;
    std::string last_launcher_detail_;
    size_t last_launcher_picked_count_ = static_cast<size_t>(-1);
    std::chrono::steady_clock::time_point detection_received_time_{};
    bool detect_to_pick_timer_active_ = false;
    double last_detect_to_pick_sec_ = 0.0;

    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::string flange_link_name_;
    std::vector<double> ready_joints_;

    double pregrasp_height_ = 0.20;
    double place_hover_height_ = 0.40;
    double lift_height_ = 0.10;
    double tool_center_offset_ = 0.019;  // Approach offset in world Z for the grasp frame
    double grasp_final_offset_ = 0.0;    // Final grasp offset in world Z for the grasp frame
    double grasp_descent_step_ = 0.002;  // Step size for final vertical descent
    double pick_approach_offset_ = 0.020;
    double pick_safe_gap_z_ = 0.003;     // Keep a small margin above the final grasp plane in seq 2
    double detected_nut_height_ = 0.008; // Approximate nut thickness; detection z is the visible top surface
    double min_grasp_tcp_above_table_ = 0.020;
    double max_pick_descent_ = 0.25;
    double pick_xy_tolerance_ = 0.008;
    double pick_z_tolerance_ = 0.010;
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
    double gripper_motion_duration_ = 1.20;
    double gripper_settle_time_ = 0.1;
    double grasp_yaw_offset_ = PI / 6.0;

    // Tightening task (see the parameter declarations in the constructor).
    std::string task_mode_ = "pick_place";
    double tighten_bolt_x_ = -0.30;
    double tighten_bolt_y_ = 0.75;
    double tighten_seat_z_ = 0.017;
    double tighten_bolt_tip_z_ = 0.037;
    double tighten_nut_bottom_z_ = 0.0245;
    double tighten_nut_height_ = 0.016;
    double tighten_nut_yaw_ = 0.0;
    double thread_pitch_ = 0.0025;
    double tighten_stroke_deg_ = 120.0;
    double tighten_snug_torque_ = 1.5;
    double tighten_max_turns_ = 4.0;
    double tighten_velocity_scaling_ = 0.5;
    double tighten_finger_clearance_ = 0.003;
    double jaw_tip_offset_ = 0.0225;
    std::string wrist_ft_topic_ = "/wrist_ft";
    bool tighten_finished_ = false;   // one tightening run per node start (success or failure)

    // Torque monitor state, shared between the wrench callback thread and mainLoop.
    std::atomic<bool> torque_monitor_armed_{false};
    std::atomic<bool> snug_detected_{false};
    std::atomic<double> wrist_torque_z_{0.0};       // latest raw torque about the tool axis
    std::atomic<double> torque_baseline_{0.0};      // value just before the stroke started
    std::atomic<double> filtered_net_torque_{0.0};  // |torque - baseline|, low-pass filtered
    std::atomic<double> peak_net_torque_{0.0};
    std::atomic<bool> wrist_wrench_received_{false};

    std::array<std::array<double, 3>, 3> bin_locations_ = {{
        {0.5, 0.50, 0.05},
        {0.5, 0.55, 0.05},
        {0.3, -0.60, 0.05},
    }};

    rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::CallbackGroup::SharedPtr wrench_callback_group_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrist_wrench_sub_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr preload_nut_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engage_nut_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pose_received_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr collision_timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    geometry_msgs::msg::Pose validated_pick_pose_{};
    bool validated_pick_pose_valid_ = false;

    // Returns an empty string when the profile is inside the real-robot envelope.
    static std::string realRobotLimitViolation(
        double velocity, double acceleration, double descent_step, double gripper_duration)
    {
        using namespace real_robot_limits;
        char buffer[256];
        if (velocity > kMaxArmVelocityScaling) {
            std::snprintf(buffer, sizeof(buffer), "arm_velocity_scaling=%.3f exceeds real-robot max %.2f",
                          velocity, kMaxArmVelocityScaling);
            return buffer;
        }
        if (acceleration > kMaxArmAccelerationScaling) {
            std::snprintf(buffer, sizeof(buffer), "arm_acceleration_scaling=%.3f exceeds real-robot max %.2f",
                          acceleration, kMaxArmAccelerationScaling);
            return buffer;
        }
        if (descent_step > kMaxGraspDescentStep + 1e-9) {
            std::snprintf(buffer, sizeof(buffer), "grasp_descent_step=%.4f m exceeds real-robot max %.4f m",
                          descent_step, kMaxGraspDescentStep);
            return buffer;
        }
        if (gripper_duration < kMinGripperMotionDuration - 1e-9) {
            std::snprintf(buffer, sizeof(buffer), "gripper_motion_duration=%.2f s is below real-robot min %.2f s",
                          gripper_duration, kMinGripperMotionDuration);
            return buffer;
        }
        return "";
    }

    void validateModeAtStartup()
    {
        const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (is_real_robot_ && use_sim_time) {
            RCLCPP_FATAL(this->get_logger(),
                         "robot_mode=real with use_sim_time=true is inconsistent. A physical JAKA S12 has no "
                         "simulator clock. Use robot_mode:=sim for Gazebo.");
            throw std::runtime_error("robot_mode/use_sim_time mismatch");
        }
        if (!is_real_robot_ && !use_sim_time) {
            RCLCPP_FATAL(this->get_logger(),
                         "robot_mode=sim requires use_sim_time=true. Refusing to start so simulation motion "
                         "settings cannot drive hardware.");
            throw std::runtime_error("robot_mode/use_sim_time mismatch");
        }

        if (task_mode_ != "pick_place" && task_mode_ != "tighten" && task_mode_ != "pick_tighten") {
            RCLCPP_FATAL(this->get_logger(), "Invalid task_mode '%s'. Expected pick_place, tighten or pick_tighten.",
                         task_mode_.c_str());
            throw std::invalid_argument("invalid task_mode: " + task_mode_);
        }
        if (task_mode_ == "tighten" || task_mode_ == "pick_tighten") {
            if (is_real_robot_) {
                // Not validated on hardware: needs a real wrist F/T signal, a verified grip
                // torque limit and a real bolt pose. Simulation only for now.
                RCLCPP_FATAL(this->get_logger(), "task_mode=%s is simulation-only (robot_mode=sim).", task_mode_.c_str());
                throw std::runtime_error("task_mode=" + task_mode_ + " requires robot_mode=sim");
            }
            const double sixties = tighten_stroke_deg_ / 60.0;
            if (tighten_stroke_deg_ <= 0.0 || tighten_stroke_deg_ > 180.0 ||
                std::abs(sixties - std::round(sixties)) > 1e-6) {
                RCLCPP_FATAL(this->get_logger(),
                             "tighten_stroke_deg=%.1f must be 60, 120 or 180 (a multiple of the hexagon's "
                             "60 deg symmetry, at most 180).", tighten_stroke_deg_);
                throw std::invalid_argument("invalid tighten_stroke_deg");
            }
            if (tighten_snug_torque_ <= 0.0 || thread_pitch_ <= 0.0 || tighten_max_turns_ <= 0.0) {
                RCLCPP_FATAL(this->get_logger(), "tighten_snug_torque, thread_pitch and tighten_max_turns must be > 0.");
                throw std::invalid_argument("invalid tightening parameters");
            }
            RCLCPP_INFO(this->get_logger(),
                        "task_mode=%s: bolt (%.3f, %.3f) seat_z=%.4f tip_z=%.4f stroke=%.0f deg "
                        "snug=%.2f N*m pitch=%.4f m/rev",
                        task_mode_.c_str(), tighten_bolt_x_, tighten_bolt_y_, tighten_seat_z_, tighten_bolt_tip_z_,
                        tighten_stroke_deg_, tighten_snug_torque_, thread_pitch_);
        }

        if (is_real_robot_) {
            const std::string violation = realRobotLimitViolation(
                arm_velocity_scaling_, arm_acceleration_scaling_, grasp_descent_step_, gripper_motion_duration_);
            if (!violation.empty()) {
                RCLCPP_FATAL(this->get_logger(),
                             "Refusing to start in robot_mode=real: %s. Simulation presets must not be used "
                             "on the physical robot.", violation.c_str());
                throw std::runtime_error("real-robot motion limits violated: " + violation);
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "robot_mode=%s use_sim_time=%s | velocity=%.2f acceleration=%.2f descent_step=%.4f "
                    "gripper_motion_duration=%.2f",
                    robot_mode_.c_str(), use_sim_time ? "true" : "false",
                    arm_velocity_scaling_, arm_acceleration_scaling_, grasp_descent_step_, gripper_motion_duration_);
    }

    rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        double new_arm_velocity_scaling = arm_velocity_scaling_;
        double new_arm_acceleration_scaling = arm_acceleration_scaling_;
        double new_grasp_descent_step = grasp_descent_step_;
        double new_gripper_motion_duration = gripper_motion_duration_;
        double new_pick_safe_gap_z = pick_safe_gap_z_;
        double new_detected_nut_height = detected_nut_height_;
        double new_min_grasp_tcp_above_table = min_grasp_tcp_above_table_;
        bool motion_profile_requested = false;
        bool pick_timing_requested = false;
        bool pick_safety_requested = false;

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "arm_velocity_scaling") {
                const double value = parameter.as_double();
                if (value <= 0.0 || value > 1.0) {
                    result.successful = false;
                    result.reason = "arm_velocity_scaling must be in (0.0, 1.0].";
                    return result;
                }
                new_arm_velocity_scaling = value;
                motion_profile_requested = true;
            } else if (parameter.get_name() == "arm_acceleration_scaling") {
                const double value = parameter.as_double();
                if (value <= 0.0 || value > 1.0) {
                    result.successful = false;
                    result.reason = "arm_acceleration_scaling must be in (0.0, 1.0].";
                    return result;
                }
                new_arm_acceleration_scaling = value;
                motion_profile_requested = true;
            } else if (parameter.get_name() == "grasp_descent_step") {
                const double value = parameter.as_double();
                if (value <= 0.0) {
                    result.successful = false;
                    result.reason = "grasp_descent_step must be > 0.0.";
                    return result;
                }
                new_grasp_descent_step = value;
                pick_timing_requested = true;
            } else if (parameter.get_name() == "gripper_motion_duration") {
                const double value = parameter.as_double();
                if (value <= 0.0) {
                    result.successful = false;
                    result.reason = "gripper_motion_duration must be > 0.0.";
                    return result;
                }
                new_gripper_motion_duration = value;
                pick_timing_requested = true;
            } else if (parameter.get_name() == "pick_safe_gap_z") {
                const double value = parameter.as_double();
                if (value < 0.0) {
                    result.successful = false;
                    result.reason = "pick_safe_gap_z must be >= 0.0.";
                    return result;
                }
                new_pick_safe_gap_z = value;
                pick_safety_requested = true;
            } else if (parameter.get_name() == "detected_nut_height") {
                const double value = parameter.as_double();
                if (value <= 0.0) {
                    result.successful = false;
                    result.reason = "detected_nut_height must be > 0.0.";
                    return result;
                }
                new_detected_nut_height = value;
                pick_safety_requested = true;
            } else if (parameter.get_name() == "min_grasp_tcp_above_table") {
                const double value = parameter.as_double();
                if (value <= 0.0) {
                    result.successful = false;
                    result.reason = "min_grasp_tcp_above_table must be > 0.0.";
                    return result;
                }
                new_min_grasp_tcp_above_table = value;
                pick_safety_requested = true;
            }
        }

        if (is_real_robot_ && (motion_profile_requested || pick_timing_requested)) {
            const std::string violation = realRobotLimitViolation(
                new_arm_velocity_scaling, new_arm_acceleration_scaling,
                new_grasp_descent_step, new_gripper_motion_duration);
            if (!violation.empty()) {
                result.successful = false;
                result.reason = "Rejected in robot_mode=real: " + violation;
                RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                return result;
            }
        }

        arm_velocity_scaling_ = new_arm_velocity_scaling;
        arm_acceleration_scaling_ = new_arm_acceleration_scaling;
        grasp_descent_step_ = new_grasp_descent_step;
        gripper_motion_duration_ = new_gripper_motion_duration;
        pick_safe_gap_z_ = new_pick_safe_gap_z;
        detected_nut_height_ = new_detected_nut_height;
        min_grasp_tcp_above_table_ = new_min_grasp_tcp_above_table;

        if (arm_group_ && motion_profile_requested) {
            arm_group_->setMaxVelocityScalingFactor(arm_velocity_scaling_);
            arm_group_->setMaxAccelerationScalingFactor(arm_acceleration_scaling_);
        }

        if (motion_profile_requested) {
            RCLCPP_INFO(
                this->get_logger(),
                "Updated motion profile: velocity_scaling=%.2f acceleration_scaling=%.2f",
                arm_velocity_scaling_,
                arm_acceleration_scaling_);
        }

        if (pick_timing_requested) {
            RCLCPP_INFO(
                this->get_logger(),
                "Updated pick timing: grasp_descent_step=%.4f gripper_motion_duration=%.2f",
                grasp_descent_step_,
                gripper_motion_duration_);
        }

        if (pick_safety_requested) {
            RCLCPP_INFO(
                this->get_logger(),
                "Updated pick safety: pick_safe_gap_z=%.4f detected_nut_height=%.4f min_grasp_tcp_above_table=%.4f",
                pick_safe_gap_z_,
                detected_nut_height_,
                min_grasp_tcp_above_table_);
        }
        return result;
    }

    void getDetectToPickSnapshot(double &seconds_out, std::string &state_out)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (detect_to_pick_timer_active_) {
            seconds_out = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - detection_received_time_).count();
            state_out = "running";
            return;
        }

        if (last_detect_to_pick_sec_ > 0.0) {
            seconds_out = last_detect_to_pick_sec_;
            state_out = "last";
            return;
        }

        seconds_out = 0.0;
        state_out = "idle";
    }

    void finalizeDetectToPickTimer()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!detect_to_pick_timer_active_) {
            return;
        }

        last_detect_to_pick_sec_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - detection_received_time_).count();
        detect_to_pick_timer_active_ = false;
    }

    void stopDetectToPickTimer()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        detect_to_pick_timer_active_ = false;
    }

    void publishLauncherStatus(const std::string &robot_case, const std::string &detail = "")
    {
        const size_t picked_count = bin_index_;
        if (robot_case == last_launcher_case_ &&
            detail == last_launcher_detail_ &&
            picked_count == last_launcher_picked_count_) {
            return;
        }

        last_launcher_case_ = robot_case;
        last_launcher_detail_ = detail;
        last_launcher_picked_count_ = picked_count;
        double detect_to_pick_sec = 0.0;
        std::string detect_to_pick_state;
        getDetectToPickSnapshot(detect_to_pick_sec, detect_to_pick_state);

        RCLCPP_INFO(
            this->get_logger(),
            "[LAUNCHER_STATUS] robot_case=%s; sequence=%d; picked_count=%zu; pick_time_sec=%.2f; "
            "pick_timer_state=%s; detail=%s",
            robot_case.c_str(),
            sequence_,
            picked_count,
            detect_to_pick_sec,
            detect_to_pick_state.c_str(),
            detail.empty() ? "-" : detail.c_str());
    }

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
        detection_received_time_ = std::chrono::steady_clock::now();
        detect_to_pick_timer_active_ = true;
        std_msgs::msg::Bool ack;
        ack.data = true;
        pose_received_pub_->publish(ack);
        RCLCPP_INFO(this->get_logger(), "Nut pose received.");
    }

    bool getNutPose(std::array<float, 4> &local_nut)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        local_nut = nut_pose_angle_;
        // local_nut[3] becomes the GRIPPER yaw: the jaws close along the tool x axis,
        // while the perception yaw is the hexagon edge direction (= corner-to-corner
        // axis). +30 deg puts the jaws on two opposite flats instead of two corners.
        local_nut[3] = static_cast<float>(local_nut[3] + grasp_yaw_offset_);
        return std::isfinite(local_nut[0]) && std::isfinite(local_nut[1]) &&
               std::isfinite(local_nut[2]) && std::isfinite(local_nut[3]);
    }

    void requestNewDetection(const char *reason)
    {
        RCLCPP_WARN(this->get_logger(), "%s. Resetting sequence and requesting a new pose.", reason);
        sequence_ = 0;
        is_busy_ = false;
        nut_pose_received_ = false;
        validated_pick_pose_valid_ = false;
        stopDetectToPickTimer();
        publishLauncherStatus("Request new detection", reason);
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

    bool isPoseCloseToCurrent(const geometry_msgs::msg::Pose &target, double pos_tol = 0.001, double ang_tol = 0.03)
    {
        const auto current = arm_group_->getCurrentPose(flange_link_name_).pose;
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
        return q_cur.angleShortestPath(q_tgt) <= ang_tol;
    }

    double estimateTableSurfaceZ(double nut_top_z) const
    {
        return std::max(table_top_z_, nut_top_z - detected_nut_height_);
    }

    double computeMinimumSafeTcpZ(double nut_z) const
    {
        return estimateTableSurfaceZ(nut_z) + min_grasp_tcp_above_table_;
    }

    double computePickTcpZ(double nut_z) const
    {
        return std::max(
            nut_z + grasp_final_offset_ + pick_safe_gap_z_,
            computeMinimumSafeTcpZ(nut_z));
    }

    bool getCurrentArmPose(geometry_msgs::msg::Pose &pose_out)
    {
        if (!arm_group_) {
            return false;
        }
        pose_out = arm_group_->getCurrentPose(flange_link_name_).pose;
        return true;
    }

    // Like isCurrentPoseNearTarget, but tolerates joint-state lag right after a trajectory
    // finishes (pronounced when Gazebo runs below real time): polls for up to timeout_sec and
    // logs the remaining error if the pose never converges. Intermittent "Pick pose validation
    // failed" resets were seen both in earlier runs and in testing with a fully executed path.
    bool waitUntilPoseNearTarget(
        const geometry_msgs::msg::Pose &target,
        double xy_tol,
        double z_tol,
        double timeout_sec = 1.5)
    {
        const auto start = std::chrono::steady_clock::now();
        while (true) {
            if (isCurrentPoseNearTarget(target, xy_tol, z_tol)) {
                return true;
            }
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= timeout_sec) {
                break;
            }
            (void)waitForRobotState(0.1);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        geometry_msgs::msg::Pose current;
        if (getCurrentArmPose(current)) {
            tf2::Quaternion q_cur, q_tgt;
            tf2::fromMsg(current.orientation, q_cur);
            tf2::fromMsg(target.orientation, q_tgt);
            RCLCPP_WARN(
                this->get_logger(),
                "Pose did not converge in %.1f s: xy_err=%.4f (tol %.4f) z_err=%.4f (tol %.4f) ang_err=%.3f rad",
                timeout_sec,
                std::hypot(current.position.x - target.position.x, current.position.y - target.position.y),
                xy_tol,
                std::abs(current.position.z - target.position.z),
                z_tol,
                q_cur.normalized().angleShortestPath(q_tgt.normalized()));
        }
        return false;
    }

    bool isCurrentPoseNearTarget(
        const geometry_msgs::msg::Pose &target,
        double xy_tol,
        double z_tol,
        double ang_tol = 0.05)
    {
        geometry_msgs::msg::Pose current;
        if (!getCurrentArmPose(current)) {
            return false;
        }

        const double dx = current.position.x - target.position.x;
        const double dy = current.position.y - target.position.y;
        const double xy_err = std::sqrt(dx * dx + dy * dy);
        const double z_err = std::abs(current.position.z - target.position.z);
        if (xy_err > xy_tol || z_err > z_tol) {
            return false;
        }

        tf2::Quaternion q_cur, q_tgt;
        tf2::fromMsg(current.orientation, q_cur);
        tf2::fromMsg(target.orientation, q_tgt);
        q_cur.normalize();
        q_tgt.normalize();
        return q_cur.angleShortestPath(q_tgt) <= ang_tol;
    }

    bool isGripperNearCommand(double command, double tolerance = 0.002)
    {
        double current_pos = 0.0;
        if (!getCurrentGripperPosition(current_pos)) {
            return false;
        }
        const double target_pos = std::clamp(command, gripper_min_position_, gripper_max_position_);
        return std::abs(current_pos - target_pos) <= tolerance;
    }

    bool ensureGripperOpenForPick()
    {
        if (isGripperNearCommand(gripper_open_cmd_)) {
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Gripper is not at the configured open command before pick descent. Re-opening first.");
        return moveGripper(gripper_open_cmd_);
    }

    bool executeTimedTrajectory(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &move_group,
        const std::string &planning_group,
        moveit_msgs::msg::RobotTrajectory trajectory_msg,
        double velocity_scaling,
        double acceleration_scaling,
        bool retime_trajectory = true)
    {
        if (trajectory_msg.joint_trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Refusing to execute empty trajectory.");
            return false;
        }

        auto current_state = move_group->getCurrentState(0.2);
        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current robot state unavailable for trajectory retiming.");
            return false;
        }

        if (retime_trajectory) {
            robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), planning_group);
            rt.setRobotTrajectoryMsg(*current_state, trajectory_msg);

            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling)) {
                RCLCPP_WARN(this->get_logger(), "Trajectory time parameterization failed.");
                return false;
            }

            rt.getRobotTrajectoryMsg(trajectory_msg);
        }
        auto &pts = trajectory_msg.joint_trajectory.points;
        auto &joint_names = trajectory_msg.joint_trajectory.joint_names;
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

        if (!pts.empty() && !joint_names.empty()) {
            auto &first_point = pts.front();
            first_point.positions.resize(joint_names.size(), 0.0);
            first_point.velocities.assign(joint_names.size(), 0.0);
            first_point.accelerations.assign(joint_names.size(), 0.0);
            first_point.time_from_start.sec = 0;
            first_point.time_from_start.nanosec = 0;

            for (size_t idx = 0; idx < joint_names.size(); ++idx) {
                const auto *joint_model = current_state->getRobotModel()->getJointModel(joint_names[idx]);
                if (!joint_model) {
                    continue;
                }

                const double *joint_position = current_state->getJointPositions(joint_model);
                if (!joint_position) {
                    continue;
                }

                first_point.positions[idx] = joint_position[0];
            }
        }

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
        if (isPoseCloseToCurrent(target)) {
            return true;
        }

        arm_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Preferred: IK seeded from the CURRENT joint state, then plan to that joint goal.
        // A plain pose goal lets MoveIt pick any IK branch (elbow/wrist flip), and OMPL then
        // swings the arm (and the held nut) through large random arcs (observed: nut carried
        // up to z=2.2 m and through y=-1.06 m on the way to the place slot).
        bool planned = false;
        if (const auto current_state = arm_group_->getCurrentState(1.0)) {
            arm_group_->setJointValueTarget(*current_state);  // seed for the IK below
            if (arm_group_->setJointValueTarget(target, flange_link_name_)) {
                planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
        }

        if (!planned) {
            RCLCPP_WARN(this->get_logger(), "Seeded-IK joint goal unavailable; falling back to pose-goal planning.");
            arm_group_->setStartStateToCurrentState();
            arm_group_->setPoseTarget(target, flange_link_name_);
            const auto plan_result = arm_group_->plan(plan);
            arm_group_->clearPoseTargets();
            if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "Pose planning failed.");
                return false;
            }
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, plan.trajectory_, arm_velocity_scaling_, arm_acceleration_scaling_, true);
    }

    bool moveArmToJointTarget(const std::vector<double> &joints)
    {
        const auto current = arm_group_->getCurrentJointValues();
        if (current.size() == joints.size()) {
            double max_abs_err = 0.0;
            for (size_t idx = 0; idx < joints.size(); ++idx) {
                max_abs_err = std::max(max_abs_err, std::abs(current[idx] - joints[idx]));
            }
            if (max_abs_err < 1e-3) {
                return true;
            }
        }

        arm_group_->setStartStateToCurrentState();
        arm_group_->setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Joint-space planning failed.");
            return false;
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, plan.trajectory_, arm_velocity_scaling_, arm_acceleration_scaling_, true);
    }

    bool moveCartesian(double x, double y, double z, double min_fraction = 0.90, bool avoid_collisions = true)
    {
        geometry_msgs::msg::Pose target = arm_group_->getCurrentPose(flange_link_name_).pose;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        return moveCartesianPose(target, min_fraction, avoid_collisions);
    }

    bool moveCartesianPose(
        const geometry_msgs::msg::Pose &target,
        double min_fraction = 0.90,
        bool avoid_collisions = true)
    {
        if (isPoseCloseToCurrent(target)) {
            return true;
        }

        (void)waitForRobotState(0.5);
        arm_group_->setStartStateToCurrentState();

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm_group_->computeCartesianPath(
            waypoints, 0.005, 0.0, trajectory, avoid_collisions);
        if (fraction < min_fraction) {
            RCLCPP_WARN(
                this->get_logger(),
                "Cartesian path fraction too low: %.3f (need %.3f), avoid_collisions=%s",
                fraction,
                min_fraction,
                avoid_collisions ? "true" : "false");
            return false;
        }

        return executeTimedTrajectory(
            arm_group_, arm_group_name_, trajectory, arm_velocity_scaling_, arm_acceleration_scaling_, true);
    }

    bool moveToPoseWithFallback(
        const geometry_msgs::msg::Pose &target,
        double min_fraction = 0.90,
        bool avoid_collisions = true)
    {
        if (moveCartesianPose(target, min_fraction, avoid_collisions)) {
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Cartesian move failed%s, falling back to pose planning.",
            avoid_collisions ? "" : " with collision checking disabled");
        return planAndExecutePose(target);
    }

    bool moveVerticalToPoseInSteps(
        const geometry_msgs::msg::Pose &target,
        double step_size,
        double min_fraction = 0.80,
        bool avoid_collisions = true,
        bool allow_partial_completion = false)
    {
        const auto start_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
        const double dz = target.position.z - start_pose.position.z;
        if (std::abs(dz) < 1e-4) {
            return true;
        }

        if (!is_real_robot_) {
            // Simulation: computeCartesianPath already interpolates the straight line
            // (5 mm eef_step), so one trajectory gives the same path without the per-step
            // start/stop overhead (~26 + 10 separate executions per pick before).
            // The physical robot keeps the conservative stepped descent below.
            return moveCartesianPose(target, min_fraction, avoid_collisions);
        }

        const double effective_step = std::max(0.001, std::abs(step_size));
        const int step_count = std::max(1, static_cast<int>(std::ceil(std::abs(dz) / effective_step)));

        bool reached_intermediate = false;
        for (int idx = 1; idx <= step_count; ++idx) {
            geometry_msgs::msg::Pose step_pose = target;
            const double alpha = static_cast<double>(idx) / static_cast<double>(step_count);
            step_pose.position.z = start_pose.position.z + dz * alpha;

            if (!moveCartesianPose(step_pose, min_fraction, avoid_collisions)) {
                if (reached_intermediate && allow_partial_completion) {
                    const auto settled_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Vertical move stopped early at tcp_z=%.4f before requested tcp_z=%.4f. "
                        "Proceeding with the deepest reachable pose.",
                        settled_pose.position.z,
                        target.position.z);
                    return true;
                }

                RCLCPP_WARN(
                    this->get_logger(),
                    "Vertical move failed on first step toward tcp_z=%.4f.",
                    target.position.z);
                return false;
            }

            reached_intermediate = true;
        }

        return true;
    }

    bool getCurrentGripperPosition(double &position_out)
    {
        auto current_state = gripper_group_->getCurrentState(0.2);
        if (!current_state) {
            return false;
        }

        const auto *joint_model_group = current_state->getRobotModel()->getJointModelGroup(gripper_group_name_);
        if (!joint_model_group) {
            return false;
        }

        std::vector<double> positions;
        current_state->copyJointGroupPositions(joint_model_group, positions);
        if (positions.empty()) {
            return false;
        }

        position_out = positions[0];
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
        return executeTimedTrajectory(gripper_group_, gripper_group_name_, trajectory_msg, 1.0, 1.0, false);
    }

    bool moveGripper(double command, bool allow_settle_timeout = false)
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
        const double settle_timeout = std::max(0.5, gripper_motion_duration_ + gripper_settle_time_);
        if (waitForGripperSettle(start_pos, target_pos, settle_timeout)) {
            return true;
        }

        double current_pos = start_pos;
        (void)getCurrentGripperPosition(current_pos);

        if (allow_settle_timeout) {
            RCLCPP_WARN(
                this->get_logger(),
                "Proceeding after gripper settle timeout. start=%.4f current=%.4f target=%.4f",
                start_pos,
                current_pos,
                target_pos);
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Gripper settle timeout treated as failure. start=%.4f current=%.4f target=%.4f",
            start_pos,
            current_pos,
            target_pos);
        return false;
    }

    bool closeGripperForGrasp()
    {
        double start_pos = 0.0;
        if (!getCurrentGripperPosition(start_pos)) {
            RCLCPP_WARN(this->get_logger(), "Gripper state unavailable before close.");
            return false;
        }

        if (!moveGripper(gripper_close_cmd_, true)) {
            return false;
        }

        double final_pos = start_pos;
        (void)getCurrentGripperPosition(final_pos);
        if (std::abs(final_pos - start_pos) < 5e-4) {
            RCLCPP_WARN(
                this->get_logger(),
                "Gripper close command produced negligible jaw motion. start=%.4f final=%.4f",
                start_pos,
                final_pos);
            return false;
        }

        if (likelyMissedGrasp()) {
            RCLCPP_WARN(this->get_logger(), "Likely missed grasp (gripper reached empty-close threshold).");
            return false;
        }

        return true;
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




        moveit_msgs::msg::CollisionObject table2_obj;
        table2_obj.header.frame_id = "world";
        table2_obj.id = "simulation_table2";


        geometry_msgs::msg::Pose table2_pose;
        table2_pose.position.x = 0.0;
        table2_pose.position.y = 0.81;
        table2_pose.position.z = -0.507;
        table2_pose.orientation.w = 1.0;

        table2_obj.primitives.push_back(table_primitive);
        table2_obj.primitive_poses.push_back(table2_pose);
        table2_obj.operation = table2_obj.ADD;


        psi.applyCollisionObject(table_obj);
        psi.applyCollisionObject(table2_obj);
        collision_timer_->cancel();
    }

    void completeCycleAndRequestNext()
    {
        ++bin_index_;
        nut_pose_received_ = false;
        sequence_ = 0;
        is_busy_ = false;
        validated_pick_pose_valid_ = false;
        stopDetectToPickTimer();

        std_msgs::msg::Bool trigger;
        trigger.data = true;
        trigger_pub_->publish(trigger);

        if (bin_index_ >= bin_locations_.size()) {
            publishLauncherStatus("Completed", "All configured place slots finished");
            RCLCPP_INFO(this->get_logger(), COLOR_CYAN "The simulation is completed!" COLOR_RESET);
        } else {
            publishLauncherStatus("Waiting for next nut pose", "Previous nut placed successfully");
            RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Cycle complete. Requesting next nut pose." COLOR_RESET);
        }
    }

    // ============================ Nut tightening (task_mode = "tighten") ============================
    //
    // The nut is already on the bolt. The gripper grips two flats, turns the nut clockwise one
    // stroke while following the thread down, lets go, turns back and grips again, until the
    // wrist force-torque sensor reports the snug torque (the nut is seated on the plate).

    static std::string format(const char *fmt, ...)
    {
        char buffer[256];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        return buffer;
    }

    // Runs on the wrench callback group (in parallel with mainLoop). While a stroke is armed,
    // it watches the torque about the tool axis and stops the arm once the nut is snug.
    void onWristWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        const double torque_z = msg->wrench.torque.z;  // sensor frame = gripper body, z = tool axis
        wrist_torque_z_.store(torque_z);
        wrist_wrench_received_.store(true);
        if (!torque_monitor_armed_.load()) {
            return;
        }

        // Low-pass filter (~25 ms time constant at 200 Hz): one noisy contact sample
        // must not end the tightening.
        constexpr double kAlpha = 0.2;
        const double net = std::abs(torque_z - torque_baseline_.load());
        const double filtered = (1.0 - kAlpha) * filtered_net_torque_.load() + kAlpha * net;
        filtered_net_torque_.store(filtered);
        if (filtered > peak_net_torque_.load()) {
            peak_net_torque_.store(filtered);
        }

        if (filtered >= tighten_snug_torque_ && !snug_detected_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(), "Snug torque reached: %.2f N*m (threshold %.2f N*m). Stopping the stroke.",
                        filtered, tighten_snug_torque_);
            if (arm_group_) {
                arm_group_->stop();  // cancels the running trajectory; execute() then returns
            }
        }
    }

    // Start watching the torque for one stroke. The baseline removes any static torque
    // the grip itself produces.
    void armTorqueMonitor()
    {
        torque_baseline_.store(wrist_torque_z_.load());
        filtered_net_torque_.store(0.0);
        peak_net_torque_.store(0.0);
        snug_detected_.store(false);
        torque_monitor_armed_.store(true);
    }

    void disarmTorqueMonitor()
    {
        torque_monitor_armed_.store(false);
    }

    // Direction of the jaw closing axis (tool x) in the horizontal plane [rad].
    static double toolYaw(const geometry_msgs::msg::Pose &pose)
    {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        const tf2::Vector3 x_axis = tf2::quatRotate(q, tf2::Vector3(1.0, 0.0, 0.0));
        return std::atan2(x_axis.y(), x_axis.x());
    }

    static double wrapAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    // Waypoints that turn the tool about the vertical axis through `start` by `angle`
    // (negative = clockwise seen from above = tightening). With follow_thread the tool also
    // moves along the axis by thread_pitch per revolution (down while tightening), exactly
    // like the nut. 30-degree steps keep the orientation interpolation unambiguous.
    std::vector<geometry_msgs::msg::Pose> helixWaypoints(
        const geometry_msgs::msg::Pose &start, double start_yaw, double angle, bool follow_thread)
    {
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / (PI / 6.0))));
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (int idx = 1; idx <= steps; ++idx) {
            const double partial = angle * idx / steps;
            const double dz = follow_thread ? thread_pitch_ * partial / (2.0 * PI) : 0.0;
            waypoints.push_back(buildTopDownPose(
                start.position.x, start.position.y, start.position.z + dz, start_yaw + partial));
        }
        return waypoints;
    }

    // One Cartesian trajectory through all waypoints (collision checking off: the gripper is
    // intentionally in contact with the nut), time-parameterised at the given scaling.
    bool executeCartesianWaypoints(
        const std::vector<geometry_msgs::msg::Pose> &waypoints, double velocity_scaling, double min_fraction = 0.95)
    {
        (void)waitForRobotState(0.5);
        arm_group_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm_group_->computeCartesianPath(waypoints, 0.002, 0.0, trajectory, false);
        if (fraction < min_fraction) {
            RCLCPP_WARN(this->get_logger(), "Cartesian path fraction too low: %.3f (need %.3f)", fraction, min_fraction);
            return false;
        }
        return executeTimedTrajectory(
            arm_group_, arm_group_name_, trajectory, velocity_scaling, velocity_scaling, true);
    }

    bool failTightening(const std::string &reason)
    {
        disarmTorqueMonitor();
        RCLCPP_ERROR(this->get_logger(), "Tightening failed: %s", reason.c_str());
        publishLauncherStatus("Tightening failed", reason);
        (void)moveGripper(gripper_open_cmd_);  // never leave the gripper clamped on the nut
        return false;
    }

    // Tighten a nut that is on the bolt. start_nut_bottom_z / nut_yaw are the robot's own
    // estimate of where the nut is (base frame; yaw = hexagon corner axis); max_turns is the
    // safety stop.
    bool runTighteningCycle(double start_nut_bottom_z, double nut_yaw, double max_turns)
    {
        // The sensor may start a few seconds after this node; wait for it before moving.
        publishLauncherStatus("Tightening: waiting for wrist F/T", wrist_ft_topic_);
        const auto wait_start = std::chrono::steady_clock::now();
        while (!wrist_wrench_received_.load() &&
               std::chrono::steady_clock::now() - wait_start < std::chrono::seconds(10)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!wrist_wrench_received_.load()) {
            return failTightening("no data on " + wrist_ft_topic_ + " (wrist force-torque sensor) after 10 s");
        }

        const double stroke = tighten_stroke_deg_ * PI / 180.0;
        const double grip_yaw = nut_yaw + grasp_yaw_offset_;  // jaws on two flats
        // The fingertips must stay at least 1 mm above the plate.
        const double min_tcp_z = tighten_seat_z_ + 0.001 + jaw_tip_offset_;
        double nut_bottom_z = start_nut_bottom_z;  // tracked as the nut runs down
        double turned = 0.0;                          // total nut rotation [rad]
        double run_peak_torque = 0.0;
        int strokes = 0;
        const auto start_time = std::chrono::steady_clock::now();

        // --- Approach: ready pose, then above the nut with the jaws aligned to two flats.
        publishLauncherStatus("Tightening: approach", "Moving above the nut on the bolt");
        if (!moveArmToJointTarget(ready_joints_)) {
            return failTightening("move to ready pose failed");
        }
        const auto hover = buildTopDownPose(
            tighten_bolt_x_, tighten_bolt_y_, nut_bottom_z + tighten_nut_height_ + pregrasp_height_, grip_yaw);
        if (!planAndExecutePose(hover)) {
            return failTightening("move above the nut failed");
        }

        while (true) {
            if (turned / (2.0 * PI) >= max_turns) {
                return failTightening(format("%.2f turns without reaching %.2f N*m (max %.1f turns)",
                                             turned / (2.0 * PI), tighten_snug_torque_, max_turns));
            }

            // --- 1. Open, then put the fingertips just above the nut's bottom face.
            if (!moveGripper(gripper_open_cmd_)) {
                return failTightening("opening the gripper failed");
            }
            const double grasp_z = std::max(nut_bottom_z + tighten_finger_clearance_ + jaw_tip_offset_, min_tcp_z);
            if (!moveCartesianPose(buildTopDownPose(tighten_bolt_x_, tighten_bolt_y_, grasp_z, grip_yaw), 0.90, false)) {
                return failTightening("descent to the nut failed");
            }

            // --- 2. Grip two flats.
            if (!closeGripperForGrasp()) {
                return failTightening("could not grip the nut");
            }

            // --- 3. Turn one stroke clockwise, following the thread down, watching the torque.
            ++strokes;
            publishLauncherStatus("Tightening: stroke " + std::to_string(strokes),
                                  format("%.2f turns so far", turned / (2.0 * PI)));
            const auto stroke_start = arm_group_->getCurrentPose(flange_link_name_).pose;
            const double start_yaw = toolYaw(stroke_start);
            auto waypoints = helixWaypoints(stroke_start, start_yaw, -stroke, true);
            for (auto &waypoint : waypoints) {
                waypoint.position.z = std::max(waypoint.position.z, min_tcp_z);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(150));  // let the grip settle
            armTorqueMonitor();
            const bool stroke_ok = executeCartesianWaypoints(waypoints, tighten_velocity_scaling_);
            disarmTorqueMonitor();
            const bool snug = snug_detected_.load();
            run_peak_torque = std::max(run_peak_torque, peak_net_torque_.load());

            // How far did the tool (and so the nut) actually turn? A snug stop ends early.
            (void)waitForRobotState(0.5);
            const double end_yaw = toolYaw(arm_group_->getCurrentPose(flange_link_name_).pose);
            const double rotated = std::clamp(wrapAngle(start_yaw - end_yaw), 0.0, stroke);
            turned += rotated;
            nut_bottom_z = std::max(tighten_seat_z_, nut_bottom_z - thread_pitch_ * rotated / (2.0 * PI));
            RCLCPP_INFO(this->get_logger(),
                        "Stroke %d: turned %.0f deg, total %.2f turns, nut bottom ~%.4f m, peak torque %.2f N*m%s",
                        strokes, rotated * 180.0 / PI, turned / (2.0 * PI), nut_bottom_z,
                        peak_net_torque_.load(), snug ? " -> SNUG" : "");
            if (snug) {
                break;
            }
            if (!stroke_ok) {
                return failTightening("turning stroke failed before the snug torque was reached");
            }

            // --- 4. Let go and turn the open jaws back to the grip yaw (no height change).
            //        The nut turned a multiple of 60 deg, so the jaws meet two flats again.
            if (!moveGripper(gripper_open_cmd_)) {
                return failTightening("releasing the nut after a stroke failed");
            }
            const auto release_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
            if (!executeCartesianWaypoints(helixWaypoints(release_pose, toolYaw(release_pose), stroke, false),
                                           arm_velocity_scaling_)) {
                return failTightening("turning the open jaws back failed");
            }
        }

        // --- Snug: report, let go and retreat.
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        const std::string summary = format("%.2f turns in %d strokes, peak torque %.2f N*m, %.0f s",
                                           turned / (2.0 * PI), strokes, run_peak_torque, elapsed);
        RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Nut is snug: %s" COLOR_RESET, summary.c_str());
        publishLauncherStatus("Tightening: snug", summary);
        if (!moveGripper(gripper_open_cmd_)) {
            return failTightening("releasing the snug nut failed");
        }
        const auto final_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
        (void)moveCartesian(final_pose.position.x, final_pose.position.y, final_pose.position.z + lift_height_, 0.80);
        (void)moveArmToJointTarget(ready_joints_);
        publishLauncherStatus("Tightening complete", summary);
        return true;
    }

    // Call a std_srvs/Trigger service and wait for the answer (the response is handled on
    // service_callback_group_ while this thread waits). Returns success; fills `message`.
    bool callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client, std::string &message,
                     double timeout_sec = 30.0)
    {
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            message = std::string("service ") + client->get_service_name() + " is not available";
            return false;
        }
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (future.wait_for(std::chrono::duration<double>(timeout_sec)) != std::future_status::ready) {
            message = std::string("no answer from ") + client->get_service_name();
            return false;
        }
        const auto response = future.get();
        message = response->message;
        return response->success;
    }

    // task_mode = "tighten": a nut is threaded onto the bolt 3 turns above the seat
    // (simulation stand-in for "a nut is already on the bolt"), then tightened.
    bool tightenPreloadedNut()
    {
        publishLauncherStatus("Tightening: preparing bolt", "Threading a nut onto the bolt fixture (simulation)");
        std::string message;
        if (!callTrigger(preload_nut_client_, message)) {
            return failTightening("could not preload a nut on the bolt: " + message);
        }
        RCLCPP_INFO(this->get_logger(), "Bolt fixture: %s", message.c_str());
        return runTighteningCycle(tighten_nut_bottom_z_, tighten_nut_yaw_, tighten_max_turns_);
    }

    // task_mode = "pick_tighten", called after the pick sequence lifted the nut:
    // carry it over the bolt, set it on the bolt tip, let go, let the thread "catch"
    // (bolt_fixture_manager swaps in a threaded nut), then tighten it.
    bool placeNutOnBoltAndTighten(double picked_nut_top_z)
    {
        // Where is the nut relative to the TCP? At the grasp, the TCP was at validated_pick_pose_
        // and the nut's bottom face at (detected top - nut height).
        const double tcp_above_nut_bottom =
            validated_pick_pose_.position.z - (picked_nut_top_z - tighten_nut_height_);
        const double place_yaw = toolYaw(arm_group_->getCurrentPose(flange_link_name_).pose);

        // --- 1. Above the bolt, keeping the jaw orientation the nut was picked with.
        publishLauncherStatus("Placing nut on bolt", "Moving above the bolt fixture");
        RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Place on bolt: tcp is %.4f m above the nut bottom" COLOR_RESET,
                    tcp_above_nut_bottom);
        const auto hover = buildTopDownPose(
            tighten_bolt_x_, tighten_bolt_y_, tighten_bolt_tip_z_ + tcp_above_nut_bottom + pregrasp_height_, place_yaw);
        if (!planAndExecutePose(hover)) {
            return failTightening("move above the bolt failed");
        }

        // --- 2. Lower the nut until its bottom face is 1 mm above the bolt tip.
        const auto place = buildTopDownPose(
            tighten_bolt_x_, tighten_bolt_y_, tighten_bolt_tip_z_ + 0.001 + tcp_above_nut_bottom, place_yaw);
        if (!moveCartesianPose(place, 0.90, false)) {
            return failTightening("lowering the nut onto the bolt failed");
        }

        // --- 3. Let go (the nut settles on the tip) and move clear.
        const double placed_yaw = toolYaw(arm_group_->getCurrentPose(flange_link_name_).pose);
        if (!moveGripper(gripper_open_cmd_)) {
            return failTightening("releasing the nut on the bolt failed");
        }
        const auto released = arm_group_->getCurrentPose(flange_link_name_).pose;
        (void)moveCartesian(released.position.x, released.position.y, released.position.z + 0.03, 0.80);

        // --- 4. The thread catches (simulation: swap in a nut on a screw joint).
        publishLauncherStatus("Placing nut on bolt", "Engaging the thread");
        std::string message;
        if (!callTrigger(engage_nut_client_, message)) {
            return failTightening("the nut did not engage the thread: " + message);
        }
        RCLCPP_INFO(this->get_logger(), "Bolt fixture: %s", message.c_str());

        // --- 5. Tighten from the robot's own estimate: nut bottom just below the tip, and
        //        nut corner axis = jaw yaw at release - grasp_yaw_offset.
        const double nut_bottom_z = tighten_bolt_tip_z_ - 0.001;
        const double nut_yaw = placed_yaw - grasp_yaw_offset_;
        const double turns_to_seat = (nut_bottom_z - tighten_seat_z_) / thread_pitch_;
        RCLCPP_INFO(this->get_logger(), "Nut on the bolt: %.2f turns above the seat, yaw %.2f rad", turns_to_seat, nut_yaw);
        return runTighteningCycle(nut_bottom_z, nut_yaw, turns_to_seat + 1.0);
    }

    void mainLoop()
    {
        std::unique_lock<std::mutex> loop_lock(main_loop_mutex_, std::try_to_lock);
        if (!loop_lock.owns_lock()) {
            return;  // previous tick is still executing a motion sequence
        }

        if (!arm_group_ || !gripper_group_ || is_busy_) {
            if (!arm_group_ || !gripper_group_) {
                if (!move_group_wait_logged_) {
                    publishLauncherStatus("Waiting for MoveGroup", "MoveGroup initialization pending");
                    RCLCPP_INFO(this->get_logger(), "Waiting for MoveGroupInterface initialization...");
                    move_group_wait_logged_ = true;
                }
            }
            return;
        }

        move_group_wait_logged_ = false;

        // Sim-mode interlock: simulation motion settings may only drive a simulator.
        // Gazebo (ros_gz_bridge) publishes /clock; the JAKA hardware driver does not.
        if (!is_real_robot_ && this->count_publishers("/clock") == 0) {
            if (!sim_clock_block_logged_) {
                publishLauncherStatus("Blocked: no simulator clock", "robot_mode=sim but /clock has no publisher");
                RCLCPP_ERROR(this->get_logger(),
                             "robot_mode=sim but no publisher on /clock. Motion is blocked until the "
                             "Gazebo simulation is running.");
                sim_clock_block_logged_ = true;
            }
            return;
        }
        sim_clock_block_logged_ = false;

        // Tighten-only task: one run per node start; the pick-and-place path below is unused.
        if (task_mode_ == "tighten") {
            if (!tighten_finished_) {
                is_busy_ = true;
                tightenPreloadedNut();
                tighten_finished_ = true;
                is_busy_ = false;
            }
            return;
        }
        // Pick + tighten: uses the normal pick sequence (0-4), then sequence 5 places the nut
        // on the bolt and tightens it. One nut per node start (there is one bolt).
        if (task_mode_ == "pick_tighten" && tighten_finished_) {
            return;
        }

        if (bin_index_ >= bin_locations_.size()) {
            publishLauncherStatus("Completed", "All configured place slots finished");
            return;
        }

        if (!nut_pose_received_) {
            publishLauncherStatus("Waiting for nut pose", "Perception has not delivered a pick pose yet");
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
                publishLauncherStatus("Sequence 0: Hover and align");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 0: Hover and align" COLOR_RESET);
                if (!moveArmToJointTarget(ready_joints_)) {
                    requestNewDetection("Failed to move to ready seed");
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(150));

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
                publishLauncherStatus("Sequence 1: Open gripper");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 1: Open gripper" COLOR_RESET);
                if (moveGripper(gripper_open_cmd_)) {
                    sequence_ = 2;
                } else {
                    requestNewDetection("Open gripper failed");
                }
                break;
            }

            case 2: {
                publishLauncherStatus("Sequence 2: Descend to pick pose");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 2: Descend to pick pose" COLOR_RESET);
                if (!ensureGripperOpenForPick()) {
                    requestNewDetection("Gripper failed to reach open state before pick descent");
                    break;
                }

                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                const double pick_tcp_z = computePickTcpZ(static_cast<double>(local_nut[2]));
                const double required_descent = current_pose.position.z - pick_tcp_z;
                if (required_descent < -0.002) {
                    requestNewDetection("Current TCP pose is already below the computed pick height");
                    break;
                }
                if (required_descent > max_pick_descent_) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Refusing pick descent of %.4f m; configured max_pick_descent is %.4f m.",
                        required_descent,
                        max_pick_descent_);
                    requestNewDetection("Pick descent exceeds configured safety limit");
                    break;
                }

                auto xy_align_pose = current_pose;
                xy_align_pose.position.x = local_nut[0];
                xy_align_pose.position.y = local_nut[1];
                xy_align_pose.orientation = buildTopDownPose(0.0, 0.0, 0.0, local_nut[3]).orientation;
                if (!moveToPoseWithFallback(xy_align_pose, 0.90, true)) {
                    requestNewDetection("Pick XY align failed");
                    break;
                }

                const auto aligned_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                const double approach_tcp_z = std::max(
                    pick_tcp_z + pick_approach_offset_,
                    table_top_z_ + pick_clearance_z_ + pick_safe_gap_z_ + pick_approach_offset_);
                if (aligned_pose.position.z > approach_tcp_z + 1e-3) {
                    auto approach_pose = buildTopDownPose(local_nut[0], local_nut[1], approach_tcp_z, local_nut[3]);
                    if (!moveVerticalToPoseInSteps(
                            approach_pose,
                            std::max(0.005, grasp_descent_step_ * 3.0),
                            0.85,
                            true,
                            false)) {
                        requestNewDetection("Pick approach descent failed");
                        break;
                    }
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Pick descent using link '%s': nut_xyz=(%.4f, %.4f, %.4f) target_tcp_z=%.4f current_tcp_z=%.4f",
                    flange_link_name_.c_str(),
                    local_nut[0],
                    local_nut[1],
                    local_nut[2],
                    pick_tcp_z,
                    arm_group_->getCurrentPose(flange_link_name_).pose.position.z);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Seq 2 Z safety settings: estimated_table_z=%.4f pick_safe_gap_z=%.4f "
                    "detected_nut_height=%.4f min_grasp_tcp_above_table=%.4f table_top_z=%.4f",
                    estimateTableSurfaceZ(static_cast<double>(local_nut[2])),
                    pick_safe_gap_z_,
                    detected_nut_height_,
                    min_grasp_tcp_above_table_,
                    table_top_z_);

                auto pick_pose = buildTopDownPose(local_nut[0], local_nut[1], pick_tcp_z, local_nut[3]);
                if (!moveVerticalToPoseInSteps(pick_pose, grasp_descent_step_, 0.80, false, false)) {
                    requestNewDetection("Pick descent failed");
                    break;
                }

                if (!waitUntilPoseNearTarget(pick_pose, pick_xy_tolerance_, pick_z_tolerance_)) {
                    requestNewDetection("Pick pose validation failed after descent");
                    break;
                }

                validated_pick_pose_ = pick_pose;
                validated_pick_pose_valid_ = true;
                sequence_ = 3;
                break;
            }

            case 3: {
                publishLauncherStatus("Sequence 3: Close gripper");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 3: Close gripper" COLOR_RESET);
                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                if (!validated_pick_pose_valid_) {
                    requestNewDetection("Validated pick pose is unavailable before close");
                    break;
                }
                if (!waitUntilPoseNearTarget(validated_pick_pose_, pick_xy_tolerance_, pick_z_tolerance_)) {
                    requestNewDetection("Robot drifted away from validated pick pose before close");
                    break;
                }
                if (current_pose.position.z < computeMinimumSafeTcpZ(static_cast<double>(local_nut[2])) - 0.002) {
                    requestNewDetection("Refusing to close gripper because TCP dropped below safe table clearance");
                    break;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Closing gripper at link '%s' pose: x=%.4f y=%.4f z=%.4f",
                    flange_link_name_.c_str(),
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z);

                if (!closeGripperForGrasp()) {
                    requestNewDetection("Close gripper failed");
                    break;
                }

                finalizeDetectToPickTimer();
                sequence_ = 4;
                break;
            }

            case 4: {
                publishLauncherStatus("Sequence 4: Lift");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 4: Lift" COLOR_RESET);
                const auto current_pose = arm_group_->getCurrentPose(flange_link_name_).pose;
                if (isGripperNearCommand(gripper_open_cmd_)) {
                    requestNewDetection("Refusing to lift because gripper appears open");
                    break;
                }
                if (!validated_pick_pose_valid_) {
                    requestNewDetection("Validated pick pose is unavailable before lift");
                    break;
                }
                if (!isCurrentPoseNearTarget(validated_pick_pose_, pick_xy_tolerance_ * 1.5, pick_z_tolerance_ * 1.5)) {
                    requestNewDetection("Robot is not at a safe post-grasp pose before lift");
                    break;
                }

                auto lift_pose = current_pose;
                lift_pose.position.z = std::max(
                    current_pose.position.z + lift_height_,
                    table_top_z_ + pick_clearance_z_ + 0.05);
                if (moveVerticalToPoseInSteps(
                        lift_pose,
                        std::max(0.004, grasp_descent_step_ * 2.0),
                        0.85,
                        true,
                        false)) {
                    sequence_ = 5;
                } else {
                    requestNewDetection("Lift failed");
                }
                break;
            }

            case 5: {
                if (task_mode_ == "pick_tighten") {
                    // The nut is held and lifted: put it on the bolt and tighten it.
                    placeNutOnBoltAndTighten(static_cast<double>(local_nut[2]));
                    tighten_finished_ = true;
                    sequence_ = 0;
                    break;
                }
                publishLauncherStatus("Sequence 5: Move to place hover");
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
                publishLauncherStatus("Sequence 6: Descend to place");
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
                publishLauncherStatus("Sequence 7: Release");
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "Sequence 7: Release" COLOR_RESET);
                if (!moveGripper(gripper_open_cmd_)) {
                    requestNewDetection("Release gripper failed");
                    break;
                }
                sequence_ = 8;
                break;
            }

            case 8: {
                publishLauncherStatus("Sequence 8: Retreat and reset");
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
