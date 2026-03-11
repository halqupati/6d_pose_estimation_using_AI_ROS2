#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

#define PI 3.14159265359
#define COLOR_GREEN   "\033[32m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RESET   "\033[0m"

class ControlRobot : public rclcpp::Node {
public:
    ControlRobot() : Node("bin_picking_node") {
        // Callback groups for multi-threaded execution
        auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group;

        // Subscribers & Publishers
        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle", 10, std::bind(&ControlRobot::poseCallback, this, std::placeholders::_1), sub_options);
        
        trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

        // Timer for the state machine
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
            std::bind(&ControlRobot::mainLoop, this), cb_group);

        RCLCPP_INFO(this->get_logger(), "Bin Picking Node Initialized.");
    }

    void initMoveIt() {
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "jaka_zu5");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        
        arm_group_->setPlanningTime(5.0);
        arm_group_->setMaxVelocityScalingFactor(0.2); // Slower is safer for debugging
        arm_group_->setMaxAccelerationScalingFactor(0.2);
    }

private:
    // Variables
    int sequence = 0;
    bool is_busy = false;
    bool nut_detected = false;
    std::array<float, 4> nut_data; // x, y, z, yaw
    std::mutex data_mutex;
    size_t bin_index = 0;

    std::array<std::array<double, 3>, 3> bin_locations = {{
        {0.4, -0.3, 0.1}, {0.4, -0.4, 0.1}, {0.4, -0.5, 0.1}
    }};

    // MoveIt Objects
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    void poseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        std::lock_guard<std::mutex> lock(data_mutex);
        std::copy_n(msg->data.begin(), 4, nut_data.begin());
        nut_detected = true;
    }

    void mainLoop() {
        if (!arm_group_ || is_busy) return;

        if (!nut_detected && bin_index < bin_locations.size()) {
            std_msgs::msg::Bool trigger; trigger.data = true;
            trigger_pub_->publish(trigger);
            return;
        }

        switch (sequence) {
            case 0: // HOVER & ALIGN
                is_busy = true;
                RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Sequence 0: Hovering..." COLOR_RESET);
                if (planAndExecute(nut_data[0], nut_data[1], nut_data[2] + 0.15, nut_data[3])) {
                    sequence = 1;
                }
                is_busy = false;
                break;

            case 1: // OPEN GRIPPER
                is_busy = true;
                moveGripper(0.0); // 0.0 = Open
                sequence = 2;
                is_busy = false;
                break;

            case 2: // DESCEND (Cartesian)
                is_busy = true;
                RCLCPP_INFO(this->get_logger(), COLOR_CYAN "Sequence 2: Descending..." COLOR_RESET);
                // 0.155 is the tool offset for your Robotiq gripper
                if (moveCartesian(nut_data[0], nut_data[1], nut_data[2] + 0.155)) {
                    sequence = 3;
                }
                is_busy = false;
                break;

            case 3: // GRASP
                is_busy = true;
                moveGripper(0.75); // 0.75 = Closed on Nut
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
                sequence = 4;
                is_busy = false;
                break;

            case 4: // LIFT
                is_busy = true;
                moveCartesian(nut_data[0], nut_data[1], nut_data[2] + 0.25);
                sequence = 5;
                is_busy = false;
                break;

            case 5: // GO TO BIN
                is_busy = true;
                if (planAndExecute(bin_locations[bin_index][0], bin_locations[bin_index][1], bin_locations[bin_index][2] + 0.1, 0.0)) {
                    sequence = 6;
                }
                is_busy = false;
                break;

            case 6: // RELEASE
                is_busy = true;
                moveGripper(0.0);
                bin_index++;
                nut_detected = false;
                sequence = 0;
                is_busy = false;
                break;
        }
    }

    // --- MOTION HELPERS WITH TIMING FIXES ---

    bool planAndExecute(double x, double y, double z, double yaw) {
        arm_group_->setStartStateToCurrentState();
        
        geometry_msgs::msg::Pose target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        // FIXED ORIENTATION: Gripper points straight down
        tf2::Quaternion q;
        q.setRPY(PI, 0, yaw); // Roll=PI flips it, Yaw matches AAE
        target.orientation = tf2::toMsg(q);

        arm_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        if (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // FIX: Smooth the trajectory timestamps to prevent Gazebo Rejection
            smoothTrajectory(plan);
            return (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool moveCartesian(double x, double y, double z) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose p = arm_group_->getCurrentPose().pose;
        p.position.x = x; p.position.y = y; p.position.z = z;
        waypoints.push_back(p);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        
        if (fraction > 0.9) {
            return (arm_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    void moveGripper(double position) {
        std::vector<double> joints = gripper_group_->getCurrentJointValues();
        if (!joints.empty()) {
            joints[0] = position;
            gripper_group_->setJointValueTarget(joints);
            gripper_group_->move();
        }
    }

    // This function fixes the "Time not strictly increasing" error
    void smoothTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
        robot_trajectory::RobotTrajectory rt(arm_group_->getRobotModel(), "jaka_zu5");
        rt.setRobotTrajectoryMsg(*arm_group_->getCurrentState(), plan.trajectory_);
        
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (totg.computeTimeStamps(rt, 0.2, 0.2)) {
            rt.getRobotTrajectoryMsg(plan.trajectory_);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlRobot>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });
    node->initMoveIt();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}