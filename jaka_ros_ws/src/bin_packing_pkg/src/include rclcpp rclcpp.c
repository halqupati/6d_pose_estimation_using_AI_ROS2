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
    sensor_msgs::msg::JointState current_joint_state_;
    std_msgs::msg::Bool msg;

    int sequence = 0;
    bool planning = false;
    bool nut_pose_received_ = false;
    std::array<float, 4> nut_pose_angle_;
    bool nut_pose_angle_received_ = false;
    double roll_current, pitch_current, yaw_current;
    size_t i = 0;

    // coordinates to place nuts. x, y, z
    std::array<std::array<double, 3>, 3> nut_places = {{
        {0.3, -0.24, 0.05},
        {0.3, -0.30, 0.05},
        {0.3, -0.36, 0.05}
    }};

    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    ControlRobot()
        : Node("robot_controller")
    {
        // ---- CREATE CALLBACK GROUPS ----
        pose_angle_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant
        );

        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant
        );

        // ---- Pose angle SUBSCRIPTION ----
        rclcpp::SubscriptionOptions pose_angle_options;
        pose_angle_options.callback_group = pose_angle_group_;

        pos_angle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pos_angle", 10,
            std::bind(&ControlRobot::pose_angleCallback, this, std::placeholders::_1),
            pose_angle_options
        );    

        // Trigger publisher
        bool_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/trigger",
            10
        );

        // ---- TIMER ----
        rclcpp::TimerBase::SharedPtr timer =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ControlRobot::timerCallback, this),
                timer_group_
            );
        timer_ = timer;

        // ---- For collision mesh ----
        planning_scene_publisher_ =
            this->create_publisher<moveit_msgs::msg::PlanningScene>(
                "/planning_scene",
                rclcpp::QoS(1).transient_local()
            );

        // Give MoveIt time to start listening
        timer_collision_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ControlRobot::addCollisionMesh, this)
        );

        RCLCPP_INFO(this->get_logger(), "Jaka_zu5 Control node created. Waiting for MoveGroup init...");
    }

    bool waitForRobotState(double timeout_sec = 2.0)
    {
        auto start = this->now();
        RCLCPP_INFO(this->get_logger(), "Waiting for robot state1...");
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);

        while ((this->now() - start) < timeout)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for robot state2...");
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

        RCLCPP_INFO(this->get_logger(), "Waiting for robot state...");

        if (!waitForRobotState(2.0))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive robot state");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Robot state ready");
        }
    }

private:

    void apply_sleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }
    
    void addCollisionMesh()
        {
           moveit::planning_interface::PlanningSceneInterface psi;

            // camera
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = "DaBai_DCW2_Link";

            shapes::Mesh* mesh = shapes::createMeshFromResource(
                "package://bin_packing_pkg/meshes/DaBai_DCW2_Link.stl"
            );

            shape_msgs::msg::Mesh mesh_msg;
            shapes::ShapeMsg mesh_msg_tmp;
            shapes::constructMsgFromShape(mesh, mesh_msg_tmp);
            mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);

            collision_object.meshes.push_back(mesh_msg);
            
            
            geometry_msgs::msg::Pose mesh_pose;
            mesh_pose.position.x = 0.3;
            mesh_pose.position.y = -0.3;
            mesh_pose.position.z = 0.0;
            //mesh_pose.orientation.w = 1.0;
            // Rotate -90° about Y axis
            tf2::Quaternion q;
            q.setRPY(0.0, -M_PI_2, 0.0);   // roll, pitch, yaw

            mesh_pose.orientation = tf2::toMsg(q);

            collision_object.mesh_poses.push_back(mesh_pose);
            collision_object.operation = collision_object.ADD;

            psi.applyCollisionObject(collision_object);
            
      
          // table
          moveit_msgs::msg::CollisionObject table_obj;
          table_obj.header.frame_id = "world";
          table_obj.id = "simulation_table";

          shape_msgs::msg::SolidPrimitive table_primitive;
          table_primitive.type = table_primitive.BOX;
          table_primitive.dimensions.resize(3);
          table_primitive.dimensions[0] = 1.5;  // X (Length)
          table_primitive.dimensions[1] = 0.8;  // Y (Width)
          table_primitive.dimensions[2] = 1.03; // Z (Total Height: legs + top)

          geometry_msgs::msg::Pose table_pose;
          // Ensure these X and Y match your Gazebo World spawn position for the table!
          table_pose.position.x = 0.31; 
          table_pose.position.y = 0.32; 
          table_pose.position.z = 0.515; // Calculated Center (1.03 / 2)
          table_pose.orientation.w = 1.0;

          table_obj.primitives.push_back(table_primitive);
          table_obj.primitive_poses.push_back(table_pose);
          table_obj.operation = table_obj.ADD;

          psi.applyCollisionObject(table_obj);


          // Stop the timer so it only publishes once (Publish ONCE)
          timer_collision_->cancel();
            
        }

    // -------------------- CALLBACKS --------------------
    void pose_angleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_angle_mutex);
        std::copy_n(msg->data.begin(), 4, nut_pose_angle_.begin());
        /*
        float x = nut_pose_angle[0];
        float y = nut_pose_angle[1];
        float z = nut_pose_angle[2];
        float angle = nut_pose_angle[3];
        */
        nut_pose_angle_received_ = true;
        RCLCPP_INFO(this->get_logger(),  "we get nut position");
    }

    void timerCallback()
    {
        if (!arm_move_group)
        {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        if (!nut_pose_angle_received_ && i < nut_places.size())
        {
            msg.data = true;
            bool_pub->publish(msg);
            RCLCPP_WARN(this->get_logger(), COLOR_YELLOW "Nut pose is not received yet." COLOR_RESET);
            return;
        }

            if (sequence == 0 && planning == false) //Moving to the nut position
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 0: Moving to the nut position" COLOR_RESET);

                arm_move_group->startStateMonitor();
                // Sometimes AAE pose is slow to update. So, we wait.
                rclcpp::sleep_for(std::chrono::milliseconds(3000));

                // --- Copy nut pose safely ---
                std::array<float, 4> local_nut_pose_angle;
                {
                    std::lock_guard<std::mutex> lock(pose_angle_mutex);
                    if (!nut_pose_angle_received_) return;
                    local_nut_pose_angle = nut_pose_angle_;
                }

                // we do not need publish nut pose for now
                msg.data = false;
                bool_pub->publish(msg);

                arm_move_group->setStartStateToCurrentState();
                current_pose = arm_move_group->getCurrentPose("Link_6");

                target_pose.position.x = local_nut_pose_angle[0];
                target_pose.position.y = local_nut_pose_angle[1];
                target_pose.position.z = current_pose.pose.position.z;
                target_pose.orientation = current_pose.pose.orientation;
            
                RCLCPP_INFO(this->get_logger(), "EE Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                RCLCPP_INFO(this->get_logger(), "EE Target Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);

                std::vector<geometry_msgs::msg::Pose> waypoints;

                geometry_msgs::msg::Pose start = arm_move_group->getCurrentPose().pose;
                waypoints.push_back(start);

                geometry_msgs::msg::Pose target = start;
                target.position.z = target_pose.position.z;
                waypoints.push_back(target);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = arm_move_group->computeCartesianPath(
                    waypoints,
                    0.002,   // eef_step (meters)
                    0.0,     // jump_threshold
                    trajectory
                );

                if (fraction > 0.99)
                {
                    robot_trajectory::RobotTrajectory rt(
                        arm_move_group->getRobotModel(),
                        arm_move_group->getName());

                    rt.setRobotTrajectoryMsg(*arm_move_group->getCurrentState(), trajectory);

                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(
                        rt,
                        0.6,   // velocity scaling factor (20%)
                        0.3    // acceleration scaling factor (10%)
                    );

                    rt.getRobotTrajectoryMsg(trajectory);
                    arm_move_group->execute(trajectory);
                }

                sequence = 1;
                planning = false;
            }
            else if (sequence == 1 && planning == false) //Rotating the gripper
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 1: Rotating gripper" COLOR_RESET);

                current_pose = arm_move_group->getCurrentPose("Link_6");
                
                q_current = current_pose.pose.orientation;
                tf2::Quaternion tf_quat_current(q_current.x, q_current.y, q_current.z, q_current.w);
                tf2::Matrix3x3(tf_quat_current).getRPY(roll_current, pitch_current, yaw_current);
                //RCLCPP_INFO(this->get_logger(), "Current EE Roll: %f, Pitch: %f, Yaw: %f", roll_current, pitch_current, yaw_current);
                
                std::array<float, 4> local_nut_pose_angle;
                {
                    std::lock_guard<std::mutex> lock(pose_angle_mutex);
                    if (!nut_pose_angle_received_) return;
                    local_nut_pose_angle = nut_pose_angle_;
                }

                tf2::Quaternion tf_quat;
                tf_quat.setRPY(roll_current, pitch_current, local_nut_pose_angle[3]);
                RCLCPP_INFO(this->get_logger(), "EE Yaw: %f, nut theta: %f", yaw_current, local_nut_pose_angle[3]);

                target_pose.position.x = local_nut_pose_angle[0];
                target_pose.position.y = local_nut_pose_angle[1];
                target_pose.position.z = current_pose.pose.position.z;
                target_pose.orientation.x = tf_quat.x();
                target_pose.orientation.y = tf_quat.y();
                target_pose.orientation.z = tf_quat.z();
                target_pose.orientation.w = tf_quat.w();

                pose_target_plan_and_execute(target_pose);

                sequence = 2;
                planning = false;

            }
            else if (sequence == 2 && planning == false) //Opening the gripper
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 2: Opening the gripper" COLOR_RESET);

                joint_value_target_plan_and_execute(0.015f);
                sequence = 3;
                planning = false;
            }
            else if (sequence == 3 && planning == false) //Moving down
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 3: Moving down." COLOR_RESET);

                std::array<float, 4> local_nut_pose_angle;
                {
                    std::lock_guard<std::mutex> lock(pose_angle_mutex);
                    if (!nut_pose_angle_received_) return;
                    local_nut_pose_angle = nut_pose_angle_;
                }

                target_pose.position.x = current_pose.pose.position.x;
                target_pose.position.y = current_pose.pose.position.y;
                target_pose.position.z = local_nut_pose_angle[2];
                target_pose.orientation = current_pose.pose.orientation;
            
                RCLCPP_INFO(this->get_logger(), "EE Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                RCLCPP_INFO(this->get_logger(), "EE Target Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);

                std::vector<geometry_msgs::msg::Pose> waypoints;

                geometry_msgs::msg::Pose start = arm_move_group->getCurrentPose().pose;
                waypoints.push_back(start);

                geometry_msgs::msg::Pose target = start;
                target.position.z = target_pose.position.z;
                waypoints.push_back(target);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = arm_move_group->computeCartesianPath(
                    waypoints,
                    0.002,   // eef_step (meters)
                    0.0,     // jump_threshold
                    trajectory
                );

                if (fraction > 0.99)
                {
                    robot_trajectory::RobotTrajectory rt(
                        arm_move_group->getRobotModel(),
                        arm_move_group->getName());

                    rt.setRobotTrajectoryMsg(*arm_move_group->getCurrentState(), trajectory);

                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(
                        rt,
                        0.6,   // velocity scaling factor (20%)
                        0.3    // acceleration scaling factor (10%)
                    );

                    rt.getRobotTrajectoryMsg(trajectory);
                    arm_move_group->execute(trajectory);
                }

                sequence = 4;
                planning = false;
            }
            else if (sequence == 4 && planning == false) //Closing the gripper
            {
                planning = true;
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 4: Closing the gripper." COLOR_RESET);
                apply_sleep(0.5);
                joint_value_target_plan_and_execute(0.003f);
                apply_sleep(0.5);
                sequence = 5;
                planning = false;
            }
            else if (sequence == 5 && planning == false) //Moving up
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 5: Moving up." COLOR_RESET);
               
                arm_move_group->setStartStateToCurrentState();
                current_pose = arm_move_group->getCurrentPose("Link_6");

                target_pose.position.x = current_pose.pose.position.x;
                target_pose.position.y = current_pose.pose.position.y;
                target_pose.position.z = current_pose.pose.position.z + 0.1;
                target_pose.orientation = current_pose.pose.orientation;
            
                RCLCPP_INFO(this->get_logger(), "EE Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                RCLCPP_INFO(this->get_logger(), "EE Target Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);

                std::vector<geometry_msgs::msg::Pose> waypoints;

                geometry_msgs::msg::Pose start = arm_move_group->getCurrentPose().pose;
                waypoints.push_back(start);

                geometry_msgs::msg::Pose target = start;
                target.position.z = target_pose.position.z;
                waypoints.push_back(target);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = arm_move_group->computeCartesianPath(
                    waypoints,
                    0.002,   // eef_step (meters)
                    0.0,     // jump_threshold
                    trajectory
                );

                if (fraction > 0.99)
                {
                    robot_trajectory::RobotTrajectory rt(
                        arm_move_group->getRobotModel(),
                        arm_move_group->getName());

                    rt.setRobotTrajectoryMsg(*arm_move_group->getCurrentState(), trajectory);

                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(
                        rt,
                        0.6,   // velocity scaling factor (20%)
                        0.3    // acceleration scaling factor (10%)
                    );

                    rt.getRobotTrajectoryMsg(trajectory);
                    arm_move_group->execute(trajectory);
                }

                sequence = 6;
                planning = false;
            }
            else if (sequence == 6 && planning == false) //Moving to nut placement position and rotate gripper
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 6: Moving to the nut placement position." COLOR_RESET);

                arm_move_group->setStartStateToCurrentState();
                current_pose = arm_move_group->getCurrentPose("Link_6");

                q_current = current_pose.pose.orientation;
                tf2::Quaternion tf_quat_current(q_current.x, q_current.y, q_current.z, q_current.w);
                tf2::Matrix3x3(tf_quat_current).getRPY(roll_current, pitch_current, yaw_current);       
                tf2::Quaternion tf_quat;
                tf_quat.setRPY(roll_current, pitch_current, -PI/4);

                target_pose.position.x = nut_places[i][0];
                target_pose.position.y = nut_places[i][1];
                target_pose.position.z = current_pose.pose.position.z;
                target_pose.orientation.x = tf_quat.x();
                target_pose.orientation.y = tf_quat.y();
                target_pose.orientation.z = tf_quat.z();
                target_pose.orientation.w = tf_quat.w();
            
                RCLCPP_INFO(this->get_logger(), "EE Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                RCLCPP_INFO(this->get_logger(), "EE Target Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);
                
                pose_target_plan_and_execute(target_pose);
                
                sequence = 7;
                planning = false;
            }
            else if (sequence == 7 && planning == false) //Moving down
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 7: Moving down." COLOR_RESET);

                target_pose.position.x = current_pose.pose.position.x;
                target_pose.position.y = current_pose.pose.position.y;
                target_pose.position.z = nut_places[i][2];
                target_pose.orientation = current_pose.pose.orientation;
            
                RCLCPP_INFO(this->get_logger(), "EE Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        current_pose.pose.orientation.x,
                        current_pose.pose.orientation.y,
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);

                RCLCPP_INFO(this->get_logger(), "EE Target Pose:");
                RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);
                RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                        target_pose.orientation.x,
                        target_pose.orientation.y,
                        target_pose.orientation.z,
                        target_pose.orientation.w);

                std::vector<geometry_msgs::msg::Pose> waypoints;

                geometry_msgs::msg::Pose start = arm_move_group->getCurrentPose().pose;
                waypoints.push_back(start);

                geometry_msgs::msg::Pose target = start;
                target.position.z = target_pose.position.z;
                waypoints.push_back(target);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = arm_move_group->computeCartesianPath(
                    waypoints,
                    0.002,   // eef_step (meters)
                    0.0,     // jump_threshold
                    trajectory
                );

                if (fraction > 0.99)
                {
                    robot_trajectory::RobotTrajectory rt(
                        arm_move_group->getRobotModel(),
                        arm_move_group->getName());

                    rt.setRobotTrajectoryMsg(*arm_move_group->getCurrentState(), trajectory);

                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(
                        rt,
                        0.3,   // velocity scaling factor (20%)
                        0.15    // acceleration scaling factor (10%)
                    );

                    rt.getRobotTrajectoryMsg(trajectory);
                    arm_move_group->execute(trajectory);
                }

                sequence = 8;
                planning = false;
            }
            else if (sequence == 8 && planning == false) //Opening the gripper.
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 8: Opening the gripper." COLOR_RESET);


                joint_value_target_plan_and_execute(0.01f);

                sequence = 9;
                planning = false;
            }
            else if (sequence == 9 && planning == false) //Moving to the ready position
            {
                planning = true;

                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 9: Moving to the ready position." COLOR_RESET);

                named_target_plan_and_execute("ready");
                joint_value_target_plan_and_execute(0.0f);

                sequence = 10;
                planning = false;
            }
            else if (sequence == 10 && planning == false)
            {
                planning = true;

                msg.data = true;
                bool_pub->publish(msg);
                RCLCPP_INFO(this->get_logger(), COLOR_GREEN "sequence 10: Sending signal to resume AAE pose inference." COLOR_RESET);
                i = i+1;
                apply_sleep(1.0);
                if(i < nut_places.size()){
                  sequence = 0;
                  planning = false;
                }
                else{
                  RCLCPP_INFO(this->get_logger(), COLOR_GREEN "The simulation is completed!" COLOR_RESET);
                }

                nut_pose_angle_received_ = false;
            }

    }

    // -------------------- MOTION FUNCTIONS --------------------
    void pose_target_plan_and_execute(const geometry_msgs::msg::Pose &target_pose)
    {
        arm_move_group->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (
            arm_move_group->plan(plan) ==
            moveit::core::MoveItErrorCode::SUCCESS
        );

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan OK, executing...");
            auto result = arm_move_group->execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
                RCLCPP_INFO(this->get_logger(), "Motion done.");
            else
                RCLCPP_WARN(this->get_logger(), "Motion failed.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed.");
        }
    }

    void named_target_plan_and_execute(const std::string &name)
    {
        bool ok = arm_move_group->setNamedTarget(name);
        if (!ok)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set named target '%s'", name.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Moving to '%s'...", name.c_str());
        auto result = arm_move_group->move();

        if (result)
            RCLCPP_INFO(this->get_logger(), "Reached target '%s'", name.c_str());
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to reach '%s'", name.c_str());
    }

    void joint_value_target_plan_and_execute(float displacement){
        // Get current joint values
        std::vector<double> joint_values = gripper_move_group->getCurrentJointValues();
        std::ostringstream joint_values_stream;
        for (const auto& value : joint_values) {
            joint_values_stream << value << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Current joint values: %s", joint_values_stream.str().c_str());

        // Set joint value
        int joint_index = 0; // We have to set only one joint, because right finger mimics left finger (index:0)
        joint_values[joint_index] = displacement;
        bool success = gripper_move_group->setJointValueTarget(joint_values);   
        if (success) {
            auto execution_success = gripper_move_group->move();
            if (execution_success) {
                RCLCPP_INFO(this->get_logger(), "Motion execution successful!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Motion execution failed.");
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Planning failed. Unable to find a valid plan.");
        }
    }

    // -------------------- MEMBER VARIABLES --------------------
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_collision_;

    rclcpp::CallbackGroup::SharedPtr pose_angle_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr joint_states_group_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;
    std::unordered_map<std::string, size_t> name_to_index_;
    std::vector<double> ordered_joint_positions_;
    std::mutex pose_angle_mutex;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ControlRobot>();
    node->initializeMoveGroup();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
