#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>

class CollisionMonitorNode : public rclcpp::Node
{
public:
  CollisionMonitorNode()
    : Node("collision_monitor_node")
  {
  }

  void initialize()
  {
    planning_scene_topic_ = this->declare_parameter<std::string>("planning_scene_topic", "/monitored_planning_scene");
    check_period_sec_ = this->declare_parameter<double>("check_period_sec", 0.20);

    collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_collision_alert", 10);

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        this->shared_from_this(), "robot_description", "collision_monitor");
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize PlanningSceneMonitor.");
      return;
    }

    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_);
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false);
    planning_scene_monitor_->setStateUpdateFrequency(20.0);

    const auto period = std::chrono::duration<double>(std::max(0.05, check_period_sec_));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&CollisionMonitorNode::checkCollision, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Collision monitor active. planning_scene_topic=%s check_period_sec=%.2f",
        planning_scene_topic_.c_str(),
        check_period_sec_);
  }

private:
  void logLauncherCollisionStatus(bool colliding, const std::string& detail)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[LAUNCHER_STATUS] collision=%d; detail=%s",
        colliding ? 1 : 0,
        detail.empty() ? "-" : detail.c_str());
  }

  void publishCollisionState(bool colliding)
  {
    std_msgs::msg::Bool msg;
    msg.data = colliding;
    collision_pub_->publish(msg);
  }

  std::string formatContacts(const collision_detection::CollisionResult::ContactMap& contacts) const
  {
    if (contacts.empty()) {
      return "contact pair unavailable";
    }

    std::ostringstream oss;
    int pair_count = 0;
    for (const auto& [pair, contact_points] : contacts) {
      if (pair_count > 0) {
        oss << "; ";
      }
      oss << pair.first << " <-> " << pair.second;
      if (!contact_points.empty()) {
        oss << " (" << contact_points.size() << " contact)";
      }
      ++pair_count;
      if (pair_count >= 3) {
        break;
      }
    }
    return oss.str();
  }

  void checkCollision()
  {
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
      return;
    }

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);

    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    request.contacts = true;
    request.max_contacts = 8;
    request.max_contacts_per_pair = 1;
    request.verbose = false;

    moveit::core::RobotState current_state = locked_scene->getCurrentState();
    current_state.update();
    locked_scene->checkCollision(request, result, current_state);

    publishCollisionState(result.collision);

    if (!collision_state_known_ || result.collision != last_collision_state_) {
      if (result.collision) {
        logLauncherCollisionStatus(true, "Collision detected in current robot state");
        RCLCPP_WARN(
            this->get_logger(),
            "COLLISION ALERT: current robot state is colliding. %s",
            formatContacts(result.contacts).c_str());
      } else {
        logLauncherCollisionStatus(false, "Collision monitor reports clear");
        RCLCPP_INFO(this->get_logger(), "Collision monitor: current robot state is clear.");
      }
      last_collision_state_ = result.collision;
      collision_state_known_ = true;
    }
  }

  std::string planning_scene_topic_;
  double check_period_sec_ = 0.20;
  bool last_collision_state_ = false;
  bool collision_state_known_ = false;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CollisionMonitorNode>();
  node->initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
