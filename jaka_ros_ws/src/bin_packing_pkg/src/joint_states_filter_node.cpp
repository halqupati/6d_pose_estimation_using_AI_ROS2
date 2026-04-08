#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatesFilterNode : public rclcpp::Node
{
public:
  JointStatesFilterNode()
  : Node("joint_states_filter_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(50),
      std::bind(&JointStatesFilterNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_filtered", rclcpp::QoS(50));
  }

private:
  static bool ends_with(const std::string & value, const std::string & suffix)
  {
    if (value.size() < suffix.size()) {
      return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
  }

  void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto out = sensor_msgs::msg::JointState();
    out.header = msg->header;
    out.name.reserve(msg->name.size());
    out.position.reserve(msg->position.size());
    out.velocity.reserve(msg->velocity.size());
    out.effort.reserve(msg->effort.size());

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      if (ends_with(name, "_mimic")) {
        continue;
      }

      out.name.push_back(name);

      if (i < msg->position.size()) {
        out.position.push_back(msg->position[i]);
      }
      if (i < msg->velocity.size()) {
        out.velocity.push_back(msg->velocity[i]);
      }
      if (i < msg->effort.size()) {
        out.effort.push_back(msg->effort[i]);
      }
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatesFilterNode>());
  rclcpp::shutdown();
  return 0;
}

