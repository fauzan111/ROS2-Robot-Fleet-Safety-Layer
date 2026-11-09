// Fail-safe velocity gate.
//
//   Nav2 --cmd_vel_in--> [speed_gate] --cmd_vel_out--> robot base
//                             ^
//         /safety/speed_scale |  (Float32 in [0, 1] from safety_monitor)
//
// Fail-safe rule: if no speed scale has arrived yet, or the last one is older than
// `safety_timeout_s`, the robot is stopped. A crashed or frozen safety monitor must
// never leave the robot moving at full speed.

#include <algorithm>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SpeedGate : public rclcpp::Node
{
public:
  SpeedGate()
  : Node("speed_gate"), last_scale_time_(now())
  {
    timeout_s_ = declare_parameter("safety_timeout_s", 0.5);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

    scale_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/safety/speed_scale", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        scale_ = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
        last_scale_time_ = now();
        have_scale_ = true;
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double s = effective_scale();
        geometry_msgs::msg::Twist out = *msg;
        out.linear.x *= s;
        out.linear.y *= s;
        out.angular.z *= s;
        pub_->publish(out);
      });

    RCLCPP_INFO(get_logger(), "speed_gate ready (timeout %.2f s)", timeout_s_);
  }

private:
  double effective_scale()
  {
    if (!have_scale_) {
      return 0.0;
    }
    if ((now() - last_scale_time_).seconds() > timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "safety scale stale: stopping");
      return 0.0;
    }
    return scale_;
  }

  double timeout_s_{0.5};
  double scale_{0.0};
  bool have_scale_{false};
  rclcpp::Time last_scale_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedGate>());
  rclcpp::shutdown();
  return 0;
}
