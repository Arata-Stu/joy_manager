#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::placeholders;

class JoyManagerNode : public rclcpp::Node {
public:
  JoyManagerNode()
  : Node("joy_manager_node"),
    joy_control_active_(false),
    ackermann_control_active_(false),
    speed_scale_(1.0),
    timer_hz_(100.0),
    joy_button_index_(2),
    ack_button_index_(3),
    stop_axis_index_(7),
    stop_axis_value_(-1.0),
    start_axis_index_(7),
    start_axis_value_(1.0),
    previous_stop_pressed_(false),
    previous_start_pressed_(false),
    speed_inverted_(false),
    steer_inverted_(false),
    steer_offset_(0.0),
    max_steer_offset_(0.2),
    min_steer_offset_(-0.2),
    offset_increment_(0.01),
    increase_steer_button_index_(11),
    decrease_steer_button_index_(12)
  {
    declare_parameter<double>("speed_scale", speed_scale_);
    declare_parameter<double>("timer_hz", timer_hz_);
    declare_parameter<int>("joy_button_index", joy_button_index_);
    declare_parameter<int>("ack_button_index", ack_button_index_);
    declare_parameter<int>("stop_axis_index", stop_axis_index_);
    declare_parameter<double>("stop_axis_value", stop_axis_value_);
    declare_parameter<int>("start_axis_index", start_axis_index_);
    declare_parameter<double>("start_axis_value", start_axis_value_);
    declare_parameter<bool>("invert_speed", speed_inverted_);
    declare_parameter<bool>("invert_steer", steer_inverted_);
    declare_parameter<double>("max_steer_offset", max_steer_offset_);
    declare_parameter<double>("min_steer_offset", min_steer_offset_);
    declare_parameter<double>("offset_increment", offset_increment_);

    get_parameter("speed_scale", speed_scale_);
    get_parameter("timer_hz", timer_hz_);
    get_parameter("joy_button_index", joy_button_index_);
    get_parameter("ack_button_index", ack_button_index_);
    get_parameter("stop_axis_index", stop_axis_index_);
    get_parameter("stop_axis_value", stop_axis_value_);
    get_parameter("start_axis_index", start_axis_index_);
    get_parameter("start_axis_value", start_axis_value_);
    get_parameter("invert_speed", speed_inverted_);
    get_parameter("invert_steer", steer_inverted_);
    get_parameter("max_steer_offset", max_steer_offset_);
    get_parameter("min_steer_offset", min_steer_offset_);
    get_parameter("offset_increment", offset_increment_);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyManagerNode::joyCallback, this, _1)
    );

    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ackermannCallback, this, _1)
    );

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
      "/jetracer/cmd_drive", 10
    );

    rosbag_trigger_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/rosbag2_recorder/trigger", 10
    );

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz_));
    timer_ = create_wall_timer(period, std::bind(&JoyManagerNode::publishDrive, this));

    RCLCPP_INFO(get_logger(), "JoyManagerNode started");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_control_active_ = (msg->buttons[joy_button_index_] == 1);
    ackermann_control_active_ = (msg->buttons[ack_button_index_] == 1);

    // Steer offset adjustment
    if (msg->buttons[increase_steer_button_index_] == 1) {
      steer_offset_ = std::min(steer_offset_ + offset_increment_, max_steer_offset_);
    }
    if (msg->buttons[decrease_steer_button_index_] == 1) {
      steer_offset_ = std::max(steer_offset_ - offset_increment_, min_steer_offset_);
    }

    bool stop_pressed = (msg->axes.size() > (size_t)stop_axis_index_ && msg->axes[stop_axis_index_] == stop_axis_value_);
    if (stop_pressed && !previous_stop_pressed_) {
      std_msgs::msg::Bool trigger_msg;
      trigger_msg.data = false;
      rosbag_trigger_pub_->publish(trigger_msg);
    }
    previous_stop_pressed_ = stop_pressed;

    bool start_pressed = (msg->axes.size() > (size_t)start_axis_index_ && msg->axes[start_axis_index_] == start_axis_value_);
    if (start_pressed && !previous_start_pressed_) {
      std_msgs::msg::Bool trigger_msg;
      trigger_msg.data = true;
      rosbag_trigger_pub_->publish(trigger_msg);
    }
    previous_start_pressed_ = start_pressed;

    if (joy_control_active_) {
      current_drive_.steering_angle = steer_inverted_ ? -msg->axes[0] : msg->axes[0];
      current_drive_.steering_angle += steer_offset_;  // <-- オフセットを追加
      current_drive_.speed = speed_inverted_ ? -msg->axes[4] * speed_scale_ : msg->axes[4] * speed_scale_;
    }
  }

  void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    if (ackermann_control_active_) current_drive_ = *msg;
  }

  void publishDrive() {
    if (!joy_control_active_ && !ackermann_control_active_) {
      current_drive_.steering_angle = 0.0;
      current_drive_.speed = 0.0;
    }
    drive_pub_->publish(current_drive_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rosbag_trigger_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ackermann_msgs::msg::AckermannDrive current_drive_;

  bool joy_control_active_;
  bool ackermann_control_active_;
  double speed_scale_;
  double timer_hz_;
  int joy_button_index_;
  int ack_button_index_;
  int stop_axis_index_;
  double stop_axis_value_;
  int start_axis_index_;
  double start_axis_value_;
  bool previous_stop_pressed_;
  bool previous_start_pressed_;
  bool speed_inverted_;
  bool steer_inverted_;

  double steer_offset_;
  double max_steer_offset_;
  double min_steer_offset_;
  double offset_increment_;
  int increase_steer_button_index_;
  int decrease_steer_button_index_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
