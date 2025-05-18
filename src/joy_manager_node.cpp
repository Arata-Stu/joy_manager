#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::placeholders;

class JoyManagerNode : public rclcpp::Node {
public:
  JoyManagerNode()
  : Node("joy_manager_node"),
    previous_start_pressed_(false),
    previous_stop_pressed_(false),
    previous_increase_pressed_(false),
    previous_decrease_pressed_(false)
  {
    // パラメータの宣言（デフォルト値）
    declare_parameter<double>("speed_scale", 1.0);
    declare_parameter<double>("timer_hz", 100.0);
    declare_parameter<int>("joy_button_index", 2);
    declare_parameter<int>("ack_button_index", 3);
    declare_parameter<int>("stop_axis_index", 7);
    declare_parameter<double>("stop_axis_value", -1.0);
    declare_parameter<int>("start_axis_index", 7);
    declare_parameter<double>("start_axis_value", 1.0);
    declare_parameter<bool>("invert_speed", false);
    declare_parameter<bool>("invert_steer", false);
    declare_parameter<double>("max_steer_offset", 0.2);
    declare_parameter<double>("min_steer_offset", -0.2);
    declare_parameter<double>("offset_increment", 0.01);
    declare_parameter<int>("increase_steer_button_index", 11);
    declare_parameter<int>("decrease_steer_button_index", 12);

    // パラメータの取得（外部ファイルから上書き可能）
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
    get_parameter("increase_steer_button_index", increase_steer_button_index_);
    get_parameter("decrease_steer_button_index", decrease_steer_button_index_);

    // サブスクライバとパブリッシャの設定
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

    RCLCPP_INFO(get_logger(), "JoyManagerNode started with external params");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    joy_control_active_ = (msg->buttons[joy_button_index_] == 1);
    ackermann_control_active_ = (msg->buttons[ack_button_index_] == 1);

    // === Offsetの調整（連射防止） ===
    bool current_increase_pressed = (msg->buttons[increase_steer_button_index_] == 1);
    bool current_decrease_pressed = (msg->buttons[decrease_steer_button_index_] == 1);

    if (current_increase_pressed && !previous_increase_pressed_) {
        steer_offset_ = std::min(steer_offset_ + offset_increment_, max_steer_offset_);
        RCLCPP_INFO(get_logger(), "Steer Offset Increased: %f", steer_offset_);
    }

    if (current_decrease_pressed && !previous_decrease_pressed_) {
        steer_offset_ = std::max(steer_offset_ - offset_increment_, min_steer_offset_);
        RCLCPP_INFO(get_logger(), "Steer Offset Decreased: %f", steer_offset_);
    }

    // 状態更新
    previous_increase_pressed_ = current_increase_pressed;
    previous_decrease_pressed_ = current_decrease_pressed;

    // === Rosbag Triggerの処理（連射防止） ===
    bool current_start_pressed = (msg->axes[start_axis_index_] == start_axis_value_);
    bool current_stop_pressed = (msg->axes[stop_axis_index_] == stop_axis_value_);

    if (current_start_pressed && !previous_start_pressed_) {
        std_msgs::msg::Bool trigger_msg;
        trigger_msg.data = true;
        rosbag_trigger_pub_->publish(trigger_msg);
        RCLCPP_INFO(get_logger(), "Rosbag recording started");
    }

    if (current_stop_pressed && !previous_stop_pressed_) {
        std_msgs::msg::Bool trigger_msg;
        trigger_msg.data = false;
        rosbag_trigger_pub_->publish(trigger_msg);
        RCLCPP_INFO(get_logger(), "Rosbag recording stopped");
    }

    const double epsilon = 1e-4; // 誤差の許容範囲
    if (std::abs(steer_offset_) < epsilon) {
        steer_offset_ = 0.0;
        RCLCPP_INFO(get_logger(), "Steer Offset was close to zero. Adjusted to 0.0");
    }

    previous_start_pressed_ = current_start_pressed;
    previous_stop_pressed_ = current_stop_pressed;
  }

  void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    if (ackermann_control_active_) {
      current_drive_ = *msg;
      current_drive_.steering_angle += steer_offset_;
    }
  }

  void publishDrive() {
    if (joy_control_active_) {
        // Joyモード: コントローラの入力を反映
        current_drive_.steering_angle = msg_->axes[3] + steer_offset_;  // ステアリング + オフセット
        current_drive_.speed = msg_->axes[2] * speed_scale_;            // 速度スケーリング
        if (speed_inverted_) {
            current_drive_.speed = -current_drive_.speed;
        }
        if (steer_inverted_) {
            current_drive_.steering_angle = -current_drive_.steering_angle;
        }
    } else if (ackermann_control_active_) {
        // アッカーマンモード: 外部コマンドをそのまま出力
        current_drive_.steering_angle += steer_offset_;
    } else {
        // どちらもアクティブでない場合: 停止
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
  bool speed_inverted_;
  bool steer_inverted_;
  double steer_offset_;
  double max_steer_offset_;
  double min_steer_offset_;
  double offset_increment_;
  int increase_steer_button_index_;
  int decrease_steer_button_index_;

  // 新しく追加したメンバ
  bool previous_start_pressed_;
  bool previous_stop_pressed_;
  bool previous_increase_pressed_;
  bool previous_decrease_pressed_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
