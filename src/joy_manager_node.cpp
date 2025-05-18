#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"

using namespace std::placeholders;

class JoyManagerNode : public rclcpp::Node {
public:
  JoyManagerNode()
  : Node("joy_manager_node"),
    joy_control_active_(false),
    ackermann_control_active_(false),
    recording_stopped_(false),
    speed_scale_(1.0),
    timer_hz_(100.0),
    joy_button_index_(2),
    ack_button_index_(3),
    stop_axis_index_(7),
    stop_axis_value_(-1.0),
    start_axis_index_(7),
    start_axis_value_(1.0)
  {
    // パラメータ宣言
    declare_parameter<double>("speed_scale", speed_scale_);
    declare_parameter<double>("timer_hz", timer_hz_);
    declare_parameter<int>("joy_button_index", joy_button_index_);
    declare_parameter<int>("ack_button_index", ack_button_index_);
    declare_parameter<int>("stop_axis_index", stop_axis_index_);
    declare_parameter<double>("stop_axis_value", stop_axis_value_);
    declare_parameter<int>("start_axis_index", start_axis_index_);
    declare_parameter<double>("start_axis_value", start_axis_value_);

    // パラメータ取得
    get_parameter("speed_scale", speed_scale_);
    get_parameter("timer_hz", timer_hz_);
    get_parameter("joy_button_index", joy_button_index_);
    get_parameter("ack_button_index", ack_button_index_);
    get_parameter("stop_axis_index", stop_axis_index_);
    get_parameter("stop_axis_value", stop_axis_value_);
    get_parameter("start_axis_index", start_axis_index_);
    get_parameter("start_axis_value", start_axis_value_);

    // サブスクライバとパブリッシャ
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyManagerNode::joyCallback, this, _1)
    );
    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ackermannCallback, this, _1)
    );
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
      "/jetracer/cmd_drive", 10
    );

    // タイマー
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz_));
    timer_ = create_wall_timer(period, std::bind(&JoyManagerNode::publishDrive, this));

    // パラメータコールバック
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&JoyManagerNode::onParameterEvent, this, _1)
    );

    // Rosbag2 サービスクライアント
    stop_client_ = create_client<rosbag2_interfaces::srv::Pause>(
      "/rosbag2_recorder/start"
    );
    start_client_ = create_client<rosbag2_interfaces::srv::Resume>(
      "/rosbag2_recorder/stop"
    );

    RCLCPP_INFO(get_logger(),
      "JoyManagerNode started: timer_hz=%.1f, joy_btn=%d, ack_btn=%d, stop_axis=%d(%.1f), start_axis=%d(%.1f)",
      timer_hz_, joy_button_index_, ack_button_index_,
      stop_axis_index_, stop_axis_value_, start_axis_index_, start_axis_value_
    );
  }

private:
  // パラメータ変更コールバック
  rcl_interfaces::msg::SetParametersResult onParameterEvent(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &p : params) {
      if (p.get_name() == "speed_scale") {
        speed_scale_ = std::max(0.1, p.as_double());
        RCLCPP_INFO(get_logger(), "Speed scale updated: %.2f", speed_scale_);
      } else if (p.get_name() == "timer_hz") {
        timer_hz_ = p.as_double();
        timer_->cancel();
        timer_ = create_wall_timer(
          std::chrono::milliseconds(int(1000.0/timer_hz_)),
          std::bind(&JoyManagerNode::publishDrive, this)
        );
        RCLCPP_INFO(get_logger(), "Timer Hz updated: %.1f", timer_hz_);
      } else if (p.get_name() == "joy_button_index") {
        joy_button_index_ = p.as_int();
        RCLCPP_INFO(get_logger(), "Joy button index updated: %d", joy_button_index_);
      } else if (p.get_name() == "ack_button_index") {
        ack_button_index_ = p.as_int();
        RCLCPP_INFO(get_logger(), "Ackermann button index updated: %d", ack_button_index_);
      } else if (p.get_name() == "stop_axis_index") {
        stop_axis_index_ = p.as_int();
        RCLCPP_INFO(get_logger(), "Stop axis index updated: %d", stop_axis_index_);
      } else if (p.get_name() == "stop_axis_value") {
        stop_axis_value_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Stop axis value updated: %.1f", stop_axis_value_);
      } else if (p.get_name() == "start_axis_index") {
        start_axis_index_ = p.as_int();
        RCLCPP_INFO(get_logger(), "Start axis index updated: %d", start_axis_index_);
      } else if (p.get_name() == "start_axis_value") {
        start_axis_value_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Start axis value updated: %.1f", start_axis_value_);
      }
    }
    return result;
  }

  // Joy入力コールバック
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Rosbag制御
    if (msg->axes.size() > (size_t)stop_axis_index_ &&
        msg->axes[stop_axis_index_] == stop_axis_value_ && !recording_stopped_) {
      auto req = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();
      stop_client_->async_send_request(req);
      recording_stopped_ = true;
      RCLCPP_INFO(get_logger(), "Rosbag stopped (axis[%d]==%.1f)", stop_axis_index_, stop_axis_value_);
    }
    if (msg->axes.size() > (size_t)start_axis_index_ &&
        msg->axes[start_axis_index_] == start_axis_value_ && recording_stopped_) {
      auto req = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
      start_client_->async_send_request(req);
      recording_stopped_ = false;
      RCLCPP_INFO(get_logger(), "Rosbag started (axis[%d]==%.1f)", start_axis_index_, start_axis_value_);
    }

    // ドライブ制御モード切替
    joy_control_active_ = (msg->buttons[joy_button_index_] == 1);
    ackermann_control_active_ = (msg->buttons[ack_button_index_] == 1);

    // スピードスケール調整 (buttons 5/4)
    if (msg->buttons.size() > 5 && msg->buttons[5] == 1) {
      set_parameter(rclcpp::Parameter("speed_scale", speed_scale_ + 0.1));
    }
    if (msg->buttons.size() > 4 && msg->buttons[4] == 1) {
      set_parameter(rclcpp::Parameter("speed_scale", speed_scale_ - 0.1));
    }

    // Joy 制御時ドライブコマンド設定
    if (joy_control_active_) {
      current_drive_.steering_angle = msg->axes[0];
      current_drive_.speed = msg->axes[4] * speed_scale_;
    }
  }

  // Ackermannコマンド受信
  void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
    if (ackermann_control_active_) current_drive_ = *msg;
  }

  // 定期パブリッシュ
  void publishDrive() {
    if (!joy_control_active_ && !ackermann_control_active_) {
      current_drive_.steering_angle = 0.0;
      current_drive_.speed = 0.0;
    }
    drive_pub_->publish(current_drive_);
  }

  // メンバ
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr stop_client_;
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr start_client_;

  ackermann_msgs::msg::AckermannDrive current_drive_;
  bool joy_control_active_;
  bool ackermann_control_active_;
  bool recording_stopped_;
  double speed_scale_;
  double timer_hz_;
  int joy_button_index_;
  int ack_button_index_;
  int stop_axis_index_;
  double stop_axis_value_;
  int start_axis_index_;
  double start_axis_value_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
