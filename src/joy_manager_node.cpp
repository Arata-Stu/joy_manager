#include <algorithm>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyManagerNode : public rclcpp::Node
{
public:
  JoyManagerNode()
  : Node("joy_manager_node"),
    joy_active_(false),
    ack_active_(false),
    joy_speed_(0.0),
    joy_steer_(0.0),
    steer_offset_(0.0),
    prev_start_pressed_(false),
    prev_stop_pressed_(false),
    prev_increase_pressed_(false),
    prev_decrease_pressed_(false)
  {
    // --- パラメータ宣言＆取得 ---
    declare_parameter<double>("speed_scale", 1.0);
    declare_parameter<double>("timer_hz", 40.0);
    declare_parameter<int>("joy_button_index", 2);
    declare_parameter<int>("ack_button_index", 3);
    declare_parameter<int>("stop_axis_index", 6);
    declare_parameter<double>("stop_axis_value", 1.0);
    declare_parameter<int>("start_axis_index", 6);
    declare_parameter<double>("start_axis_value", -1.0);
    declare_parameter<bool>("invert_speed", true);
    declare_parameter<bool>("invert_steer", false);
    declare_parameter<int>("increase_steer_button_index", 7);
    declare_parameter<int>("decrease_steer_button_index", 6);
    declare_parameter<double>("max_steer_offset", 0.3);
    declare_parameter<double>("min_steer_offset", -0.3);
    declare_parameter<double>("offset_increment", 0.01);

    get_parameter("speed_scale", speed_scale_);
    get_parameter("timer_hz", timer_hz_);
    get_parameter("joy_button_index", joy_button_index_);
    get_parameter("ack_button_index", ack_button_index_);
    get_parameter("stop_axis_index", stop_axis_index_);
    get_parameter("stop_axis_value", stop_axis_value_);
    get_parameter("start_axis_index", start_axis_index_);
    get_parameter("start_axis_value", start_axis_value_);
    get_parameter("invert_speed", invert_speed_);
    get_parameter("invert_steer", invert_steer_);
    get_parameter("increase_steer_button_index", increase_steer_button_index_);
    get_parameter("decrease_steer_button_index", decrease_steer_button_index_);
    get_parameter("max_steer_offset", max_steer_offset_);
    get_parameter("min_steer_offset", min_steer_offset_);
    get_parameter("offset_increment", offset_increment_);

    // --- サブスクライバ／パブリッシャ設定 ---
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyManagerNode::joy_callback, this, _1));

    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ack_callback, this, _1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
      "/cmd_drive", 10);

    trigger_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/rosbag2_recorder/trigger", 10);

    // --- タイマー（一定周期でコマンド出力）---
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_hz_),
      std::bind(&JoyManagerNode::timer_callback, this));
  }

private:
  // コールバック
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // --- 0) トリガー start/stop 処理（joy_active に関わらず常に反応） ---
    bool curr_start = false, curr_stop = false;
    if ((int)msg->axes.size() > start_axis_index_
        && std::abs(msg->axes[start_axis_index_] - start_axis_value_) < 1e-3) {
      curr_start = true;
    }
    if ((int)msg->axes.size() > stop_axis_index_
        && std::abs(msg->axes[stop_axis_index_] - stop_axis_value_) < 1e-3) {
      curr_stop = true;
    }

    if (curr_start && !prev_start_pressed_) {
      std_msgs::msg::Bool b; b.data = true;
      trigger_pub_->publish(b);
      prev_start_pressed_ = true;
    } else if (!curr_start) {
      prev_start_pressed_ = false;
    }

    if (curr_stop && !prev_stop_pressed_) {
      std_msgs::msg::Bool b; b.data = false;
      trigger_pub_->publish(b);
      prev_stop_pressed_ = true;
    } else if (!curr_stop) {
      prev_stop_pressed_ = false;
    }

    // --- 1) joy と ackermann ボタン状態取得 & モード切替 ---
    bool joy_pressed = (joy_button_index_ < (int)msg->buttons.size()
                        && msg->buttons[joy_button_index_] == 1);
    bool ack_pressed = (ack_button_index_ < (int)msg->buttons.size()
                        && msg->buttons[ack_button_index_] == 1);

    if (ack_pressed) {
      ack_active_ = true;
      joy_active_ = false;
    } else if (joy_pressed) {
      joy_active_ = true;
      ack_active_ = false;
    } else {
      joy_active_ = false;
      ack_active_ = false;
    }

    // --- 2) joy 操作からの速度・ステアリング算出（joy_active_ 時のみ）---
    if (joy_active_) {
      // speed: axes[1] に変更
      double raw_speed = (msg->axes.size() > 1 ? msg->axes[1] : 0.0);
      double raw_steer = (msg->axes.size() > 3 ? msg->axes[3] : 0.0);

      if (invert_speed_) raw_speed = -raw_speed;
      if (invert_steer_) raw_steer = -raw_steer;

      joy_speed_ = raw_speed * speed_scale_;
      joy_steer_ = raw_steer * speed_scale_ + steer_offset_;
    }

    // 3) オフセット調整（連射防止付き。常に反応）
    bool inc = (increase_steer_button_index_ < (int)msg->buttons.size()
                && msg->buttons[increase_steer_button_index_] == 1);
    bool dec = (decrease_steer_button_index_ < (int)msg->buttons.size()
                && msg->buttons[decrease_steer_button_index_] == 1);

    if (inc && !prev_increase_pressed_) {
      // 元の加算
      steer_offset_ = std::min(steer_offset_ + offset_increment_, max_steer_offset_);

      // 量子化：offset_increment_ の倍数に丸め
      steer_offset_ = std::round(steer_offset_ / offset_increment_) * offset_increment_;

      // ごく小さい値はゼロに
      if (std::fabs(steer_offset_) < 1e-6) {
        steer_offset_ = 0.0;
      }

      RCLCPP_INFO(get_logger(), "steer_offset = %.2f", steer_offset_);
      prev_increase_pressed_ = true;
    } else if (!inc) {
      prev_increase_pressed_ = false;
    }

    if (dec && !prev_decrease_pressed_) {
      steer_offset_ = std::max(steer_offset_ - offset_increment_, min_steer_offset_);

      steer_offset_ = std::round(steer_offset_ / offset_increment_) * offset_increment_;
      if (std::fabs(steer_offset_) < 1e-6) {
        steer_offset_ = 0.0;
      }

      RCLCPP_INFO(get_logger(), "steer_offset = %.2f", steer_offset_);
      prev_decrease_pressed_ = true;
    } else if (!dec) {
      prev_decrease_pressed_ = false;
    }
  }


  void ack_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    // 自律走行からのコマンドをキャッシュ
    last_autonomy_msg_ = *msg;
  }

  void timer_callback()
  {
    ackermann_msgs::msg::AckermannDrive out;
    if (joy_active_) {
      // 人間操作
      out.speed = joy_speed_;
      out.steering_angle = joy_steer_;
    } else if (ack_active_) {
      // 自律走行コマンド
      out = last_autonomy_msg_;
    } else {
      // 安全停止
      out.speed = 0.0;
      out.steering_angle = 0.0;
    }
    drive_pub_->publish(out);
  }

  // --- メンバ変数 ---
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // パラメータ
  double speed_scale_, timer_hz_;
  int joy_button_index_, ack_button_index_;
  int stop_axis_index_, start_axis_index_;
  double stop_axis_value_, start_axis_value_;
  bool invert_speed_, invert_steer_;
  int increase_steer_button_index_, decrease_steer_button_index_;
  double max_steer_offset_, min_steer_offset_, offset_increment_;

  // 状態保存
  bool joy_active_, ack_active_;
  double joy_speed_, joy_steer_;
  double steer_offset_;
  ackermann_msgs::msg::AckermannDrive last_autonomy_msg_;

  // 連射防止フラグ
  bool prev_start_pressed_, prev_stop_pressed_;
  bool prev_increase_pressed_, prev_decrease_pressed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
