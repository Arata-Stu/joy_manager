#include <algorithm>
#include <chrono>
#include <cmath>
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
    speed_offset_(0.0),
    prev_start_pressed_(false),
    prev_stop_pressed_(false),
    prev_steer_inc_pressed_(false),
    prev_steer_dec_pressed_(false),
    prev_speed_inc_pressed_(false),
    prev_speed_dec_pressed_(false),
    prev_scale_inc_pressed_(false),
    prev_scale_dec_pressed_(false)
  {
    // --- パラメータ宣言＆取得 ---
    declare_parameter<double>("speed_scale",      1.0);
    declare_parameter<double>("steer_scale",      1.0);
    declare_parameter<double>("offset_increment", 0.01);
    declare_parameter<double>("max_steer_offset", 0.3);
    declare_parameter<double>("min_steer_offset", -0.3);
    declare_parameter<double>("max_speed_offset", 1.0);
    declare_parameter<double>("min_speed_offset", -1.0);
    declare_parameter<double>("initial_steer_offset", 0.0);
    declare_parameter<double>("initial_speed_offset", 0.0);

    declare_parameter<int>("joy_button_index",   2);
    declare_parameter<int>("ack_button_index",   3);
    declare_parameter<int>("start_button_index", 9);
    declare_parameter<int>("stop_button_index",  8);

    declare_parameter<bool>("invert_speed", true);
    declare_parameter<bool>("invert_steer", false);

    declare_parameter<double>("timer_hz", 40.0);

    get_parameter("speed_scale",      speed_scale_);
    get_parameter("steer_scale",      steer_scale_);
    get_parameter("offset_increment", offset_increment_);
    get_parameter("max_steer_offset", max_steer_offset_);
    get_parameter("min_steer_offset", min_steer_offset_);
    get_parameter("max_speed_offset", max_speed_offset_);
    get_parameter("min_speed_offset", min_speed_offset_);
    get_parameter("initial_steer_offset", steer_offset_);
    get_parameter("initial_speed_offset", speed_offset_);

    get_parameter("joy_button_index",   joy_button_index_);
    get_parameter("ack_button_index",   ack_button_index_);
    get_parameter("start_button_index", start_button_index_);
    get_parameter("stop_button_index",  stop_button_index_);

    get_parameter("invert_speed",  invert_speed_);
    get_parameter("invert_steer",  invert_steer_);
    get_parameter("timer_hz",      timer_hz_);

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
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // --- 0) start/stop トリガ処理（ボタン，連射防止） ---
    bool curr_start = (start_button_index_ < (int)msg->buttons.size()
                       && msg->buttons[start_button_index_] == 1);
    bool curr_stop  = (stop_button_index_ < (int)msg->buttons.size()
                       && msg->buttons[stop_button_index_]  == 1);

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

    // --- 1) joy/ack モード判定 ---
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
      double raw_speed = (msg->axes.size() > 1 ? msg->axes[1] : 0.0);
      double raw_steer = (msg->axes.size() > 2 ? msg->axes[2] : 0.0);

      if (invert_speed_) raw_speed = -raw_speed;
      if (invert_steer_) raw_steer = -raw_steer;

      joy_speed_ = raw_speed * speed_scale_ + speed_offset_;
      joy_steer_ = raw_steer * steer_scale_ + steer_offset_;
    }

    // --- 3) D-pad でのオフセット調整（axes[6], axes[7]，連射防止付き） ---
    double a6 = (msg->axes.size() > 6 ? msg->axes[6] : 0.0);
    double a7 = (msg->axes.size() > 7 ? msg->axes[7] : 0.0);

    // Steer offset: →(–1.0) +, ←(+1.0) –
    bool steer_inc = (std::abs(a6 + 1.0) < 1e-3);
    bool steer_dec = (std::abs(a6 - 1.0) < 1e-3);
    if (steer_inc && !prev_steer_inc_pressed_) {
      steer_offset_ = std::min(steer_offset_ + offset_increment_, max_steer_offset_);
      steer_offset_ = std::round(steer_offset_ / offset_increment_) * offset_increment_;
      if (std::fabs(steer_offset_) < 1e-6) steer_offset_ = 0.0;
      RCLCPP_INFO(get_logger(), "steer_offset = %.2f", steer_offset_);
      prev_steer_inc_pressed_ = true;
    } else if (!steer_inc) {
      prev_steer_inc_pressed_ = false;
    }
    if (steer_dec && !prev_steer_dec_pressed_) {
      steer_offset_ = std::max(steer_offset_ - offset_increment_, min_steer_offset_);
      steer_offset_ = std::round(steer_offset_ / offset_increment_) * offset_increment_;
      if (std::fabs(steer_offset_) < 1e-6) steer_offset_ = 0.0;
      RCLCPP_INFO(get_logger(), "steer_offset = %.2f", steer_offset_);
      prev_steer_dec_pressed_ = true;
    } else if (!steer_dec) {
      prev_steer_dec_pressed_ = false;
    }

    // Speed offset: ↑(+1.0) +, ↓(–1.0) –
    bool speed_inc = (std::abs(a7 - 1.0) < 1e-3);
    bool speed_dec = (std::abs(a7 + 1.0) < 1e-3);
    if (speed_inc && !prev_speed_inc_pressed_) {
      speed_offset_ = std::min(speed_offset_ + offset_increment_, max_speed_offset_);
      speed_offset_ = std::round(speed_offset_ / offset_increment_) * offset_increment_;
      if (std::fabs(speed_offset_) < 1e-6) speed_offset_ = 0.0;
      RCLCPP_INFO(get_logger(), "speed_offset = %.2f", speed_offset_);
      prev_speed_inc_pressed_ = true;
    } else if (!speed_inc) {
      prev_speed_inc_pressed_ = false;
    }
    if (speed_dec && !prev_speed_dec_pressed_) {
      speed_offset_ = std::max(speed_offset_ - offset_increment_, min_speed_offset_);
      speed_offset_ = std::round(speed_offset_ / offset_increment_) * offset_increment_;
      if (std::fabs(speed_offset_) < 1e-6) speed_offset_ = 0.0;
      RCLCPP_INFO(get_logger(), "speed_offset = %.2f", speed_offset_);
      prev_speed_dec_pressed_ = true;
    } else if (!speed_dec) {
      prev_speed_dec_pressed_ = false;
    }

    // --- 4) steer_scale の動的調整（R1/L1，連射防止，0.1刻み） ---
    bool scale_inc = (msg->buttons.size() > 5 && msg->buttons[5] == 1); // R1
    bool scale_dec = (msg->buttons.size() > 4 && msg->buttons[4] == 1); // L1

    if (scale_inc && !prev_scale_inc_pressed_) {
      steer_scale_ = std::round((steer_scale_ + 0.1) * 10.0) / 10.0;
      if (steer_scale_ < 0.0) steer_scale_ = 0.0;
      RCLCPP_INFO(get_logger(), "steer_scale = %.1f", steer_scale_);
      prev_scale_inc_pressed_ = true;
    } else if (!scale_inc) {
      prev_scale_inc_pressed_ = false;
    }

    if (scale_dec && !prev_scale_dec_pressed_) {
      steer_scale_ = std::max(steer_scale_ - 0.1, 0.0);
      steer_scale_ = std::round(steer_scale_ * 10.0) / 10.0;
      RCLCPP_INFO(get_logger(), "steer_scale = %.1f", steer_scale_);
      prev_scale_dec_pressed_ = true;
    } else if (!scale_dec) {
      prev_scale_dec_pressed_ = false;
    }
  }

  void ack_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    last_autonomy_msg_ = *msg;
  }

  void timer_callback()
  {
    ackermann_msgs::msg::AckermannDrive out;
    if (joy_active_) {
      out.speed          = joy_speed_;
      out.steering_angle = joy_steer_;
    } else if (ack_active_) {
      out = last_autonomy_msg_;
    } else {
      out.speed          = 0.0;
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
  double speed_scale_, steer_scale_, offset_increment_;
  double max_steer_offset_, min_steer_offset_;
  double max_speed_offset_, min_speed_offset_;
  bool invert_speed_, invert_steer_;
  int joy_button_index_, ack_button_index_;
  int start_button_index_, stop_button_index_;
  double timer_hz_;

  // 状態
  bool joy_active_, ack_active_;
  double joy_speed_, joy_steer_;
  double steer_offset_, speed_offset_;
  ackermann_msgs::msg::AckermannDrive last_autonomy_msg_;

  // 連射防止フラグ
  bool prev_start_pressed_, prev_stop_pressed_;
  bool prev_steer_inc_pressed_, prev_steer_dec_pressed_;
  bool prev_speed_inc_pressed_, prev_speed_dec_pressed_;
  bool prev_scale_inc_pressed_, prev_scale_dec_pressed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
