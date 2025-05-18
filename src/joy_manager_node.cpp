#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

using std::placeholders::_1;

class JoyManagerNode : public rclcpp::Node {
public:
    JoyManagerNode()
        : Node("joy_manager_node"),
          joy_control_active_(false),
          ackermann_control_active_(false),
          speed_scale_(1.0),
          timer_hz_(100.0),
          joy_button_index_(2),
          ack_button_index_(3)
    {
        // パラメータ宣言
        this->declare_parameter("speed_scale", speed_scale_);
        this->declare_parameter("timer_hz", timer_hz_);
        this->declare_parameter("joy_button_index", joy_button_index_);
        this->declare_parameter("ack_button_index", ack_button_index_);

        // パラメータ取得
        this->get_parameter("speed_scale", speed_scale_);
        this->get_parameter("timer_hz", timer_hz_);
        this->get_parameter("joy_button_index", joy_button_index_);
        this->get_parameter("ack_button_index", ack_button_index_);

        // サブスクリプション
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyManagerNode::joyCallback, this, _1));

        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ackermannCallback, this, _1));

        // パブリッシャー
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/jetracer/cmd_drive", 10);

        // タイマー（timer_hz_ を使用）
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz_));
        timer_ = this->create_wall_timer(
            period, std::bind(&JoyManagerNode::publishDrive, this));

        // パラメータコールバック
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JoyManagerNode::onParameterEvent, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Node started: timer_hz=%.1f, joy_btn=%d, ack_btn=%d",
                    timer_hz_, joy_button_index_, ack_button_index_);
    }

private:
    rcl_interfaces::msg::SetParametersResult onParameterEvent(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : params) {
            if (param.get_name() == "speed_scale") {
                double new_scale = param.as_double();
                if (new_scale != speed_scale_) {
                    speed_scale_ = std::max(0.1, new_scale);
                    RCLCPP_INFO(this->get_logger(),
                                "Speed Scale changed: %.2f", speed_scale_);
                }
            } else if (param.get_name() == "timer_hz") {
                double new_timer_hz = param.as_double();
                if (new_timer_hz != timer_hz_) {
                    timer_hz_ = new_timer_hz;
                    auto new_period = std::chrono::milliseconds(static_cast<int>(1000.0 / timer_hz_));
                    timer_->cancel();
                    timer_ = this->create_wall_timer(
                        new_period, std::bind(&JoyManagerNode::publishDrive, this));
                    RCLCPP_INFO(this->get_logger(),
                                "Timer Hz changed: %.1f", timer_hz_);
                }
            } else if (param.get_name() == "joy_button_index") {
                joy_button_index_ = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "Joy button index changed: %d", joy_button_index_);
            } else if (param.get_name() == "ack_button_index") {
                ack_button_index_ = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "Ackermann button index changed: %d", ack_button_index_);
            }
        }
        return result;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // コントロールモード切替（パラメータ化）
        joy_control_active_ = (msg->buttons[joy_button_index_] == 1);
        ackermann_control_active_ = (msg->buttons[ack_button_index_] == 1);

        // speed スケール調整
        if (msg->buttons[5] == 1) {
            double new_scale = speed_scale_ + 0.1;
            this->set_parameter(rclcpp::Parameter("speed_scale", new_scale));
        }
        if (msg->buttons[4] == 1) {
            double new_scale = speed_scale_ - 0.1;
            this->set_parameter(rclcpp::Parameter("speed_scale", new_scale));
        }

        // joy コントロール時の drive 設定
        if (joy_control_active_) {
            current_drive_.steering_angle = msg->axes[0];
            current_drive_.speed = msg->axes[4] * speed_scale_;
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
        drive_publisher_->publish(current_drive_);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    ackermann_msgs::msg::AckermannDrive current_drive_;
    bool joy_control_active_;
    bool ackermann_control_active_;
    double speed_scale_;
    double timer_hz_;
    int joy_button_index_;
    int ack_button_index_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
