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
          speed_scale_(1.0) {

        // サブスクリプション
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyManagerNode::joyCallback, this, _1));

        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ackermannCallback, this, _1));

        // パブリッシャー
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/jetracer/cmd_drive", 10);

        // タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&JoyManagerNode::publishDrive, this));

        // 動的パラメータ（speed_scale のみ）
        this->declare_parameter("speed_scale", 1.0);
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JoyManagerNode::onParameterEvent, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "JoyManagerNode has been started.");
    }

private:
    rcl_interfaces::msg::SetParametersResult onParameterEvent(
        const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : params) {
            if (param.get_name() == "speed_scale") {
                double new_scale = param.as_double();
                if (new_scale != speed_scale_) {
                    speed_scale_ = std::max(0.1, new_scale);
                    RCLCPP_INFO(this->get_logger(), "Speed Scale changed: %.2f", speed_scale_);
                }
            }
        }
        return result;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // コントロールモード切替
        joy_control_active_ = (msg->buttons[0] == 1);
        ackermann_control_active_ = (msg->buttons[1] == 1);

        // speed スケール調整（ボタン6,5）
        if (msg->buttons[6] == 1) {
            double new_scale = speed_scale_ + 0.1;
            this->set_parameter(rclcpp::Parameter("speed_scale", new_scale));
        }
        if (msg->buttons[5] == 1) {
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
        if (ackermann_control_active_) {
            current_drive_ = *msg;
        }
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
