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
          steering_scale_(1.0),
          speed_scale_(1.0) {

        // サブスクリプション
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyManagerNode::joyCallback, this, _1));

        ackermann_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ackermannCallback, this, _1));

        // パブリッシャー
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/cmd_drive", 10);

        // タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JoyManagerNode::publishDrive, this));

        // 動的パラメータ
        this->declare_parameter("steering_scale", 1.0);
        this->declare_parameter("speed_scale", 1.0);

        RCLCPP_INFO(this->get_logger(), "JoyManagerNode has been started.");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        joy_control_active_ = msg->buttons[0] == 1;
        ackermann_control_active_ = msg->buttons[1] == 1;

        // スケール調整
        if (msg->buttons[2] == 1) steering_scale_ += 0.1;
        if (msg->buttons[3] == 1) steering_scale_ -= 0.1;
        if (msg->buttons[4] == 1) speed_scale_ += 0.1;
        if (msg->buttons[5] == 1) speed_scale_ -= 0.1;

        // 最小値を制限
        steering_scale_ = std::max(0.1, steering_scale_);
        speed_scale_ = std::max(0.1, speed_scale_);

        RCLCPP_INFO(this->get_logger(), "Steering Scale: %.2f, Speed Scale: %.2f", steering_scale_, speed_scale_);

        // ジョイパッドの値にスケールを掛ける
        if (joy_control_active_) {
            current_drive_.steering_angle = msg->axes[0] * steering_scale_;
            current_drive_.speed = msg->axes[1] * speed_scale_;
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

    // ROS2パラメータコールバック
    void updateParameters() {
        this->get_parameter("steering_scale", steering_scale_);
        this->get_parameter("speed_scale", speed_scale_);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    ackermann_msgs::msg::AckermannDrive current_drive_;
    bool joy_control_active_;
    bool ackermann_control_active_;
    double steering_scale_;
    double speed_scale_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
