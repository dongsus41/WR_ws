#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>   // close
#include <cstring>    // strerror, memcpy
#include <cerrno>     // errno
#include <chrono>     // std::chrono::seconds

class CANCommandTransmitNode : public rclcpp::Node {
public:
    CANCommandTransmitNode() : Node("can_command_transmit") {
        // 구독자 설정
        actuator_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&CANCommandTransmitNode::actuator_callback, this, std::placeholders::_1));

        fan_sub_ = this->create_subscription<wearable_robot_interfaces::msg::FanCommand>(
            "fan_command", 10,
            std::bind(&CANCommandTransmitNode::fan_callback, this, std::placeholders::_1));

        // CAN 초기화 시도 및 재시도 타이머 설정
        if (!setup_CAN()) {
            retry_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&CANCommandTransmitNode::retry_setup, this));
        }
    }

    ~CANCommandTransmitNode() {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    // 멤버 변수 선언 추가
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr actuator_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::FanCommand>::SharedPtr fan_sub_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    int can_socket_ = -1;
    const uint32_t COMMAND_ID = 0x400;
    const size_t MAX_RETRIES = 3;

    uint8_t last_pwm_values_[4] = {0};  // 마지막 PWM 값 저장
    uint8_t last_fan_states_[4] = {0};   // 마지막 FAN 상태 저장

    void retry_setup() {
        if (setup_CAN()) {
            retry_timer_->cancel();
            RCLCPP_INFO(get_logger(), "CAN setup successful after retry");
        }
    }

    bool setup_CAN() {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create CAN socket: %s", strerror(errno));
            return false;
        }

        struct ifreq ifr = {};
        strcpy(ifr.ifr_name, "can0");
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to get CAN interface index: %s", strerror(errno));
            close(can_socket_);
            return false;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket: %s", strerror(errno));
            close(can_socket_);
            return false;
        }

        int enable_canfd = 1;
        if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                      &enable_canfd, sizeof(enable_canfd)) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to enable CAN FD: %s", strerror(errno));
            close(can_socket_);
            return false;
        }

        return true;
    }

    bool send_command(const uint8_t* data, size_t length, size_t retry_count = 0) {
        if (can_socket_ < 0) {
            RCLCPP_ERROR(get_logger(), "CAN socket not initialized");
            return false;
        }

        struct canfd_frame frame = {};
        frame.can_id = COMMAND_ID;
        frame.flags = CANFD_BRS;
        frame.len = 64;  // 항상 64바이트로 고정
        memcpy(frame.data, data, std::min(length, static_cast<size_t>(64)));

        ssize_t nbytes = write(can_socket_, &frame, sizeof(struct canfd_frame));
        if (nbytes != sizeof(struct canfd_frame)) {
            if (retry_count < MAX_RETRIES) {
                RCLCPP_WARN(get_logger(), "Retrying CAN transmission (%zu/%zu)",
                           retry_count + 1, MAX_RETRIES);
                if (setup_CAN()) {
                    return send_command(data, length, retry_count + 1);
                }
            }
            RCLCPP_ERROR(get_logger(), "Failed to send CAN frame: %s", strerror(errno));
            return false;
        }

        return true;
    }

    // 값을 0-100 범위로 제한하는 헬퍼 함수
    uint8_t limit_pwm(int value) {
        if (value < 0) return 0;
        if (value > 100) return 100;
        return static_cast<uint8_t>(value);
    }

    void actuator_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg) {
        uint8_t data[64] = {0};

        // PWM 값 범위 제한 및 설정
        data[0] = limit_pwm(msg->pwm0);
        data[1] = limit_pwm(msg->pwm1);
        data[2] = limit_pwm(msg->pwm2);
        data[3] = limit_pwm(msg->pwm3);

        memcpy(last_pwm_values_, data, 4);

        memcpy(&data[4], last_fan_states_, 4);

        if (send_command(data, sizeof(data))) {
            RCLCPP_DEBUG(get_logger(), "Actuator command sent successfully");
        }
    }

    void fan_callback(const wearable_robot_interfaces::msg::FanCommand::SharedPtr msg) {
        uint8_t data[64] = {0};

        memcpy(data, last_pwm_values_, 4);

        // FAN 상태를 CAN 데이터에 설정
        data[4] = msg->fan0 ? 1 : 0;
        data[5] = msg->fan1 ? 1 : 0;
        data[6] = msg->fan2 ? 1 : 0;
        data[7] = msg->fan3 ? 1 : 0;

        memcpy(last_fan_states_, &data[4], 4);

        if (send_command(data, sizeof(data))) {
            RCLCPP_DEBUG(get_logger(), "Fan command sent successfully");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANCommandTransmitNode>());
    rclcpp::shutdown();
    return 0;
}
