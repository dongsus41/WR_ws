#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <unistd.h>

class CANCommandTransmitNode : public rclcpp::Node
{
public:
    CANCommandTransmitNode() : Node("CAN_command_transmit")
    {
        // 구독자 설정
        actuator_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&CANCommandTransmitNode::actuator_callback, this, std::placeholders::_1));

        fan_sub_ = this->create_subscription<wearable_robot_interfaces::msg::FanCommand>(
            "fan_command", 10,
            std::bind(&CANCommandTransmitNode::fan_callback, this, std::placeholders::_1));



        setup_CAN();
    }

    ~CANCommandTransmitNode()
    {
        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    int can_socket_ = -1;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr actuator_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::FanCommand>::SharedPtr fan_sub_;


    void setup_CAN()
    {
        // SocketCAN 설정
        const char* ifname = "can0";
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            throw std::runtime_error("Error creating CAN socket");
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, ifname);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("Error getting CAN interface index");
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            throw std::runtime_error("Error binding CAN socket");
        }

        // CAN FD 프레임 활성화
        int enable_canfd = 1;
        if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
            throw std::runtime_error("Error setting CAN FD support");
        }

        RCLCPP_INFO(this->get_logger(), "CAN interface setup completed");
    }

    void send_command(uint32_t can_id, const uint8_t* data, size_t length)
    {
        struct canfd_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = can_id & CAN_SFF_MASK;
        frame.flags = CANFD_BRS;  // BRS 플래그는 flags 필드에 설정
        frame.len = length;       // 실제 데이터 길이

        // 데이터 복사
        memcpy(frame.data, data, length);

        RCLCPP_INFO(this->get_logger(), "Sending CAN frame - ID: 0x%X, Len: %d", frame.can_id, frame.len);

        // 프레임 전송
        ssize_t nbytes = write(can_socket_, &frame, sizeof(struct canfd_frame));
        if (nbytes != sizeof(struct canfd_frame)) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to send CAN frame. Error: %s (errno: %d)",
                strerror(errno), errno);
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully sent CAN frame");
        }
    }

    void actuator_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        uint8_t data[9] = {0};

        uint8_t pwm_values[4];


        for(int i = 0; i < 4; i++) {
            int pwm_val = 0;
            switch(i) {
                case 0: pwm_val = msg->pwm0; break;
                case 1: pwm_val = msg->pwm1; break;
                case 2: pwm_val = msg->pwm2; break;
                case 3: pwm_val = msg->pwm3; break;
            }

        pwm_val = (pwm_val < 0) ? 0 : (pwm_val > 100) ? 100 : pwm_val;
        pwm_values[i] = static_cast<uint8_t>(pwm_val);
    }

        data[0] = pwm_values[0];  // PWM0
        data[1] = pwm_values[1];  // PWM1
        data[2] = pwm_values[2];  // PWM2
        data[3] = pwm_values[3];  // PWM3

        send_command(0x400, data, 9);

        RCLCPP_INFO(this->get_logger(), "Sent PWM command: %d, %d, %d, %d",
            pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3]);
    }

    void fan_callback(const wearable_robot_interfaces::msg::FanCommand::SharedPtr msg)
    {
        // CAN ID 0x400으로 FAN 명령 전송
        uint8_t data[64] = {0};

        // FAN 상태를 CAN 데이터에 삽입 (1바이트씩)
        data[4] = msg->fan0 ? 1 : 0;
        data[5] = msg->fan1 ? 1 : 0;
        data[6] = msg->fan2 ? 1 : 0;
        data[7] = msg->fan3 ? 1 : 0;

        send_command(0x400, data, 64);

        RCLCPP_INFO(this->get_logger(), "Sent FAN command: %d, %d, %d, %d",
            msg->fan0, msg->fan1, msg->fan2, msg->fan3);
    }


    // void motion_callback(const wearable_robot_interfaces::msg::IntensionData::SharedPtr msg)
    // {
    //     // motion 값에 따른 PWM 설정
    //     uint8_t data[64] = {0};

    //     // 기능 비활성화
    //     // uint16_t pwm_value = 0;

    //     // switch(msg->intention_id) {
    //     //     case 1: // PWM1, 3번 채널 50%
    //     //         memcpy(&data[2], &pwm_value, sizeof(uint16_t));  // PWM1 위치
    //     //         break;
    //     //     case 7: // PWM0, 2번 채널 50%
    //     //         memcpy(&data[0], &pwm_value, sizeof(uint16_t));  // PWM0 위치
    //     //         break;
    //     //     default:
    //     //         RCLCPP_WARN(this->get_logger(), "Unknown motion command: %d", msg->intention_id);
    //     //         return;
    //     // }

    //     send_command(0x400, data, 64);
    // }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANCommandTransmitNode>());
    rclcpp::shutdown();
    return 0;
}
