#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <string>
#include <vector>
#include <cstdlib>  // std::system()
#include <sstream>  // std::stringstream
#include <iomanip>  // std::setfill, std::setw, std::hex

class CANTransmitterNode : public rclcpp::Node
{
public:
    CANTransmitterNode() : Node("can_transmitter_node")
    {
        // PWM 명령 구독
        pwm_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&CANTransmitterNode::pwm_command_callback, this, std::placeholders::_1));

        // CAN 인터페이스 파라미터
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("can_id", "400");  // 기본 ID 0x400 (1024)

        RCLCPP_INFO(this->get_logger(), "CAN 송신 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "CAN 인터페이스: %s, CAN ID: 0x%s",
            this->get_parameter("can_interface").as_string().c_str(),
            this->get_parameter("can_id").as_string().c_str());
    }

private:
    // PWM 명령 수신 및 CAN 전송 콜백
    void pwm_command_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        // CAN 데이터 생성 (64바이트)
        std::vector<uint8_t> data(64, 0);

        // PWM 값을 CAN 데이터에 복사
        for (size_t i = 0; i < msg->pwm.size() && i < 6; ++i) {
            data[i] = msg->pwm[i];
        }

        // CAN 메시지 전송
        send_can_message(data);
    }

    // CAN 메시지 전송 함수
    void send_can_message(const std::vector<uint8_t>& data)
    {
        std::string can_interface = this->get_parameter("can_interface").as_string();
        std::string can_id = this->get_parameter("can_id").as_string();

        // cansend 명령을 위한 데이터 문자열 생성
        std::stringstream ss;
        ss << "cansend " << can_interface << " " << can_id << "##0";

        // 64바이트 데이터를 16진수 문자열로 변환
        for (uint8_t byte : data) {
            ss << std::setfill('0') << std::setw(2) << std::hex
               << static_cast<int>(byte);
        }

        // 명령 실행
        std::string cmd = ss.str();
        int result = std::system(cmd.c_str());

        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(),
                "CAN 메시지 전송 실패: %s (오류 코드: %d)",
                cmd.c_str(), result);
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "CAN 메시지 전송 성공: %s", cmd.c_str());
        }
    }

    // 구독
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANTransmitterNode>());
    rclcpp::shutdown();
    return 0;
}
