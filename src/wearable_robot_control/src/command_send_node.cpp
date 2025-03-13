#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <wearable_robot_interfaces/msg/can_data_frame.hpp>
#include <mutex>
#include <memory>

using namespace std::chrono_literals;

class CommandIntegratorNode : public rclcpp::Node
{
public:
    CommandIntegratorNode() : Node("command_integrator_node")
    {
        // 파라미터 선언
        this->declare_parameter("publish_rate_ms", 10);  // 기본값 10ms (100Hz)

        int publish_rate = this->get_parameter("publish_rate_ms").as_int();

        // 통합된 명령을 12바이트로 초기화 (0~5: 구동기, 6~11: 팬)
        integrated_command_ = std::make_shared<wearable_robot_interfaces::msg::ActuatorCommand>();
        integrated_command_->pwm.resize(12, 0);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "명령 통합 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "발행 주기: %d ms", publish_rate);

        // 구독자 생성 - 하위 제어기로부터 구동기 명령 수신
        // 메시지 타입은 하위 제어기의 출력에 맞게 조정
        actuator_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "actuator_pwm_commands", 10,
            std::bind(&CommandIntegratorNode::actuator_command_callback, this, std::placeholders::_1));

        // 구독자 생성 - 팬 제어 명령 수신
        fan_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "fan_control_commands", 10,
            std::bind(&CommandIntegratorNode::fan_command_callback, this, std::placeholders::_1));

        // 퍼블리셔 생성 - CAN 송신 노드로 통합된 명령 발행
        command_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // 발행 타이머 생성 (주기적인 명령 발행)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate),
            std::bind(&CommandIntegratorNode::publish_integrated_command, this));

        // 명령 수신 상태 확인용 타이머 (선택사항)
        status_timer_ = this->create_wall_timer(
            1s,  // 1초마다 상태 출력
            std::bind(&CommandIntegratorNode::check_command_status, this));
    }

private:
    // 구동기 명령 수신 콜백
    void actuator_command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        // 로그 출력 (디버그 레벨)
        RCLCPP_DEBUG(this->get_logger(), "구동기 명령 수신: 크기 %zu", msg->data.size());

        // 구동기 명령을 통합 명령의 0~5번 인덱스에 복사
        for (size_t i = 0; i < msg->data.size() && i < 6; ++i) {
            integrated_command_->pwm[i] = msg->data[i];
        }

        // 마지막 구동기 명령 수신 시간 갱신
        last_actuator_command_ = this->now();
    }

    // 팬 제어 명령 수신 콜백
    void fan_command_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        // 로그 출력 (디버그 레벨)
        RCLCPP_DEBUG(this->get_logger(), "팬 제어 명령 수신: 크기 %zu", msg->data.size());

        // 팬 제어 명령을 통합 명령의 6~11번 인덱스에 복사
        for (size_t i = 0; i < msg->data.size() && i < 6; ++i) {
            integrated_command_->pwm[i + 6] = msg->data[i];
        }

        // 마지막 팬 제어 명령 수신 시간 갱신
        last_fan_command_ = this->now();
    }

    // 통합된 명령 발행 함수
    void publish_integrated_command()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        // 명령 발행
        command_publisher_->publish(*integrated_command_);

        // 로그 출력 (디버그 레벨, 너무 빈번한 로그 방지)
        if (debug_counter_++ % 100 == 0) {  // 100번에 한 번만 로그 출력
            RCLCPP_DEBUG(this->get_logger(), "통합 명령 발행");
        }
    }

    // 명령 수신 상태 확인 함수
    void check_command_status()
    {
        auto now = this->now();
        auto actuator_elapsed = now - last_actuator_command_;
        auto fan_elapsed = now - last_fan_command_;

        // 1초 이상 명령이 수신되지 않으면 경고
        if (actuator_elapsed > rclcpp::Duration(1, 0) && last_actuator_command_ != rclcpp::Time(0, 0)) {
            RCLCPP_WARN(this->get_logger(),
                "구동기 명령이 %.2f초 동안 수신되지 않았습니다",
                actuator_elapsed.seconds());
        }

        if (fan_elapsed > rclcpp::Duration(1, 0) && last_fan_command_ != rclcpp::Time(0, 0)) {
            RCLCPP_WARN(this->get_logger(),
                "팬 제어 명령이 %.2f초 동안 수신되지 않았습니다",
                fan_elapsed.seconds());
        }
    }

    // 구독자
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr actuator_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr fan_subscription_;

    // 퍼블리셔
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr command_publisher_;

    // 타이머
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 데이터
    std::shared_ptr<wearable_robot_interfaces::msg::ActuatorCommand> integrated_command_;
    std::mutex command_mutex_;  // 스레드 안전성 보장을 위한 뮤텍스

    // 상태 추적
    rclcpp::Time last_actuator_command_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_fan_command_{0, 0, RCL_ROS_TIME};
    int debug_counter_ = 0;  // 디버그 로그 제한용
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandIntegratorNode>());
    rclcpp::shutdown();
    return 0;
}
