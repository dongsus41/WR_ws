#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <ros2_socketcan_msgs/msg/fd_frame.hpp>
#include <string>
#include <vector>

class ActuatorTempControlNode : public rclcpp::Node
{
public:
    ActuatorTempControlNode() : Node("actuator_temp_control_node")
    {
        // 온도 데이터 구독
        temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&ActuatorTempControlNode::temperature_callback, this, std::placeholders::_1));

        // CAN 명령 발행
        can_publisher_ = this->create_publisher<ros2_socketcan_msgs::msg::FdFrame>(
            "to_can_bus_fd", 10);

        // PWM 상태 발행 (모니터링용)
        pwm_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // PI 제어 파라미터 및 목표 온도 선언
        this->declare_parameter("target_temperature", 50.0);  // 기본값 50도
        this->declare_parameter("kp", 2.0);                  // 비례 게인
        this->declare_parameter("ki", 0.1);                  // 적분 게인
        this->declare_parameter("max_pwm", 100.0);           // 최대 PWM
        this->declare_parameter("min_pwm", 0.0);             // 최소 PWM
        this->declare_parameter("active_actuator", 5);       // 제어할 구동기 번호

        // 파라미터 변경 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ActuatorTempControlNode::parameter_callback, this, std::placeholders::_1));

        // 제어 타이머 (10Hz로 제어)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ActuatorTempControlNode::control_callback, this));

        // 내부 변수 초기화
        current_temp_ = 0.0;
        previous_error_ = 0.0;
        integral_ = 0.0;
        pwm_output_ = 0;

        RCLCPP_INFO(this->get_logger(), "Temperature Control Node has been started");
    }

private:
    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        size_t actuator_idx = static_cast<size_t>(this->get_parameter("active_actuator").as_int());

        // 온도 센서 데이터 확인
        if (msg->temperature.size() > actuator_idx) {
            current_temp_ = msg->temperature[actuator_idx];
            RCLCPP_DEBUG(this->get_logger(), "Current temperature of actuator %zu: %.2f",
                actuator_idx, current_temp_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Temperature data missing for actuator %zu", actuator_idx);
        }
    }

    void control_callback()
    {
        // 파라미터 불러오기
        double target_temp = this->get_parameter("target_temperature").as_double();
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double max_pwm = this->get_parameter("max_pwm").as_double();
        double min_pwm = this->get_parameter("min_pwm").as_double();
        int actuator_idx = this->get_parameter("active_actuator").as_int();

        // PI 제어 연산
        double error = target_temp - current_temp_;
        integral_ += error * 0.1;  // dt = 0.1s (10Hz)

        // Anti-windup
        if (integral_ > max_pwm) integral_ = max_pwm;
        if (integral_ < min_pwm) integral_ = min_pwm;

        // PI 출력 계산
        double output = kp * error + ki * integral_;

        // 출력 제한
        if (output > max_pwm) output = max_pwm;
        if (output < min_pwm) output = min_pwm;

        // 정수화
        pwm_output_ = static_cast<uint8_t>(output);

        // CAN 메시지 생성 및 발행
        send_pwm_command(pwm_output_, actuator_idx);

        // 디버그 정보
        RCLCPP_DEBUG(this->get_logger(),
            "Control: Target=%.1f, Current=%.1f, Error=%.1f, Output=%d",
            target_temp, current_temp_, error, pwm_output_);
    }

    void send_pwm_command(uint8_t pwm_value, int actuator_idx)
    {
        // PWM 명령 메시지 생성
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = this->now();
        pwm_msg.pwm.resize(6, 0);  // 6개 채널 모두 0으로 초기화
        pwm_msg.pwm[actuator_idx] = pwm_value;  // 지정 채널만 값 설정

        // 모니터링용 발행
        pwm_publisher_->publish(pwm_msg);

        // CAN 메시지 생성
        auto can_msg = ros2_socketcan_msgs::msg::FdFrame();
        can_msg.header.stamp = this->now();
        can_msg.id = 0x400;  // board 명령용 CAN ID 400
        can_msg.is_extended = false;
        can_msg.is_error = false;
        can_msg.len = 64;  // CAN FD 프레임 - 64바이트

        // 데이터 채우기
        can_msg.data.resize(64, 0);  // 64 바이트 초기화
        for (int i = 0; i < 6; i++) {
            can_msg.data[i] = (i == actuator_idx) ? pwm_value : 0;
        }

        // 다음 6바이트는 팬 상태 (현재는 모두 0으로 설정)
        // 필요시 팬 상태 설정 로직 추가

        // CAN 메시지 발행
        can_publisher_->publish(can_msg);

        RCLCPP_DEBUG(this->get_logger(),
        "Sent CAN command - ID: 0x400, PWM[%d]: %d",
        actuator_idx, pwm_value);
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "target_temperature") {
                RCLCPP_INFO(this->get_logger(),
                    "Target temperature changed to: %.1f", param.as_double());
            }
        }

        return result;
    }

    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_subscription_;
    rclcpp::Publisher<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr can_publisher_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_publisher_;

    // 파라미터 콜백
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 타이머
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 제어 변수
    double current_temp_;
    double previous_error_;
    double integral_;
    uint8_t pwm_output_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorTempControlNode>());
    rclcpp::shutdown();
    return 0;
}
