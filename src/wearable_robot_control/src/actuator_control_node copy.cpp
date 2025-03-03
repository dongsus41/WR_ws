#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>
#include <cstdlib>  // std::system()
#include <sstream>  // std::stringstream
#include <iomanip>  // std::setfill, std::setw, std::hex

class ActuatorTempControlNode : public rclcpp::Node
{
public:
    ActuatorTempControlNode() : Node("actuator_temp_control_node")
    {
        // 온도 데이터 구독
        temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&ActuatorTempControlNode::temperature_callback, this, std::placeholders::_1));

        // PWM 상태 발행 (모니터링용)
        pwm_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // 목표 온도 구독 (GUI에서 오는 명령)
        target_temp_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_temperature", 10,
            std::bind(&ActuatorTempControlNode::target_temp_callback, this, std::placeholders::_1));

        // 비상 정지 구독
        emergency_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&ActuatorTempControlNode::emergency_callback, this, std::placeholders::_1));

        // PI 제어 파라미터 및 목표 온도 선언
        this->declare_parameter("target_temperature", 50.0);  // 기본값 50도
        this->declare_parameter("kp", 2.0);                  // 비례 게인
        this->declare_parameter("ki", 0.1);                  // 적분 게인
        this->declare_parameter("max_pwm", 100.0);           // 최대 PWM
        this->declare_parameter("min_pwm", 0.0);             // 최소 PWM
        this->declare_parameter("active_actuator", 5);       // 제어할 구동기 번호
        this->declare_parameter("can_interface", "can0");    // CAN 인터페이스
        this->declare_parameter("safety_threshold", 80.0);   // 안전 온도 임계값 (섭씨)

        // 파라미터 변경 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ActuatorTempControlNode::parameter_callback, this, std::placeholders::_1));

        // PI 게인 파라미터 구독
        kp_param_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "kp_param", 10,
            std::bind(&ActuatorTempControlNode::kp_param_callback, this, std::placeholders::_1));

        ki_param_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "ki_param", 10,
            std::bind(&ActuatorTempControlNode::ki_param_callback, this, std::placeholders::_1));

        // 제어 타이머 (250Hz로 제어)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),  // 4ms (250Hz)
            std::bind(&ActuatorTempControlNode::control_callback, this));

        // 내부 변수 초기화
        current_temp_ = 0.0;
        previous_error_ = 0.0;
        integral_ = 0.0;
        pwm_output_ = 0;
        is_emergency_stop_ = false;
        is_temperature_safe_ = true;

        RCLCPP_INFO(this->get_logger(), "Temperature Control Node (using cansend) has been started");
        RCLCPP_INFO(this->get_logger(), "안전 온도 임계값: %.1f°C", this->get_parameter("safety_threshold").as_double());
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

            // 온도 안전성 검사
            check_temperature_safety();
        } else {
            RCLCPP_WARN(this->get_logger(), "Temperature data missing for actuator %zu", actuator_idx);
        }
    }

    void check_temperature_safety()
    {
        double safety_threshold = this->get_parameter("safety_threshold").as_double();

        // 온도가 임계값을 초과하면 비상 정지
        if (current_temp_ >= safety_threshold && is_temperature_safe_) {
            is_temperature_safe_ = false;
            RCLCPP_ERROR(this->get_logger(),
                "온도가 안전 임계값(%.1f°C)을 초과했습니다! 현재 온도: %.1f°C",
                safety_threshold, current_temp_);

            // 비상 정지 활성화
            activate_emergency_stop();
        }
    }

    void target_temp_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 비상 정지 중에는 목표 온도 변경 무시
        if (is_emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "비상 정지 중입니다. 목표 온도 변경이 무시됩니다.");
            return;
        }

        // GUI에서 온 목표 온도 업데이트
        this->set_parameter(rclcpp::Parameter("target_temperature", msg->data));
        RCLCPP_INFO(this->get_logger(), "Target temperature updated to: %.1f°C", msg->data);
    }

    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // 비상 정지 활성화
            if (!is_emergency_stop_) {
                activate_emergency_stop();
            }
        } else {
            // 비상 정지 해제
            if (is_emergency_stop_) {
                is_emergency_stop_ = false;
                RCLCPP_INFO(this->get_logger(), "비상 정지가 해제되었습니다. 제어 재개");

                // 온도가 안전하면 안전 상태도 초기화
                if (current_temp_ < this->get_parameter("safety_threshold").as_double()) {
                    is_temperature_safe_ = true;
                }
            }
        }
    }

    void activate_emergency_stop()
    {
        is_emergency_stop_ = true;
        RCLCPP_ERROR(this->get_logger(), "비상 정지가 활성화되었습니다! 모든 출력이 중단됩니다.");

        // PWM 출력을 0으로 설정하고 즉시 전송
        send_pwm_command_via_cansend(0, this->get_parameter("active_actuator").as_int());

        // 적분기 초기화
        integral_ = 0.0;
    }

    void kp_param_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 비상 정지 중에는 게인 변경 허용 (제어에는 영향 없음)
        this->set_parameter(rclcpp::Parameter("kp", msg->data));
        RCLCPP_INFO(this->get_logger(), "Kp gain updated to: %.2f", msg->data);
    }

    void ki_param_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 비상 정지 중에는 게인 변경 허용 (제어에는 영향 없음)
        this->set_parameter(rclcpp::Parameter("ki", msg->data));
        RCLCPP_INFO(this->get_logger(), "Ki gain updated to: %.3f", msg->data);
    }

    void control_callback()
    {
        // 비상 정지 중일 때는 제어 스킵
        if (is_emergency_stop_) {
            // 주기적으로 PWM 0 전송 (안전성 강화)
            send_pwm_command_via_cansend(0, this->get_parameter("active_actuator").as_int());
            return;
        }

        // 파라미터 불러오기
        double target_temp = this->get_parameter("target_temperature").as_double();
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double max_pwm = this->get_parameter("max_pwm").as_double();
        double min_pwm = this->get_parameter("min_pwm").as_double();
        int actuator_idx = this->get_parameter("active_actuator").as_int();

        // PI 제어 연산
        double error = target_temp - current_temp_;
        integral_ += error * 0.004;  // dt = 0.004s (250Hz)

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
        send_pwm_command_via_cansend(pwm_output_, actuator_idx);

        // 디버그 정보
        RCLCPP_DEBUG(this->get_logger(),
            "Control: Target=%.1f, Current=%.1f, Error=%.1f, Output=%d",
            target_temp, current_temp_, error, pwm_output_);
    }

    void send_pwm_command_via_cansend(uint8_t pwm_value, int actuator_idx)
    {
        // PWM 명령 메시지 생성 (모니터링용)
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = this->now();
        pwm_msg.pwm.resize(6, 0);  // 6개 채널 모두 0으로 초기화
        pwm_msg.pwm[actuator_idx] = pwm_value;  // 지정 채널만 값 설정

        // 모니터링용 발행
        pwm_publisher_->publish(pwm_msg);

        // CAN 데이터 생성
        std::vector<uint8_t> data(64, 0); // 64바이트 전체를 0으로 초기화

        // 구동기 PWM 값 설정 (바이트 0-5)
        data[actuator_idx] = pwm_value;

        // cansend 명령을 위한 데이터 문자열 생성
        std::stringstream ss;
        ss << "cansend " << this->get_parameter("can_interface").as_string()
           << " 400##0"; // CANFD ID 0x400 (1024)

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
                "Failed to send CAN command: %s (error code: %d)",
                cmd.c_str(), result);
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "Sent CAN command via cansend: %s", cmd.c_str());
        }
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
            } else if (param.get_name() == "kp") {
                RCLCPP_INFO(this->get_logger(),
                    "Kp gain changed to: %.2f", param.as_double());
            } else if (param.get_name() == "ki") {
                RCLCPP_INFO(this->get_logger(),
                    "Ki gain changed to: %.3f", param.as_double());
            } else if (param.get_name() == "safety_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "Safety temperature threshold changed to: %.1f", param.as_double());
            }
        }

        return result;
    }

    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_temp_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr kp_param_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ki_param_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription_;
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

    // 안전 관련 변수
    bool is_emergency_stop_;
    bool is_temperature_safe_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorTempControlNode>());
    rclcpp::shutdown();
    return 0;
}
