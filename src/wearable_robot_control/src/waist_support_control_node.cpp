#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/displacement_data.hpp>
#include <wearable_robot_interfaces/msg/back_intension.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>
#include <array>
#include <mutex>

class WaistAssistControlNode : public rclcpp::Node
{
public:
    // 상수 정의
    static constexpr int NUM_ACTUATORS = 2;  // 허리 보조용 구동기 수 (4번, 5번)
    static constexpr int DEFAULT_ACTUATOR_INDICES[NUM_ACTUATORS] = {4, 5};  // 기본 구동기 인덱스

    WaistAssistControlNode() : Node("waist_support_control_node")
    {
        // 온도 데이터 구독
        temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&WaistAssistControlNode::temperature_callback, this, std::placeholders::_1));

        // 변위 센서 데이터 구독 (허리 각도 측정용)
        displacement_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementData>(
            "displacement_data", 10,
            std::bind(&WaistAssistControlNode::displacement_callback, this, std::placeholders::_1));

        // 사용자 의도 데이터 구독 (GUI에서 전송)
        intention_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::BackIntension>(
            "intention_data", 10,
            std::bind(&WaistAssistControlNode::intention_callback, this, std::placeholders::_1));

        // 제어 모드 구독 (자동/수동 모드 전환)
        control_mode_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "control_mode", 10,
            std::bind(&WaistAssistControlNode::control_mode_callback, this, std::placeholders::_1));

        // 액추에이터 명령 발행
        pwm_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // 목표 온도 구독
        target_temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "target_temperature", 10,
            std::bind(&WaistAssistControlNode::target_temp_callback, this, std::placeholders::_1));

        // 직접 PWM 값 구독
        direct_pwm_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "direct_pwm_command", 10,
            std::bind(&WaistAssistControlNode::direct_pwm_callback, this, std::placeholders::_1));

        // 비상 정지 구독
        emergency_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&WaistAssistControlNode::emergency_callback, this, std::placeholders::_1));

        // ----- 파라미터 선언 -----
        // 구동기 및 센서 인덱스 파라미터
        this->declare_parameter("actuator_indices", std::vector<int64_t>{4, 5});  // 기본값: 4, 5번 구동기
        this->declare_parameter("waist_sensor_index", 0);  // 허리 변위 센서 인덱스 (기본값: 0번)

        // 허리 보조 제어 파라미터
        this->declare_parameter("waist_assist_active", false);  // 허리 보조 동작 활성화 여부
        this->declare_parameter("waist_angle_threshold", 30.0);  // 허리 각도 임계값 (기본값: 30도)
        this->declare_parameter("max_assist_power", 80);  // 최대 보조력 PWM 값 (0-100)

        // 온도 제어 파라미터
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            std::string idx_str = std::to_string(i);
            this->declare_parameter("target_temperature_" + idx_str, 50.0);  // 목표 온도
            this->declare_parameter("kp_" + idx_str, 2.0);  // 비례 게인
            this->declare_parameter("ki_" + idx_str, 0.1);  // 적분 게인
        }

        this->declare_parameter("max_pwm", 100.0);  // 최대 PWM
        this->declare_parameter("min_pwm", 0.0);    // 최소 PWM
        this->declare_parameter("safety_threshold", 80.0);  // 안전 온도 임계값 (섭씨)
        this->declare_parameter("control_frequency", 250.0);  // 제어 주파수 (Hz)

        // 파라미터 변경 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WaistAssistControlNode::parameter_callback, this, std::placeholders::_1));

        // PI 게인 구독
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            std::string idx_str = std::to_string(i);
            kp_param_subscriptions_[i] = this->create_subscription<std_msgs::msg::Float64>(
                "kp_param_" + idx_str, 10,
                [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                    this->gain_param_callback(msg, "kp_" + std::to_string(i));
                });

            ki_param_subscriptions_[i] = this->create_subscription<std_msgs::msg::Float64>(
                "ki_param_" + idx_str, 10,
                [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                    this->gain_param_callback(msg, "ki_" + std::to_string(i));
                });
        }

        // 제어 타이머 (250Hz로 제어)
        double control_frequency = this->get_parameter("control_frequency").as_double();
        int period_ms = static_cast<int>(1000.0 / control_frequency);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&WaistAssistControlNode::control_callback, this));

        // ----- 내부 변수 초기화 -----
        // 구동기 인덱스 초기화
        std::vector<int64_t> actuator_indices = this->get_parameter("actuator_indices").as_integer_array();
        for (int i = 0; i < NUM_ACTUATORS && i < static_cast<int>(actuator_indices.size()); i++) {
            actuator_indices_[i] = static_cast<int>(actuator_indices[i]);
        }

        waist_sensor_index_ = this->get_parameter("waist_sensor_index").as_int();

        // 제어 변수 초기화
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            current_temps_[i] = 0.0;
            integrals_[i] = 0.0;
        }

        waist_angle_ = 0.0;
        assist_intention_ = false;
        is_emergency_stop_ = false;
        is_temperature_safe_ = true;
        use_direct_pwm_ = true;  // 기본적으로 수동 제어 모드 사용

        // PWM 값 배열 초기화 (전체 6개 채널)
        current_pwm_values_.resize(6, 0);

        // 초기화 정보 출력
        RCLCPP_INFO(this->get_logger(), "허리 보조 제어 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "제어 구동기: %d번, %d번", actuator_indices_[0], actuator_indices_[1]);
        RCLCPP_INFO(this->get_logger(), "허리 변위 센서: %d번", waist_sensor_index_);
        RCLCPP_INFO(this->get_logger(), "안전 온도 임계값: %.1f°C", this->get_parameter("safety_threshold").as_double());
        RCLCPP_INFO(this->get_logger(), "제어 주파수: %.1fHz", control_frequency);
        RCLCPP_INFO(this->get_logger(), "초기 제어 모드: %s", use_direct_pwm_ ? "수동 제어" : "자동 제어");

        // 초기 PWM 상태 발행
        publish_pwm_command();
    }

private:
    // 온도 데이터 수신 콜백
    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 각 구동기의 온도 데이터 업데이트
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            if (msg->temperature.size() > static_cast<size_t>(actuator_indices_[i])) {
                current_temps_[i] = msg->temperature[actuator_indices_[i]];
                RCLCPP_DEBUG(this->get_logger(), "구동기 %d의 현재 온도: %.2f°C",
                    actuator_indices_[i], current_temps_[i]);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "구동기 %d의 온도 데이터가 없습니다", actuator_indices_[i]);
            }
        }

        // 온도 안전성 검사
        check_temperature_safety();
    }

    // 변위 센서 데이터 수신 콜백
    void displacement_callback(const wearable_robot_interfaces::msg::DisplacementData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 허리 각도 센서 데이터 확인
        if (msg->displacement.size() > static_cast<size_t>(waist_sensor_index_)) {
            waist_angle_ = msg->displacement[waist_sensor_index_];
            RCLCPP_DEBUG(this->get_logger(), "허리 각도: %.2f", waist_angle_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "허리 변위 센서(%d번) 데이터가 없습니다", waist_sensor_index_);
        }
    }

    // 사용자 의도 데이터 수신 콜백
    void intention_callback(const wearable_robot_interfaces::msg::BackIntension::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 의도 ID에 따른 처리 (허리 보조 의도만 수신)
        // 허리 굽힘/폄 의도 ID는 3과 4로 가정
        if (msg->intention_id == 3 || msg->intention_id == 4) {
            bool previous_intention = assist_intention_;
            assist_intention_ = true;

            if (!previous_intention) {
                RCLCPP_INFO(this->get_logger(), "허리 보조 의도가 감지되었습니다. 보조 모드 활성화");
            }
        } else {
            bool previous_intention = assist_intention_;
            assist_intention_ = false;

            if (previous_intention) {
                RCLCPP_INFO(this->get_logger(), "허리 보조 의도가 해제되었습니다. 보조 모드 비활성화");
            }
        }
    }

    // 직접 PWM 명령 수신 콜백
    void direct_pwm_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중에는 명령 무시
        if (is_emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "비상 정지 중입니다. PWM 명령이 무시됩니다.");
            return;
        }

        // 자동 모드일 때는 명령 무시
        if (!use_direct_pwm_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "자동 제어 모드에서 직접 PWM 명령이 수신되었습니다. 자동 모드에서는 수동 제어가 무시됩니다.");
            return;
        }

        // 각 구동기의 PWM 값 설정
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            int idx = actuator_indices_[i];
            if (msg->pwm.size() > static_cast<size_t>(idx)) {
                current_pwm_values_[idx] = msg->pwm[idx];
                RCLCPP_DEBUG(this->get_logger(),
                    "직접 PWM 제어 명령 수신: 구동기 %d, PWM 값: %d",
                    idx, current_pwm_values_[idx]);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "유효하지 않은 PWM 명령: 구동기 %d의 데이터가 없습니다", idx);
            }
        }

        // PWM 명령 발행
        publish_pwm_command();
    }

    // 제어 모드 콜백 (자동/수동)
    void control_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // true: 자동 모드, false: 수동 모드
        bool auto_mode = msg->data;
        use_direct_pwm_ = !auto_mode;

        RCLCPP_INFO(this->get_logger(),
            "제어 모드가 %s로 변경되었습니다", auto_mode ? "자동 제어" : "수동 PWM 제어");

        // 모드 전환 시 적분 누적값 초기화
        if (auto_mode) {
            for (auto& integral : integrals_) {
                integral = 0.0;
            }
        }
    }

    // 온도 안전성 검사
    void check_temperature_safety()
    {
        double safety_threshold = this->get_parameter("safety_threshold").as_double();
        bool any_temperature_unsafe = false;

        // 모든 구동기의 온도 검사
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            if (current_temps_[i] >= safety_threshold) {
                any_temperature_unsafe = true;
                RCLCPP_ERROR(this->get_logger(),
                    "구동기 %d의 온도가 안전 임계값(%.1f°C)을 초과했습니다! 현재 온도: %.1f°C",
                    actuator_indices_[i], safety_threshold, current_temps_[i]);
            }
        }

        // 하나라도 온도가 임계값을 초과하면 비상 정지
        if (any_temperature_unsafe && is_temperature_safe_) {
            is_temperature_safe_ = false;
            activate_emergency_stop();
        }
    }

    // 목표 온도 설정 콜백
    void target_temp_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중에는 목표 온도 변경 무시
        if (is_emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "비상 정지 중입니다. 목표 온도 변경이 무시됩니다.");
            return;
        }

        // 각 구동기별 목표 온도 업데이트
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            int idx = actuator_indices_[i];
            if (msg->temperature.size() > static_cast<size_t>(idx)) {
                std::string param_name = "target_temperature_" + std::to_string(i);
                double target_temp = msg->temperature[idx];
                this->set_parameter(rclcpp::Parameter(param_name, target_temp));
                RCLCPP_INFO(this->get_logger(), "구동기 %d의 목표 온도가 %.1f°C로 업데이트되었습니다",
                    idx, target_temp);
            }
        }
    }

    // 비상 정지 콜백
    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

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

                // 모든 온도가 안전하면 안전 상태 초기화
                bool all_temps_safe = true;
                double safety_threshold = this->get_parameter("safety_threshold").as_double();

                for (int i = 0; i < NUM_ACTUATORS; i++) {
                    if (current_temps_[i] >= safety_threshold) {
                        all_temps_safe = false;
                        break;
                    }
                }

                if (all_temps_safe) {
                    is_temperature_safe_ = true;
                }
            }
        }
    }

    // 게인 파라미터 업데이트 콜백
    void gain_param_callback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& param_name)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        this->set_parameter(rclcpp::Parameter(param_name, msg->data));
        RCLCPP_INFO(this->get_logger(), "%s 게인이 %.3f로 업데이트되었습니다", param_name.c_str(), msg->data);
    }

    // 비상 정지 활성화
    void activate_emergency_stop()
    {
        is_emergency_stop_ = true;
        RCLCPP_ERROR(this->get_logger(), "비상 정지가 활성화되었습니다! 모든 출력이 중단됩니다.");

        // PWM 출력을 0으로 설정하고 발행
        for (auto& value : current_pwm_values_) {
            value = 0;
        }
        publish_pwm_command();

        // 적분기 초기화
        for (auto& integral : integrals_) {
            integral = 0.0;
        }
    }

    // 주기적 제어 콜백
    void control_callback()
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중일 때는 제어 스킵, 0 값만 발행
        if (is_emergency_stop_) {
            for (auto& value : current_pwm_values_) {
                value = 0;
            }
            publish_pwm_command();
            return;
        }

        // 직접 PWM 제어 모드일 경우
        if (use_direct_pwm_) {
            // 현재 상태 발행 (이미 direct_pwm_callback에서 설정된 값 사용)
            publish_pwm_command();
            return;
        }

        // ----- 자동 제어 모드 -----
        // 1. 허리 보조 논리 구현
        bool waist_assist_active = this->get_parameter("waist_assist_active").as_bool();
        double waist_angle_threshold = this->get_parameter("waist_angle_threshold").as_double();
        uint8_t max_assist_power = static_cast<uint8_t>(this->get_parameter("max_assist_power").as_int());

        // 기본적으로 모든 출력 0으로 초기화
        for (int i = 0; i < NUM_ACTUATORS; i++) {
            int idx = actuator_indices_[i];
            current_pwm_values_[idx] = 0;
        }

        // 허리 보조 논리: (의도 있음 && 각도 임계값 초과) => 보조력 제공
        if (waist_assist_active && assist_intention_ && waist_angle_ > waist_angle_threshold) {
            // 허리 각도에 비례하는 출력값 계산 (간단한 선형 비례)
            double angle_ratio = std::min(1.0, (waist_angle_ - waist_angle_threshold) / 30.0);  // 임계값 초과분을 30도로 정규화
            uint8_t assist_power = static_cast<uint8_t>(angle_ratio * max_assist_power);

            // 두 개의 구동기에 동일한 출력 적용 (실제로는 더 복잡한 로직일 수 있음)
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                int idx = actuator_indices_[i];
                current_pwm_values_[idx] = assist_power;
            }

            RCLCPP_DEBUG(this->get_logger(),
                "허리 보조 활성화: 각도=%.1f° (임계값=%.1f°), 출력=%d (최대=%d)",
                waist_angle_, waist_angle_threshold, assist_power, max_assist_power);
        }

        // 2. 온도 관리 로직 (기존 PI 제어 적용)
        double control_frequency = this->get_parameter("control_frequency").as_double();
        double dt = 1.0 / control_frequency;
        double max_pwm = this->get_parameter("max_pwm").as_double();
        double min_pwm = this->get_parameter("min_pwm").as_double();

        for (int i = 0; i < NUM_ACTUATORS; i++) {
            std::string idx_str = std::to_string(i);
            double target_temp = this->get_parameter("target_temperature_" + idx_str).as_double();
            double kp = this->get_parameter("kp_" + idx_str).as_double();
            double ki = this->get_parameter("ki_" + idx_str).as_double();

            // 현재 온도가 목표 온도를 초과하면 온도 제어 로직 적용
            if (current_temps_[i] > target_temp) {
                // PI 제어 연산
                double error = target_temp - current_temps_[i];  // 음수값 (현재 온도가 목표보다 높음)
                integrals_[i] += error * dt;

                // Anti-windup
                if (integrals_[i] > max_pwm / ki) integrals_[i] = max_pwm / ki;
                if (integrals_[i] < min_pwm / ki) integrals_[i] = min_pwm / ki;

                // PI 출력 계산 (음수 오차로 인해 출력값이 줄어듦)
                double output = kp * error + ki * integrals_[i];

                // 출력 제한
                if (output > max_pwm) output = max_pwm;
                if (output < min_pwm) output = min_pwm;

                // 정수화 및 구동기에 적용
                int idx = actuator_indices_[i];
                uint8_t pwm_value = static_cast<uint8_t>(output);

                // 온도 제어는 허리 보조력보다 우선 적용 (안전을 위함)
                // 온도 제어에 의한 값이 더 작을 경우만 적용 (출력 감소 방향으로만 영향)
                if (pwm_value < current_pwm_values_[idx]) {
                    current_pwm_values_[idx] = pwm_value;
                    RCLCPP_DEBUG(this->get_logger(),
                        "구동기 %d 온도 제어: 목표=%.1f°C, 현재=%.1f°C, 오차=%.1f°C, 출력=%d",
                        idx, target_temp, current_temps_[i], error, pwm_value);
                }
            }
        }

        // 명령 발행
        publish_pwm_command();
    }

    // PWM 명령 발행 함수
    void publish_pwm_command()
    {
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = this->now();
        pwm_msg.pwm = current_pwm_values_;

        // 메시지 발행
        pwm_publisher_->publish(pwm_msg);
    }

    // 파라미터 설정 콜백
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            const std::string& name = param.get_name();

            if (name == "waist_assist_active") {
                RCLCPP_INFO(this->get_logger(),
                    "허리 보조 기능이 %s되었습니다", param.as_bool() ? "활성화" : "비활성화");
            }
            else if (name == "waist_angle_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "허리 각도 임계값이 %.1f°로 변경되었습니다", param.as_double());
            }
            else if (name == "max_assist_power") {
                RCLCPP_INFO(this->get_logger(),
                    "최대 보조력이 %d로 변경되었습니다", param.as_int());
            }
            else if (name == "waist_sensor_index") {
                waist_sensor_index_ = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                    "허리 변위 센서 인덱스가 %d번으로 변경되었습니다", waist_sensor_index_);
            }
            else if (name == "actuator_indices") {
                std::vector<int64_t> indices = param.as_integer_array();
                if (indices.size() >= NUM_ACTUATORS) {
                    for (int i = 0; i < NUM_ACTUATORS; i++) {
                        actuator_indices_[i] = static_cast<int>(indices[i]);
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "제어 구동기가 %d번, %d번으로 변경되었습니다",
                        actuator_indices_[0], actuator_indices_[1]);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "유효하지 않은 구동기 인덱스 목록: 최소 %d개 필요", NUM_ACTUATORS);
                    result.successful = false;
                }
            }
            else if (name.find("target_temperature_") == 0 ||
                     name.find("kp_") == 0 ||
                     name.find("ki_") == 0) {
                RCLCPP_INFO(this->get_logger(), "%s가 %.3f로 변경되었습니다",
                    name.c_str(), param.as_double());
            }
            else if (name == "safety_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "안전 온도 임계값이 %.1f°C로 변경되었습니다", param.as_double());
            }
        }

        return result;
    }

    // ----- 멤버 변수 -----
    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementData>::SharedPtr displacement_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::BackIntension>::SharedPtr intention_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr target_temp_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_mode_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr direct_pwm_subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_publisher_;

    // PI 게인 파라미터 구독
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, NUM_ACTUATORS> kp_param_subscriptions_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, NUM_ACTUATORS> ki_param_subscriptions_;

    // 파라미터 콜백
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 타이머
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 구동기 및 센서 인덱스
    std::array<int, NUM_ACTUATORS> actuator_indices_;
    int waist_sensor_index_;

    // 제어 상태 변수
    std::array<double, NUM_ACTUATORS> current_temps_;  // 각 구동기의 현재 온도
    std::array<double, NUM_ACTUATORS> integrals_;      // 각 구동기의 적분 누적값
    double waist_angle_;                               // 허리 각도
    bool assist_intention_;                            // 사용자의 보조 의도
    bool use_direct_pwm_;                              // 직접 PWM 제어 모드 사용 여부
    std::vector<uint8_t> current_pwm_values_;          // 현재 PWM 값 배열
    std::mutex control_mutex_;                         // 제어 로직 동기화를 위한 뮤텍스

    // 안전 관련 변수
    bool is_emergency_stop_;                           // 비상 정지 상태
    bool is_temperature_safe_;                         // 온도 안전 상태
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaistAssistControlNode>());
    rclcpp::shutdown();
    return 0;
}
