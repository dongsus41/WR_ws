#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/srv/set_control_params.hpp>
#include <wearable_robot_interfaces/srv/emergency_stop.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <set>

class WaistActuatorControlNode : public rclcpp::Node
{
public:
    // 상수 정의
    static constexpr int TOTAL_CHANNELS = 6;     // 총 채널 수
    static constexpr double DEFAULT_KP = 2.0;    // 기본 비례 게인
    static constexpr double DEFAULT_KI = 0.05;   // 기본 적분 게인

    // 액추에이터 상태 정보를 담는 구조체
    struct ActuatorState {
        double current_temp;     // 현재 온도
        double integral;         // 적분 누적값
        double kp;               // 비례 게인
        double ki;               // 적분 게인

        // 기본 생성자
        ActuatorState()
            : current_temp(0.0), integral(0.0),
              kp(DEFAULT_KP), ki(DEFAULT_KI) {}
    };

    WaistActuatorControlNode() : Node("waist_actuator_control_node")
    {
        // 온도 데이터 구독
        temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&WaistActuatorControlNode::temperature_callback, this, std::placeholders::_1));

        // 명령 발행
        pwm_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // auto mode: 목표 온도 구독
        target_temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "target_temperature", 10,
            std::bind(&WaistActuatorControlNode::target_temp_callback, this, std::placeholders::_1));

        // 서비스 설정
        setupServices();

        // 파라미터 설정
        setupParameters();

        // 내부 변수 초기화
        is_emergency_stop_ = false;
        is_temperature_safe_ = true;

        // PWM 값 배열 초기화
        current_pwm_values_.resize(TOTAL_CHANNELS, 0);

        // 초기화 로그 출력
        logInitInfo();

        // 초기 PWM 상태 발행
        publish_pwm_command();
    }

private:
    // 서비스 설정 함수
    void setupServices() {
        // PI 파라미터 설정 서비스
        pi_param_service_ = this->create_service<wearable_robot_interfaces::srv::SetControlParams>(
            "set_pi_parameters",
            std::bind(&WaistActuatorControlNode::handle_pi_parameters, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 비상 정지 서비스
        emergency_service_ = this->create_service<wearable_robot_interfaces::srv::EmergencyStop>(
            "set_emergency_stop",
            std::bind(&WaistActuatorControlNode::handle_emergency_stop, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    // 파라미터 설정 함수
    void setupParameters() {
        // 기본 파라미터 선언
        this->declare_parameter("target_temperature", 50.0);
        this->declare_parameter("max_pwm", 60.0);
        this->declare_parameter("min_pwm", 0.0);
        this->declare_parameter("safety_threshold", 80.0);
        this->declare_parameter("control_frequency", 250.0);
        this->declare_parameter("can_interface", "can0");

        // 구동기별 파라미터 선언
        this->declare_parameter("actuator_ids", std::vector<int64_t>{4, 5});

        // 구동기 초기화
        initializeActuators();

        // 제어 타이머 설정
        double control_frequency = this->get_parameter("control_frequency").as_double();
        int period_ms = static_cast<int>(1000.0 / control_frequency);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&WaistActuatorControlNode::control_callback, this));

        // 파라미터 변경 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WaistActuatorControlNode::parameter_callback, this, std::placeholders::_1));
    }

    // 구동기 초기화 함수
    void initializeActuators() {
        std::vector<int64_t> actuator_ids = this->get_parameter("actuator_ids").as_integer_array();
        for (const auto& id : actuator_ids) {
            if (id >= 0 && id < TOTAL_CHANNELS) {
                active_actuator_ids_.insert(static_cast<int>(id));

                // 구동기별 상태 객체 초기화
                actuator_states_[id] = ActuatorState();

                // YAML 구조에 맞게 파라미터 접근 경로 설정
                std::string actuator_prefix = "actuator" + std::to_string(id);
                std::string kp_param = actuator_prefix + ".kp";
                std::string ki_param = actuator_prefix + ".ki";

                // 파라미터가 있는지 확인
                if (this->has_parameter(kp_param)) {
                    actuator_states_[id].kp = this->get_parameter(kp_param).as_double();
                } else {
                    // 파라미터가 없으면 기본값 사용
                    this->declare_parameter(kp_param, DEFAULT_KP);
                    actuator_states_[id].kp = DEFAULT_KP;
                }

                if (this->has_parameter(ki_param)) {
                    actuator_states_[id].ki = this->get_parameter(ki_param).as_double();
                } else {
                    // 파라미터가 없으면 기본값 사용
                    this->declare_parameter(ki_param, DEFAULT_KI);
                    actuator_states_[id].ki = DEFAULT_KI;
                }

                RCLCPP_INFO(this->get_logger(),
                    "구동기 %d의 PI 게인 - Kp: %.2f, Ki: %.3f",
                    id, actuator_states_[id].kp, actuator_states_[id].ki);
            }
        }
    }

    // 초기화 정보 로그 출력
    void logInitInfo() {
        RCLCPP_INFO(this->get_logger(), "액추에이터 제어 노드가 시작되었습니다");

        std::string active_actuators = "활성화된 구동기: ";
        for (const auto& id : active_actuator_ids_) {
            active_actuators += std::to_string(id) + ", ";
        }

        if (!active_actuator_ids_.empty()) {
            active_actuators.pop_back();
            active_actuators.pop_back();
        }

        RCLCPP_INFO(this->get_logger(), "%s", active_actuators.c_str());
        RCLCPP_INFO(this->get_logger(), "CAN 인터페이스: %s", this->get_parameter("can_interface").as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "안전 온도 임계값: %.1f°C", this->get_parameter("safety_threshold").as_double());
        RCLCPP_INFO(this->get_logger(), "목표 온도: %.1f°C", this->get_parameter("target_temperature").as_double());
        RCLCPP_INFO(this->get_logger(), "최대 PWM: %.1f", this->get_parameter("max_pwm").as_double());
        RCLCPP_INFO(this->get_logger(), "제어 주파수: %.1fHz", this->get_parameter("control_frequency").as_double());
    }

    // 온도 데이터 수신 콜백
    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 각 액추에이터별 온도 데이터 확인 및 업데이트
        for (const auto& id : active_actuator_ids_) {
            if (msg->temperature.size() > static_cast<size_t>(id)) {
                actuator_states_[id].current_temp = msg->temperature[id];
                RCLCPP_DEBUG(this->get_logger(), "구동기 %d의 현재 온도: %.2f°C",
                    id, actuator_states_[id].current_temp);

                // 온도가 임계값을 초과하면 비상 정지
                check_temperature_safety(id);
            }
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

        // 목표 온도 설정
        if (!msg->temperature.empty()) {
            // 공통 목표 온도 업데이트
            double target_temp = msg->temperature[0];
            this->set_parameter(rclcpp::Parameter("target_temperature", target_temp));

            // 온도가 0에 가까우면 즉시 모든 PWM 출력을 0으로 설정
            if (target_temp <= 0.1) {
                RCLCPP_INFO(this->get_logger(), "목표 온도가 0°C로 설정되어 모든 구동기 출력을 0으로 설정합니다");

                // PWM 출력을 0으로 설정하고 발행
                std::fill(current_pwm_values_.begin(), current_pwm_values_.end(), 0);
                publish_pwm_command();
            } else {
                RCLCPP_INFO(this->get_logger(), "목표 온도가 %.1f°C로 설정되었습니다", target_temp);

                // 온도가 0이 아닌 경우 적분기 초기화 (급격한 변화 방지)
                for (auto& pair : actuator_states_) {
                    pair.second.integral = 0.0;
                }
            }
        }
    }

    // 구동기 온도 안전성 검사
    void check_temperature_safety(int actuator_id)
    {
        double safety_threshold = this->get_parameter("safety_threshold").as_double();
        double current_temp = actuator_states_[actuator_id].current_temp;

        // 온도가 임계값을 초과하면 비상 정지
        if (current_temp >= safety_threshold && is_temperature_safe_) {
            is_temperature_safe_ = false;
            RCLCPP_ERROR(this->get_logger(),
                "구동기 %d의 온도가 안전 임계값(%.1f°C)을 초과했습니다! 현재 온도: %.1f°C",
                actuator_id, safety_threshold, current_temp);

            // 비상 정지 활성화
            activate_emergency_stop();
        }
    }

    // 비상 정지 활성화
    void activate_emergency_stop()
    {
        is_emergency_stop_ = true;
        RCLCPP_ERROR(this->get_logger(), "비상 정지가 활성화되었습니다! 모든 출력이 중단됩니다.");

        // PWM 출력을 0으로 설정하고 발행
        std::fill(current_pwm_values_.begin(), current_pwm_values_.end(), 0);
        publish_pwm_command();

        // 모든 액추에이터의 적분기 초기화
        for (auto& pair : actuator_states_) {
            pair.second.integral = 0.0;
        }
    }

    // PI 제어 계산 함수
    uint8_t calculate_pi_control(int id, double target_temp, double dt, double max_pwm, double min_pwm)
    {
        ActuatorState& state = actuator_states_[id];

        // PI 제어 연산
        double error = target_temp - state.current_temp;
        state.integral += error * dt;

        // Anti-windup
        if (state.integral > max_pwm / state.ki) state.integral = max_pwm / state.ki;
        if (state.integral < min_pwm / state.ki) state.integral = min_pwm / state.ki;

        // PI 출력 계산
        double output = state.kp * error + state.ki * state.integral;

        // 출력 제한
        output = std::max(min_pwm, std::min(output, max_pwm));

        RCLCPP_DEBUG(this->get_logger(),
            "구동기 %d PI 계산: 오차=%.1f, 적분=%.1f, Kp=%.2f, Ki=%.3f, 출력=%.1f",
            id, error, state.integral, state.kp, state.ki, output);

        return static_cast<uint8_t>(output);
    }

    // 주기적 제어 콜백
    void control_callback()
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중일 때는 제어 스킵, 0 값만 발행
        if (is_emergency_stop_) {
            std::fill(current_pwm_values_.begin(), current_pwm_values_.end(), 0);
            publish_pwm_command();
            return;
        }

        double target_temp = this->get_parameter("target_temperature").as_double();

        // 목표 온도가 0에 가까우면 모든 출력을 0으로 설정
        if (target_temp <= 0.1) {
            std::fill(current_pwm_values_.begin(), current_pwm_values_.end(), 0);
            publish_pwm_command();
            return;
        }

        // 이하 기존 PI 제어 로직 (온도가 0보다 클 때만 실행)
        double max_pwm = this->get_parameter("max_pwm").as_double();
        double min_pwm = this->get_parameter("min_pwm").as_double();
        double control_frequency = this->get_parameter("control_frequency").as_double();
        double dt = 1.0 / control_frequency;

        // 각 액추에이터별 제어 로직 적용
        for (const auto& id : active_actuator_ids_) {
            uint8_t pwm_value = calculate_pi_control(id, target_temp, dt, max_pwm, min_pwm);
            current_pwm_values_[id] = pwm_value;

            RCLCPP_DEBUG(this->get_logger(),
                "구동기 %d 온도 제어: 목표=%.1f°C, 현재=%.1f°C, 출력=%d",
                id, target_temp, actuator_states_[id].current_temp, pwm_value);
        }

        // 모든 구동기 처리 후 통합 명령 발행
        publish_pwm_command();
    }

    // PWM 명령 발행 함수
    void publish_pwm_command()
    {
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = this->now();
        pwm_msg.pwm = current_pwm_values_;
        pwm_publisher_->publish(pwm_msg);
    }

    // PI 파라미터 설정 서비스 핸들러
    void handle_pi_parameters(
        const std::shared_ptr<wearable_robot_interfaces::srv::SetControlParams::Request> request,
        std::shared_ptr<wearable_robot_interfaces::srv::SetControlParams::Response> response)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        try {
            // 파라미터 유효성 검사
            if (request->kp < 0.0 || request->ki < 0.0) {
                throw std::invalid_argument("게인 값은 음수가 될 수 없습니다");
            }

            int actuator_id = request->actuator_id;
            bool success = false;
            std::string message;

            // 특정 구동기만 파라미터 변경
            if (actuator_id >= 0 && actuator_id < TOTAL_CHANNELS) {
                if (active_actuator_ids_.find(actuator_id) != active_actuator_ids_.end()) {
                    // 개별 구동기 파라미터 업데이트
                    std::string kp_param = "actuator" + std::to_string(actuator_id) + ".kp";
                    std::string ki_param = "actuator" + std::to_string(actuator_id) + ".ki";

                    this->set_parameter(rclcpp::Parameter(kp_param, request->kp));
                    this->set_parameter(rclcpp::Parameter(ki_param, request->ki));

                    // 내부 상태 변수도 업데이트
                    actuator_states_[actuator_id].kp = request->kp;
                    actuator_states_[actuator_id].ki = request->ki;

                    success = true;
                    message = "구동기 " + std::to_string(actuator_id) + "의 PI 파라미터가 업데이트되었습니다 (Kp=" +
                            std::to_string(request->kp) + ", Ki=" + std::to_string(request->ki) + ")";
                } else {
                    message = "구동기 " + std::to_string(actuator_id) + "은(는) 활성화되지 않았습니다";
                }
            }
            // 모든 구동기 파라미터 변경
            else if (actuator_id == -1) {
                for (const auto& id : active_actuator_ids_) {
                    // 개별 구동기 파라미터 업데이트
                    std::string kp_param = "actuator" + std::to_string(id) + ".kp";
                    std::string ki_param = "actuator" + std::to_string(id) + ".ki";

                    this->set_parameter(rclcpp::Parameter(kp_param, request->kp));
                    this->set_parameter(rclcpp::Parameter(ki_param, request->ki));

                    // 내부 상태 변수도 업데이트
                    actuator_states_[id].kp = request->kp;
                    actuator_states_[id].ki = request->ki;
                }

                success = true;
                message = "모든 구동기의 PI 파라미터가 업데이트되었습니다 (Kp=" +
                        std::to_string(request->kp) + ", Ki=" + std::to_string(request->ki) + ")";
            } else {
                message = "유효하지 않은 구동기 ID: " + std::to_string(actuator_id);
            }

            response->success = success;
            response->message = message;
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "PI 파라미터 업데이트 실패: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // 비상 정지 서비스 핸들러
    void handle_emergency_stop(
        const std::shared_ptr<wearable_robot_interfaces::srv::EmergencyStop::Request> request,
        std::shared_ptr<wearable_robot_interfaces::srv::EmergencyStop::Response> response)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        try {
            if (request->activate) {
                // 비상 정지 활성화
                if (!is_emergency_stop_) {
                    activate_emergency_stop();
                    response->message = "비상 정지가 활성화되었습니다";
                } else {
                    response->message = "비상 정지가 이미 활성화되어 있습니다";
                }
            } else {
                // 비상 정지 해제
                if (is_emergency_stop_) {
                    is_emergency_stop_ = false;
                    response->message = "비상 정지가 해제되었습니다";

                    // 모든 구동기의 온도 안전성 확인
                    bool all_safe = true;
                    double safety_threshold = this->get_parameter("safety_threshold").as_double();

                    for (const auto& id : active_actuator_ids_) {
                        if (actuator_states_[id].current_temp >= safety_threshold) {
                            all_safe = false;
                            break;
                        }
                    }

                    if (all_safe) {
                        is_temperature_safe_ = true;
                    } else {
                        response->message += " (주의: 일부 구동기의 온도가 여전히 안전 임계값을 초과함)";
                    }
                } else {
                    response->message = "비상 정지가 이미 해제되어 있습니다";
                }
            }

            response->success = true;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "비상 정지 상태 변경 실패: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // 파라미터 설정 콜백
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            std::string param_name = param.get_name();

            if (param_name == "target_temperature") {
                double target_temp = param.as_double();
                RCLCPP_INFO(this->get_logger(), "목표 온도가 %.1f°C로 변경되었습니다", target_temp);

                // 목표 온도가 0에 가까우면 즉시 모든 PWM 출력을 0으로 설정
                if (target_temp <= 0.1) {
                    std::fill(current_pwm_values_.begin(), current_pwm_values_.end(), 0);
                    publish_pwm_command();
                }
            } else if (param_name == "safety_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "안전 온도 임계값이 %.1f°C로 변경되었습니다", param.as_double());
            } else if (param_name == "actuator_ids") {
                // 활성화된 구동기 ID 업데이트
                active_actuator_ids_.clear();
                std::vector<int64_t> actuator_ids = param.as_integer_array();

                std::string active_actuators = "활성화된 구동기: ";
                for (const auto& id : actuator_ids) {
                    if (id >= 0 && id < TOTAL_CHANNELS) {
                        active_actuator_ids_.insert(static_cast<int>(id));
                        active_actuators += std::to_string(id) + ", ";

                        // 필요한 경우 상태 객체 초기화
                        if (actuator_states_.find(id) == actuator_states_.end()) {
                            actuator_states_[id] = ActuatorState();

                            // PI 파라미터 초기화
                            std::string actuator_prefix = "actuator" + std::to_string(id);
                            std::string kp_param = actuator_prefix + ".kp";
                            std::string ki_param = actuator_prefix + ".ki";

                            if (this->has_parameter(kp_param)) {
                                actuator_states_[id].kp = this->get_parameter(kp_param).as_double();
                            } else {
                                this->declare_parameter(kp_param, DEFAULT_KP);
                                actuator_states_[id].kp = DEFAULT_KP;
                            }

                            if (this->has_parameter(ki_param)) {
                                actuator_states_[id].ki = this->get_parameter(ki_param).as_double();
                            } else {
                                this->declare_parameter(ki_param, DEFAULT_KI);
                                actuator_states_[id].ki = DEFAULT_KI;
                            }
                        }
                    }
                }

                // 마지막 쉼표와 공백 제거
                if (!active_actuator_ids_.empty()) {
                    active_actuators.pop_back();
                    active_actuators.pop_back();
                }

                RCLCPP_INFO(this->get_logger(), "%s", active_actuators.c_str());
            } else if (param_name.find(".kp") != std::string::npos || param_name.find(".ki") != std::string::npos) {
                // 개별 구동기 PI 게인 업데이트
                size_t prefix_pos = param_name.find("actuator");
                if (prefix_pos == 0) {
                    size_t id_start = 8;  // "actuator"의 길이
                    size_t id_end = param_name.find(".", id_start);

                    if (id_end != std::string::npos) {
                        std::string id_str = param_name.substr(id_start, id_end - id_start);
                        int actuator_id = std::stoi(id_str);

                        if (active_actuator_ids_.find(actuator_id) != active_actuator_ids_.end()) {
                            std::string param_type = param_name.substr(id_end + 1);

                            if (param_type == "kp") {
                                actuator_states_[actuator_id].kp = param.as_double();
                                RCLCPP_INFO(this->get_logger(),
                                    "구동기 %d의 Kp 게인이 %.2f로 변경되었습니다",
                                    actuator_id, param.as_double());
                            } else if (param_type == "ki") {
                                actuator_states_[actuator_id].ki = param.as_double();
                                RCLCPP_INFO(this->get_logger(),
                                    "구동기 %d의 Ki 게인이 %.3f로 변경되었습니다",
                                    actuator_id, param.as_double());
                            }
                        }
                    }
                }
            }
        }

        return result;
    }

    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr target_temp_subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_publisher_;

    // 서비스
    rclcpp::Service<wearable_robot_interfaces::srv::SetControlParams>::SharedPtr pi_param_service_;
    rclcpp::Service<wearable_robot_interfaces::srv::EmergencyStop>::SharedPtr emergency_service_;

    // 파라미터 콜백
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 타이머
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 제어 변수
    std::set<int> active_actuator_ids_;              // 활성화된 구동기 ID 집합
    std::map<int, ActuatorState> actuator_states_;   // 각 구동기별 상태 정보
    std::vector<uint8_t> current_pwm_values_;        // 현재 PWM 값 배열
    std::mutex control_mutex_;                       // 제어 로직 동기화를 위한 뮤텍스

    // 안전 관련 변수
    bool is_emergency_stop_;    // 비상 정지 상태
    bool is_temperature_safe_;  // 온도 안전 상태
};

// 클래스 외부에서 static constexpr 상수들의 정의 추가
constexpr int WaistActuatorControlNode::TOTAL_CHANNELS;
constexpr double WaistActuatorControlNode::DEFAULT_KP;
constexpr double WaistActuatorControlNode::DEFAULT_KI;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaistActuatorControlNode>());
    rclcpp::shutdown();
    return 0;
}
