#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/displacement_data.hpp>
#include <wearable_robot_interfaces/msg/back_intension.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>
#include <array>
#include <mutex>

class WaistAssistControlNode : public rclcpp::Node
{
public:
    // 상수 정의
    static constexpr int NUM_ACTUATORS = 2;             // 허리 보조용 구동기 수
    static constexpr int WAIST_SENSOR_INDEX = 0;        // 허리 변위 센서는 항상 0번 인덱스
    static constexpr int TOTAL_CHANNELS = 6;            // 전체 채널 수
    static constexpr int DEFAULT_INDICES[NUM_ACTUATORS] = {4, 5}; // 기본 구동기 인덱스

    WaistAssistControlNode() : Node("waist_support_control_node")
    {
        // 변위 센서 데이터 구독 (허리 각도 측정용)
        displacement_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementData>(
            "displacement_data", 10,
            std::bind(&WaistAssistControlNode::displacement_callback, this, std::placeholders::_1));

        // 사용자 의도 데이터 구독
        intention_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::BackIntension>(
            "intention_data", 10,
            std::bind(&WaistAssistControlNode::intention_callback, this, std::placeholders::_1));

        // GUI에서 타겟 온도 수정용 구독
        target_temp_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_temperature_setting", 10,
            std::bind(&WaistAssistControlNode::target_temp_callback, this, std::placeholders::_1));

        // 타겟 온도 발행
        target_temp_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::TemperatureData>(
            "target_temperature", 10);

        // ----- 파라미터 선언 -----
        this->declare_parameter("actuator_indices", std::vector<int64_t>{4, 5});
        this->declare_parameter("waist_assist_active", true);
        this->declare_parameter("waist_angle_threshold", 3.0);
        this->declare_parameter("target_temperature", 60.0);
        this->declare_parameter("control_frequency", 250.0);

        // 파라미터 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WaistAssistControlNode::parameter_callback, this, std::placeholders::_1));

        // 제어 타이머 설정
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

        target_temperature_ = this->get_parameter("target_temperature").as_double();
        waist_angle_ = 0.0;
        current_intention_id_ = 0;
        is_active_ = false;

        // 초기화 정보 출력
        RCLCPP_INFO(this->get_logger(), "허리 보조 제어 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "제어 구동기: %d번, %d번", actuator_indices_[0], actuator_indices_[1]);
        RCLCPP_INFO(this->get_logger(), "허리 변위 센서: %d번", WAIST_SENSOR_INDEX);
        RCLCPP_INFO(this->get_logger(), "허리 각도 임계값: %.1f", this->get_parameter("waist_angle_threshold").as_double());
        RCLCPP_INFO(this->get_logger(), "구동기 목표 온도: %.1f°C", target_temperature_);
        RCLCPP_INFO(this->get_logger(), "제어 주파수: %.1fHz", control_frequency);

        // 초기 타겟 온도 발행
        publish_target_temperature();
    }

private:
    // 변위 센서 데이터 수신 콜백
    void displacement_callback(const wearable_robot_interfaces::msg::DisplacementData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 허리 각도 센서 데이터 확인
        if (msg->displacement.size() > WAIST_SENSOR_INDEX) {
            waist_angle_ = msg->displacement[WAIST_SENSOR_INDEX];
            RCLCPP_DEBUG(this->get_logger(), "허리 각도: %.2f", waist_angle_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "허리 변위 센서 데이터가 없습니다");
        }
    }

    // 사용자 의도 데이터 수신 콜백
    void intention_callback(const wearable_robot_interfaces::msg::BackIntension::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        int previous_intention = current_intention_id_;
        current_intention_id_ = msg->intention_id;

        // 의도 ID 변경 로그 출력
        if (previous_intention != current_intention_id_) {
            if (current_intention_id_ == 0) {
                RCLCPP_INFO(this->get_logger(), "허리 보조 의도가 해제되었습니다 (ID: 0). 보조 모드 비활성화");
            } else {
                RCLCPP_INFO(this->get_logger(), "허리 보조 의도가 감지되었습니다 (ID: %d). 보조 모드 활성화", current_intention_id_);
            }
        }
    }

    // GUI에서 타겟 온도 수정 콜백
    void target_temp_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_temperature_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "구동기 목표 온도가 %.1f°C로 변경되었습니다", target_temperature_);
        publish_target_temperature();
    }

    // 주기적 제어 콜백
    void control_callback()
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 허리 보조 제어 로직
        bool waist_assist_active = this->get_parameter("waist_assist_active").as_bool();
        double waist_angle_threshold = this->get_parameter("waist_angle_threshold").as_double();

        // 활성화 상태 계산
        bool new_active_state = waist_assist_active &&
                               current_intention_id_ != 0 &&
                               waist_angle_ > waist_angle_threshold;

        // 상태가 변경되었을 때만 로그 출력 및 타겟 온도 발행
        if (new_active_state != is_active_) {
            is_active_ = new_active_state;

            if (is_active_) {
                RCLCPP_INFO(this->get_logger(), "구동기 활성화: 목표 온도 %.1f°C 설정", target_temperature_);
            } else {
                RCLCPP_INFO(this->get_logger(), "구동기 비활성화: 목표 온도 0°C 설정");
            }

            publish_target_temperature();
        }
    }

    // 타겟 온도 발행 함수
    void publish_target_temperature()
    {
        auto temp_msg = wearable_robot_interfaces::msg::TemperatureData();
        temp_msg.header.stamp = this->now();
        temp_msg.temperature.resize(TOTAL_CHANNELS, 0.0);

        // 활성화된 경우에만 타겟 온도 설정
        if (is_active_) {
            // 목표 구동기에만 타겟 온도 설정
            for (int i = 0; i < NUM_ACTUATORS; i++) {
                int idx = actuator_indices_[i];
                if (idx >= 0 && idx < TOTAL_CHANNELS) {
                    temp_msg.temperature[idx] = target_temperature_;
                }
            }
        }

        // 메시지 발행
        target_temp_publisher_->publish(temp_msg);
    }

    // 파라미터 설정 콜백
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool need_to_publish = false;

        for (const auto &param : parameters) {
            const std::string& name = param.get_name();

            if (name == "waist_assist_active") {
                RCLCPP_INFO(this->get_logger(),
                    "허리 보조 기능이 %s되었습니다", param.as_bool() ? "활성화" : "비활성화");
                need_to_publish = true;
            }
            else if (name == "waist_angle_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "허리 각도 임계값이 %.1f로 변경되었습니다", param.as_double());
                need_to_publish = true;
            }
            else if (name == "target_temperature") {
                target_temperature_ = param.as_double();
                RCLCPP_INFO(this->get_logger(),
                    "구동기 목표 온도 %.1f°C로 변경", target_temperature_);
                need_to_publish = true;
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
                    need_to_publish = true;
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "유효하지 않은 구동기 인덱스 목록: 최소 %d개 필요", NUM_ACTUATORS);
                    result.successful = false;
                }
            }
        }

        // 파라미터 변경으로 인해 타겟 온도 발행이 필요하면 발행
        if (need_to_publish && is_active_) {
            publish_target_temperature();
        }

        return result;
    }

    // ----- 멤버 변수 -----
    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementData>::SharedPtr displacement_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::BackIntension>::SharedPtr intention_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_temp_subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr target_temp_publisher_;

    // 파라미터 콜백
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 타이머
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 구동기 인덱스
    std::array<int, NUM_ACTUATORS> actuator_indices_;

    // 제어 상태 변수
    double waist_angle_;          // 허리 각도
    int current_intention_id_;    // 현재 의도 ID (0: 동작 없음)
    double target_temperature_;   // 구동기 목표 온도
    bool is_active_;              // 현재 활성화 상태
    std::mutex control_mutex_;    // 제어 로직 동기화를 위한 뮤텍스
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaistAssistControlNode>());
    rclcpp::shutdown();
    return 0;
}
