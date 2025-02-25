#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/displacement_raw_data.hpp>
#include <wearable_robot_interfaces/msg/displacement_data.hpp>
#include <deque>
#include <vector>
#include <cmath>

class DisplacementProcessingNode : public rclcpp::Node
{
public:
    static const int FILTER_SIZE = 5;
    const size_t NUM_SENSORS = 10;

    DisplacementProcessingNode() : Node("displacement_processing_node")
    {
        // Subscriber for raw displacement data
        subscription_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementRawData>(
            "displacement_raw_data", 10,
            std::bind(&DisplacementProcessingNode::displacement_callback, this, std::placeholders::_1));

        // Publisher for processed displacement data
        publisher_ = this->create_publisher<wearable_robot_interfaces::msg::DisplacementData>(
            "displacement_data", 10);

        // Initialize filter queues for each sensor
        displacement_queues_.resize(NUM_SENSORS);
        calibration_coeffs_.resize(NUM_SENSORS);

        // Initialize calibration coefficients
        initializeCalibrationCoefficients();

        RCLCPP_INFO(this->get_logger(), "Displacement Processing Node has been started");
    }

private:
    struct CalibrationCoefficients {
        double a, b, c, d, e, f;
    };

    void initializeCalibrationCoefficients()
    {
        // 각 센서별로 캘리브레이션 계수 로드
        for (size_t i = 0; i < NUM_SENSORS; ++i) {
            std::string sensor_prefix = "sensor" + std::to_string(i);

            // 각 계수에 대한 파라미터 선언 및 로드
            this->declare_parameter(sensor_prefix + ".a", 0.0);
            this->declare_parameter(sensor_prefix + ".b", 0.0);
            this->declare_parameter(sensor_prefix + ".c", 0.0);
            this->declare_parameter(sensor_prefix + ".d", 0.0);
            this->declare_parameter(sensor_prefix + ".e", 0.0);
            this->declare_parameter(sensor_prefix + ".f", 0.0);

            calibration_coeffs_[i].a = this->get_parameter(sensor_prefix + ".a").as_double();
            calibration_coeffs_[i].b = this->get_parameter(sensor_prefix + ".b").as_double();
            calibration_coeffs_[i].c = this->get_parameter(sensor_prefix + ".c").as_double();
            calibration_coeffs_[i].d = this->get_parameter(sensor_prefix + ".d").as_double();
            calibration_coeffs_[i].e = this->get_parameter(sensor_prefix + ".e").as_double();
            calibration_coeffs_[i].f = this->get_parameter(sensor_prefix + ".f").as_double();
        }
    }

    float filterDisplacementValue(float raw_value, std::deque<float>& queue) {
        // ADC 값을 전압으로 변환 (0-65535 -> 0-5V)
        float voltage = raw_value * 5.0f / 65535.0f;

        queue.push_back(voltage);
        if (queue.size() > FILTER_SIZE) {
            queue.pop_front();
        }

        float sum = 0.0f;
        for (const auto& val : queue) {
            sum += val;
        }
        return sum / queue.size();
    }

    double calibrateDisplacement(float voltage, const CalibrationCoefficients& coeff)
    {
        return coeff.a * std::pow(voltage, 5) +
               coeff.b * std::pow(voltage, 4) +
               coeff.c * std::pow(voltage, 3) +
               coeff.d * std::pow(voltage, 2) +
               coeff.e * voltage +
               coeff.f;
    }

    void displacement_callback(const wearable_robot_interfaces::msg::DisplacementRawData::SharedPtr msg)
    {
        auto processed_msg = wearable_robot_interfaces::msg::DisplacementData();
        processed_msg.header = msg->header;

        // 수신된 센서 개수 확인 및 처리
        size_t num_received_sensors = msg->displacement_raw.size();
        size_t num_sensors_to_process = std::min(num_received_sensors, NUM_SENSORS);

        // 출력 메시지 배열 초기화
        processed_msg.displacement.resize(num_sensors_to_process);

        // 모든 변위 센서 데이터 처리
        for (size_t i = 0; i < num_sensors_to_process; ++i) {
            // 필터링
            float filtered_voltage = filterDisplacementValue(msg->displacement_raw[i], displacement_queues_[i]);
            // 캘리브레이션
            double calibrated_value = calibrateDisplacement(filtered_voltage, calibration_coeffs_[i]);

            // 결과값 저장
            processed_msg.displacement[i] = calibrated_value;
        }

        publisher_->publish(processed_msg);

        if (num_sensors_to_process >= 2) {
            RCLCPP_DEBUG(this->get_logger(),
                "Processed displacement data - Sensor0: %.2f mm, Sensor1: %.2f mm",
                processed_msg.displacement[0], processed_msg.displacement[1]);
        }
    }

    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementRawData>::SharedPtr subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::DisplacementData>::SharedPtr publisher_;
    std::vector<std::deque<float>> displacement_queues_;
    std::vector<CalibrationCoefficients> calibration_coeffs_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplacementProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
