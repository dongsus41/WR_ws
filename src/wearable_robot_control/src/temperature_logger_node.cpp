#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <fstream>
#include <ctime>
#include <iomanip>

class TemperatureLogger : public rclcpp::Node
{
public:
    TemperatureLogger() : Node("temperature_logger")
    {
        // 파라미터 설정
        this->declare_parameter("active_actuator", 5);
        this->declare_parameter("target_temperature", 50.0);

        // CSV 파일 생성 및 헤더 작성
        setupCSVFile();

        // 온도 데이터 구독
        temp_sub_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&TemperatureLogger::temperature_callback, this, std::placeholders::_1));

        // PWM 명령 구독
        pwm_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&TemperatureLogger::pwm_callback, this, std::placeholders::_1));

        // 타이머 설정 (1초마다 로깅)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TemperatureLogger::log_data, this));

        RCLCPP_INFO(this->get_logger(), "Temperature Logger started. Saving to: %s", csv_filename_.c_str());
    }

    ~TemperatureLogger()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed");
        }
    }

private:
    void setupCSVFile()
    {
        // 현재 시간을 파일 이름에 포함
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "temperature_control_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".csv";
        csv_filename_ = ss.str();

        // CSV 파일 열기
        csv_file_.open(csv_filename_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            return;
        }

        // CSV 헤더 작성
        csv_file_ << "timestamp,temperature,target_temperature,pwm_output" << std::endl;
    }

    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        int actuator_idx = this->get_parameter("active_actuator").as_int();
        if (msg->temperature.size() > static_cast<size_t>(actuator_idx)) {
            current_temp_ = msg->temperature[actuator_idx];
            temp_timestamp_ = this->now();
        }
    }

    void pwm_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        int actuator_idx = this->get_parameter("active_actuator").as_int();
        if (msg->pwm.size() > static_cast<size_t>(actuator_idx)) {
            current_pwm_ = msg->pwm[actuator_idx];
            pwm_timestamp_ = this->now();
        }
    }

    void log_data()
    {
        if (!csv_file_.is_open()) return;

        double target_temp = this->get_parameter("target_temperature").as_double();
        double timestamp = this->now().seconds();

        // CSV 데이터 작성
        csv_file_ << std::fixed << std::setprecision(3)
                 << timestamp << ","
                 << current_temp_ << ","
                 << target_temp << ","
                 << static_cast<int>(current_pwm_)
                 << std::endl;

        // 파일을 즉시 저장 (버퍼링 방지)
        csv_file_.flush();

        RCLCPP_DEBUG(this->get_logger(),
            "Logged: Time=%.3f, Temp=%.2f, Target=%.2f, PWM=%d",
            timestamp, current_temp_, target_temp, current_pwm_);
    }

    std::string csv_filename_;
    std::ofstream csv_file_;
    double current_temp_ = 0.0;
    uint8_t current_pwm_ = 0;
    rclcpp::Time temp_timestamp_{0, 0};
    rclcpp::Time pwm_timestamp_{0, 0};

    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureLogger>());
    rclcpp::shutdown();
    return 0;
}
