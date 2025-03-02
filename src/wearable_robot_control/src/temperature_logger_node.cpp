#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <cstring>
#include <cerrno>

class TemperatureLogger : public rclcpp::Node
{
public:
    TemperatureLogger() : Node("temperature_logger")
    {
        // 파라미터 설정
        this->declare_parameter("active_actuator", 5);
        this->declare_parameter("target_temperature", 50.0);
        this->declare_parameter("log_directory", "~/temp_logs");
        this->declare_parameter("safety_threshold", 80.0);

        // 시작 시간 초기화
        start_time_ = this->now();

        // 온도 데이터 구독
        temp_sub_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&TemperatureLogger::temperature_callback, this, std::placeholders::_1));

        // PWM 명령 구독
        pwm_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&TemperatureLogger::pwm_callback, this, std::placeholders::_1));

        // 목표 온도 구독
        target_temp_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_temperature", 10,
            std::bind(&TemperatureLogger::target_temp_callback, this, std::placeholders::_1));

        // 파일명 발행 토픽 (현재 저장되는 파일명을 GUI에 알림)
        filename_pub_ = this->create_publisher<std_msgs::msg::String>(
            "current_log_filename", 10);

        // 서비스 설정
        setupServices();

        // CSV 파일 생성 및 헤더 작성 (서비스 설정 후에 호출)
        if (!setupCSVFile()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup CSV file. Logger will not record data.");
        }

        // 타이머 설정 (40ms마다 로깅, 즉 25Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&TemperatureLogger::log_data, this));

        RCLCPP_INFO(this->get_logger(), "Temperature Logger started.");
    }

    ~TemperatureLogger()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed");
        }
    }

private:
    void setupServices()
    {
        try {
            // 파일 저장 서비스
            save_file_service_ = this->create_service<std_srvs::srv::SetBool>(
                "save_temperature_log",
                std::bind(&TemperatureLogger::handle_save_file, this, std::placeholders::_1, std::placeholders::_2));

            // 로깅 초기화 서비스
            reset_service_ = this->create_service<std_srvs::srv::Trigger>(
                "reset_temperature_log",
                std::bind(&TemperatureLogger::handle_reset_logging, this, std::placeholders::_1, std::placeholders::_2));

            // 파일명 설정 서비스
            set_filename_service_ = this->create_service<std_srvs::srv::SetBool>(
                "set_log_filename",
                std::bind(&TemperatureLogger::handle_set_filename, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Services initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create services: %s", e.what());
        }
    }

    // POSIX 디렉토리 존재 여부 확인 함수
    bool directory_exists(const std::string& path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            return false; // 경로 접근 불가
        }
        return (info.st_mode & S_IFDIR); // 디렉토리인지 확인
    }

    // POSIX 디렉토리 생성 함수
    bool create_directory(const std::string& path) {
        int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (status != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                path.c_str(), strerror(errno));
            return false;
        }
        return true;
    }

    bool setupCSVFile(const std::string& custom_filename = "")
    {
        try {
            // 기존 파일 닫기
            if (csv_file_.is_open()) {
                csv_file_.close();
                RCLCPP_INFO(this->get_logger(), "Previous CSV file closed");
            }

            // 파일 이름 생성
            if (custom_filename.empty()) {
                // 현재 시간을 파일 이름에 포함
                auto now = std::chrono::system_clock::now();
                auto time = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << "temperature_control_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".csv";
                csv_filename_ = ss.str();
            } else {
                // 사용자 지정 파일명 사용
                csv_filename_ = custom_filename;
                if (csv_filename_.find(".csv") == std::string::npos) {
                    csv_filename_ += ".csv";
                }
            }

            // 로그 디렉토리 처리 - 안전하게 처리
            std::string log_dir = this->get_parameter("log_directory").as_string();

            // 홈 디렉토리 확장
            if (!log_dir.empty() && log_dir.front() == '~') {
                const char* home = std::getenv("HOME");
                if (home) {
                    log_dir.replace(0, 1, home);
                } else {
                    RCLCPP_WARN(this->get_logger(), "HOME environment variable not set, using current directory");
                    log_dir = ".";
                }
            }

            // 디렉토리가 없으면 생성 시도
            if (!directory_exists(log_dir)) {
                RCLCPP_INFO(this->get_logger(), "Directory does not exist: %s. Attempting to create...", log_dir.c_str());
                if (!create_directory(log_dir)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to create directory, using current directory instead");
                    log_dir = ".";
                } else {
                    RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.c_str());
                }
            }

            // 전체 경로 설정
            std::string full_path = log_dir + "/" + csv_filename_;
            RCLCPP_INFO(this->get_logger(), "Saving to file: %s", full_path.c_str());

            // CSV 파일 열기
            csv_file_.open(full_path);
            if (!csv_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", full_path.c_str());
                return false;
            }

            // CSV 헤더 작성
            csv_file_ << "timestamp,elapsed_seconds,temperature,target_temperature,pwm_output" << std::endl;

            // 시작 시간 재설정
            start_time_ = this->now();

            RCLCPP_INFO(this->get_logger(), "New CSV file created: %s", full_path.c_str());

            // 현재 파일명 발행
            publish_current_filename();
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in setupCSVFile: %s", e.what());
            return false;
        }
    }

    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        try {
            int actuator_idx = this->get_parameter("active_actuator").as_int();
            if (msg->temperature.size() > static_cast<size_t>(actuator_idx)) {
                current_temp_ = msg->temperature[actuator_idx];
                temp_timestamp_ = this->now();
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in temperature_callback: %s", e.what());
        }
    }

    void pwm_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        try {
            int actuator_idx = this->get_parameter("active_actuator").as_int();
            if (msg->pwm.size() > static_cast<size_t>(actuator_idx)) {
                current_pwm_ = msg->pwm[actuator_idx];
                pwm_timestamp_ = this->now();
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in pwm_callback: %s", e.what());
        }
    }

    void target_temp_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        try {
            // 목표 온도 업데이트
            current_target_temp_ = msg->data;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in target_temp_callback: %s", e.what());
        }
    }

    void publish_current_filename()
    {
        try {
            auto msg = std::make_shared<std_msgs::msg::String>();
            msg->data = csv_filename_;
            filename_pub_->publish(*msg);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in publish_current_filename: %s", e.what());
        }
    }

    bool handle_save_file(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> /* request */,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        try {
            if (!csv_file_.is_open()) {
                response->success = false;
                response->message = "Error: CSV file is not open";
                return true;
            }

            // 파일 강제 저장
            csv_file_.flush();

            response->success = true;
            response->message = "CSV file saved: " + csv_filename_;
            RCLCPP_INFO(this->get_logger(), "CSV file flushed to disk: %s", csv_filename_.c_str());
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in handle_save_file: %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            return true;
        }
    }

    bool handle_reset_logging(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try {
            // 새 파일 생성
            if (setupCSVFile()) {
                response->success = true;
                response->message = "Logging reset, new file created: " + csv_filename_;
                RCLCPP_INFO(this->get_logger(), "Logging reset with new file: %s", csv_filename_.c_str());
            } else {
                response->success = false;
                response->message = "Failed to reset logging";
                RCLCPP_ERROR(this->get_logger(), "Failed to reset logging");
            }
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in handle_reset_logging: %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            return true;
        }
    }

    bool handle_set_filename(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> /* request */,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        try {
            // 새 파일 생성
            if (setupCSVFile()) {
                response->success = true;
                response->message = "New log file created: " + csv_filename_;
            } else {
                response->success = false;
                response->message = "Failed to create new log file";
            }
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in handle_set_filename: %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            return true;
        }
    }

    void log_data()
    {
        try {
            if (!csv_file_.is_open()) return;

            // 메인 타이머 주기에 맞춰서 기록
            rclcpp::Time current_time = this->now();
            double timestamp = current_time.seconds();

            // 시작 시간으로부터의 경과 시간 계산 (초 단위)
            double elapsed_seconds = (current_time - start_time_).seconds();

            // CSV 데이터 작성
            csv_file_ << std::fixed << std::setprecision(3)
                    << timestamp << ","
                    << elapsed_seconds << ","  // 경과 시간 추가
                    << current_temp_ << ","
                    << current_target_temp_ << ","
                    << static_cast<int>(current_pwm_)
                    << std::endl;

            // 주기적 파일 저장 (메모리 버퍼에서 디스크로)
            flush_counter_++;
            if (flush_counter_ >= FLUSH_INTERVAL) {
                csv_file_.flush();
                flush_counter_ = 0;
            }

            RCLCPP_DEBUG(this->get_logger(),
                "Logged: Time=%.3f, Elapsed=%.3f, Temp=%.2f, Target=%.2f, PWM=%d",
                timestamp, elapsed_seconds, current_temp_, current_target_temp_, current_pwm_);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in log_data: %s", e.what());
        }
    }

    // 구독자 및 발행자
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_temp_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr filename_pub_;

    // 서비스 서버
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr save_file_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_filename_service_;

    // 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // 데이터 관련 변수
    std::string csv_filename_;
    std::ofstream csv_file_;
    double current_temp_ = 0.0;
    double current_target_temp_ = 50.0;  // 기본 목표 온도
    uint8_t current_pwm_ = 0;
    rclcpp::Time start_time_{0, 0};
    rclcpp::Time temp_timestamp_{0, 0};
    rclcpp::Time pwm_timestamp_{0, 0};

    // 파일 저장 관련
    int flush_counter_ = 0;
    const int FLUSH_INTERVAL = 25;  // 약 1초마다 디스크에 저장
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureLogger>());
    rclcpp::shutdown();
    return 0;
}
