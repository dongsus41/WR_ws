#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/displacement_data.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/back_intention.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fstream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <deque>
#include <string>
#include <sys/stat.h>  // mkdir 사용을 위해 추가
#include <condition_variable>

// ROS2 Foxy에서는 std::filesystem을 사용하는 대신 sys/stat.h 함수 사용
inline bool directory_exists(const std::string& dir) {
    struct stat info;
    return (stat(dir.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

inline bool create_directory(const std::string& dir) {
    // 리눅스에서 디렉토리 생성 (0755 권한)
    return (mkdir(dir.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) == 0);
}

/**
 * @brief 웨어러블 로봇 데이터 로거 노드
 *
 * 웨어러블 로봇의 다양한 센서 및 제어 데이터를 125Hz 이상의 고속으로 수집하여
 * CSV 파일로 저장합니다. GUI 플러그인과 연동되어 데이터 저장을 시작/중지할 수 있습니다.
 */
class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node"),
                      is_logging_(false),
                      thread_exit_(false),
                      data_points_count_(0)
    {
        // 파라미터 선언 및 로드
        this->declare_parameter("log_directory", "~/ros2_csv_logs");
        this->declare_parameter("sampling_rate", 125);  // 기본 샘플링 속도 125Hz
        this->declare_parameter("max_queue_size", 10000);  // 최대 큐 크기
        this->declare_parameter("buffer_flush_size", 1000); // 버퍼 플러시 크기

        // 파라미터 로드
        log_directory_ = this->get_parameter("log_directory").as_string();
        sampling_rate_ = this->get_parameter("sampling_rate").as_int();
        max_queue_size_ = this->get_parameter("max_queue_size").as_int();
        buffer_flush_size_ = this->get_parameter("buffer_flush_size").as_int();

        // 홈 디렉토리 확장
        if (log_directory_.find("~") == 0) {
            const char* home = std::getenv("HOME");
            if (home) {
                log_directory_.replace(0, 1, home);
            }
        }

        // 로그 디렉토리 생성
        if (!directory_exists(log_directory_)) {
            try {
                if (!create_directory(log_directory_)) {
                    RCLCPP_ERROR(this->get_logger(), "로그 디렉토리 생성 실패: %s",
                               log_directory_.c_str());
                    log_directory_ = "./";  // 현재 디렉토리로 폴백
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "로그 디렉토리 생성 중 오류: %s, 오류: %s",
                           log_directory_.c_str(), e.what());
                log_directory_ = "./";  // 현재 디렉토리로 폴백
            }
        }

        // 데이터 큐 초기화
        data_queue_.clear();

        // 구독자 생성
        displacement_sub_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementData>(
            "displacement_data", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::displacement_callback, this, std::placeholders::_1));

        temperature_sub_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::temperature_callback, this, std::placeholders::_1));

        fan_sub_ = this->create_subscription<wearable_robot_interfaces::msg::FanCommand>(
            "fan_state", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::fan_callback, this, std::placeholders::_1));

        pwm_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "pwm_state", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::pwm_callback, this, std::placeholders::_1));

        intention_sub_ = this->create_subscription<wearable_robot_interfaces::msg::BackIntention>(
            "intention_data", rclcpp::SensorDataQoS(),
            std::bind(&DataLoggerNode::intention_callback, this, std::placeholders::_1));

        // GUI 제어용 구독자
        start_logging_sub_ = this->create_subscription<std_msgs::msg::String>(
            "start_data_logging", 10,
            std::bind(&DataLoggerNode::start_logging_callback, this, std::placeholders::_1));

        stop_logging_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "stop_data_logging", 10,
            std::bind(&DataLoggerNode::stop_logging_callback, this, std::placeholders::_1));

        // 상태 발행자
        logging_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "data_logging_status", 10);

        // 초기 상태 설정
        current_file_ = nullptr;

        // 로깅 스레드 시작
        logging_thread_ = std::thread(&DataLoggerNode::logging_thread_func, this);

        // 타이머 생성 (메인 스레드의 상태 업데이트용, 1Hz)
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DataLoggerNode::publish_status, this));

        RCLCPP_INFO(this->get_logger(),
                    "데이터 로거 노드가 초기화되었습니다. 샘플링 속도: %dHz, 로그 디렉토리: %s",
                    sampling_rate_, log_directory_.c_str());
    }

    ~DataLoggerNode()
    {
        // 로깅 중단
        if (is_logging_) {
            stop_logging();
        }

        // 스레드 종료
        if (logging_thread_.joinable()) {
            thread_exit_ = true;
            logging_cv_.notify_all();  // 조건 변수 깨우기
            logging_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "데이터 로거 노드가 종료됩니다");
    }

private:
    // 데이터 저장 구조체
    struct DataPoint {
        std::chrono::system_clock::time_point timestamp;
        double elapsed_seconds;  // 시작 시간으로부터 경과 시간
        float displacement[10];  // 최대 10개 변위 센서 지원
        float temperature[6];    // 6개 온도 센서
        uint8_t pwm[6];          // 6개 PWM 채널
        bool fan[6];             // 6개 팬 상태
        int intention_id;        // 의도 ID
        int n_displacement;      // 실제 변위 센서 개수
        bool data_valid;         // 데이터 유효성 플래그
    };

    // 변위 데이터 콜백
    void displacement_callback(const wearable_robot_interfaces::msg::DisplacementData::SharedPtr msg)
    {
        if (!is_logging_) return;

        // 현재 시간 기록
        auto now = std::chrono::system_clock::now();

        // 데이터 잠금 및 임시 저장
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 변위 데이터 갱신
            current_data_.n_displacement = std::min(static_cast<int>(msg->displacement.size()), 10);
            for (int i = 0; i < current_data_.n_displacement; i++) {
                current_data_.displacement[i] = msg->displacement[i];
            }

            // 이 콜백에서 현재 데이터를 유효하게 표시하고 큐에 추가
            current_data_.timestamp = now;

            if (start_time_ == std::chrono::system_clock::time_point()) {
                start_time_ = now;  // 첫 번째 데이터 포인트에서 시작 시간 설정
            }

            // 경과 시간 계산 (초 단위)
            current_data_.elapsed_seconds = std::chrono::duration<double>(now - start_time_).count();

            // 데이터 유효성 표시
            current_data_.data_valid = true;

            // 큐에 복사본 추가
            if (static_cast<int>(data_queue_.size()) < max_queue_size_) {
                data_queue_.push_back(current_data_);
                data_points_count_++;

                // 큐가 충분히 차면 로깅 스레드 깨우기
                if (static_cast<int>(data_queue_.size()) >= buffer_flush_size_) {
                    logging_cv_.notify_one();
                }
            } else {
                // 큐가 꽉 찬 경우 경고
                if (static_cast<int>(data_queue_.size()) == max_queue_size_) {
                    RCLCPP_WARN(this->get_logger(), "데이터 큐가 꽉 찼습니다! 일부 데이터가 손실될 수 있습니다.");
                }
            }

            // 데이터 처리 후 유효성 플래그 초기화
            current_data_.data_valid = false;
        }
    }

    // 온도 데이터 콜백
    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        if (!is_logging_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 온도 데이터 갱신
        int temp_count = std::min(static_cast<int>(msg->temperature.size()), 6);
        for (int i = 0; i < temp_count; i++) {
            current_data_.temperature[i] = msg->temperature[i];
        }
    }

    // 팬 상태 콜백
    void fan_callback(const wearable_robot_interfaces::msg::FanCommand::SharedPtr msg)
    {
        if (!is_logging_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 팬 상태 갱신
        int fan_count = std::min(static_cast<int>(msg->fan.size()), 6);
        for (int i = 0; i < fan_count; i++) {
            current_data_.fan[i] = msg->fan[i];
        }
    }

    // PWM 값 콜백
    void pwm_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        if (!is_logging_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // PWM 값 갱신
        int pwm_count = std::min(static_cast<int>(msg->pwm.size()), 6);
        for (int i = 0; i < pwm_count; i++) {
            current_data_.pwm[i] = msg->pwm[i];
        }
    }

    // 의도 데이터 콜백
    void intention_callback(const wearable_robot_interfaces::msg::BackIntention::SharedPtr msg)
    {
        if (!is_logging_) return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 의도 ID 갱신
        current_data_.intention_id = msg->intention_id;
    }

    // 로깅 시작 콜백 (GUI에서 호출)
    void start_logging_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (is_logging_) {
            RCLCPP_WARN(this->get_logger(), "이미 로깅이 진행 중입니다");
            return;
        }

        // 파일 이름 설정 (GUI에서 제공한 이름 사용)
        std::string filename = msg->data;
        if (filename.empty()) {
            filename = "waist_robot_log";  // 기본 이름
        }

        // 로깅 시작
        start_logging(filename);
    }

    // 로깅 중지 콜백 (GUI에서 호출)
    void stop_logging_callback(const std_msgs::msg::Bool::SharedPtr)
    {
        if (!is_logging_) {
            RCLCPP_WARN(this->get_logger(), "로깅이 진행 중이지 않습니다");
            return;
        }

        // 로깅 중지
        stop_logging();
    }

    // 로깅 시작 함수
    void start_logging(const std::string& base_filename)
    {
        // 현재 시간으로 타임스탬프 생성
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time_t);

        std::stringstream ss;
        ss << std::put_time(&now_tm, "%Y-%m-%d-%H-%M-%S");
        std::string timestamp = ss.str();

        // 파일 경로 구성
        std::string filename = base_filename + "_" + timestamp + ".csv";
        std::string filepath = log_directory_ + "/" + filename;

        try {
            // 파일 생성
            current_file_ = std::make_unique<std::ofstream>(filepath);
            if (!current_file_->is_open()) {
                throw std::runtime_error("파일을 열 수 없습니다: " + filepath);
            }

            // CSV 헤더 작성
            *current_file_ << "timestamp,elapsed_seconds,"
                          << "displacement0,displacement1,displacement2,displacement3,displacement4,"
                          << "displacement5,displacement6,displacement7,displacement8,displacement9,"
                          << "temp0,temp1,temp2,temp3,temp4,temp5,"
                          << "pwm0,pwm1,pwm2,pwm3,pwm4,pwm5,"
                          << "fan0,fan1,fan2,fan3,fan4,fan5,"
                          << "intention_id" << std::endl;

            // 로깅 상태 초기화
            is_logging_ = true;
            current_filename_ = filename;
            data_points_count_ = 0;
            data_queue_.clear();
            start_time_ = std::chrono::system_clock::time_point();  // 리셋
            logging_start_time_ = now;

            // 버퍼 초기화
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_data_ = DataPoint();  // 데이터 초기화

            RCLCPP_INFO(this->get_logger(), "데이터 로깅 시작: %s", filepath.c_str());

            // 상태 메시지 발행
            auto status_msg = std::make_unique<std_msgs::msg::String>();
            status_msg->data = "logging_started:" + filename;
            logging_status_pub_->publish(std::move(status_msg));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "로깅 시작 오류: %s", e.what());
            current_file_.reset();  // 파일 닫기
            is_logging_ = false;

            // 오류 상태 발행
            auto status_msg = std::make_unique<std_msgs::msg::String>();
            status_msg->data = "logging_error:" + std::string(e.what());
            logging_status_pub_->publish(std::move(status_msg));
        }
    }

    // 로깅 중지 함수
    void stop_logging()
    {
        if (!is_logging_) {
            return;
        }

        // 큐에 남은 데이터 플러시
        flush_queue();

        // 파일 닫기
        {
            std::lock_guard<std::mutex> lock(file_mutex_);
            if (current_file_ && current_file_->is_open()) {
                current_file_->close();
                current_file_.reset();
            }
        }

        // 로깅 종료 시간 및 통계 계산
        auto now = std::chrono::system_clock::now();
        double duration_sec = std::chrono::duration<double>(now - logging_start_time_).count();
        int points_count = data_points_count_; // atomic 변수를 복사
        double rate = (duration_sec > 0) ? (static_cast<double>(points_count) / duration_sec) : 0;

        // 로깅 상태 갱신 (atomic 변수 사용 전에 로컬 변수로 복사)
        is_logging_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "데이터 로깅 종료: %s, 총 %d개 데이터, 기간: %.1f초, 평균 속도: %.1fHz",
                    current_filename_.c_str(), points_count, duration_sec, rate);

        // 상태 메시지 발행
        auto status_msg = std::make_unique<std_msgs::msg::String>();
        std::stringstream ss;
        ss << "logging_stopped:" << current_filename_ << ":" << points_count
           << ":" << std::fixed << std::setprecision(1) << rate;
        status_msg->data = ss.str();
        logging_status_pub_->publish(std::move(status_msg));
    }

    // 로깅 상태 발행 (1Hz)
    void publish_status()
    {
        if (!is_logging_) {
            return;
        }

        // 현재 시간 기준 통계 계산
        auto now = std::chrono::system_clock::now();
        double duration_sec = std::chrono::duration<double>(now - logging_start_time_).count();
        int points_count = data_points_count_; // atomic 변수를 복사
        double rate = (duration_sec > 0) ? (static_cast<double>(points_count) / duration_sec) : 0;

        // 상태 메시지 발행
        auto status_msg = std::make_unique<std_msgs::msg::String>();
        std::stringstream ss;
        ss << "logging_active:" << current_filename_ << ":" << points_count
           << ":" << std::fixed << std::setprecision(1) << rate
           << ":" << data_queue_.size();
        status_msg->data = ss.str();
        logging_status_pub_->publish(std::move(status_msg));
    }

    // 큐 데이터 플러시 (파일에 쓰기)
    void flush_queue()
    {
        // 큐가 비어있으면 무시
        if (data_queue_.empty()) {
            return;
        }

        std::deque<DataPoint> queue_copy;
        {
            // 데이터 뮤텍스 잠금 및 큐 복사
            std::lock_guard<std::mutex> lock(data_mutex_);
            queue_copy.swap(data_queue_);  // 현재 큐와 빈 큐를 교체 (효율적인 방법)
        }

        // 파일 뮤텍스 잠금
        std::lock_guard<std::mutex> lock(file_mutex_);
        if (!current_file_ || !current_file_->is_open()) {
            // 파일이 없으면 데이터 버림
            RCLCPP_ERROR(this->get_logger(), "파일이 열려있지 않습니다. %lu개 데이터 손실됨", queue_copy.size());
            return;
        }

        // 파일에 데이터 작성
        for (const auto& data_point : queue_copy) {
            // 타임스탬프를 ISO 8601 형식으로 변환
            auto time_t = std::chrono::system_clock::to_time_t(data_point.timestamp);
            std::tm tm = *std::localtime(&time_t);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                data_point.timestamp.time_since_epoch()) % 1000;

            *current_file_ << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "."
                          << std::setfill('0') << std::setw(3) << ms.count() << ","
                          << std::fixed << std::setprecision(6) << data_point.elapsed_seconds << ",";

            // 변위 데이터
            for (int i = 0; i < 10; i++) {
                if (i < data_point.n_displacement) {
                    *current_file_ << data_point.displacement[i];
                } else {
                    *current_file_ << "0.0";  // 빈 값 채우기
                }
                *current_file_ << ",";
            }

            // 온도 데이터
            for (int i = 0; i < 6; i++) {
                *current_file_ << data_point.temperature[i] << ",";
            }

            // PWM 데이터
            for (int i = 0; i < 6; i++) {
                *current_file_ << static_cast<int>(data_point.pwm[i]) << ",";
            }

            // 팬 데이터
            for (int i = 0; i < 6; i++) {
                *current_file_ << (data_point.fan[i] ? "1" : "0") << ",";
            }

            // 의도 ID
            *current_file_ << data_point.intention_id << std::endl;
        }

        // 파일 플러시 (디스크에 쓰기)
        current_file_->flush();
    }

    // 로깅 스레드 함수
    void logging_thread_func()
    {
        RCLCPP_INFO(this->get_logger(), "로깅 스레드 시작됨");

        while (!thread_exit_) {
            // 조건 변수로 대기 (큐가 차거나 로깅이 중지될 때까지)
            std::unique_lock<std::mutex> lock(logging_mutex_);
            logging_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return thread_exit_ ||
                       (is_logging_ && static_cast<int>(data_queue_.size()) >= buffer_flush_size_) ||
                       (!is_logging_ && !data_queue_.empty());
            });

            // 스레드 종료 신호 확인
            if (thread_exit_) {
                break;
            }

            // 큐가 충분히 찼거나 로깅이 중지되면 파일에 쓰기
            if ((is_logging_ && static_cast<int>(data_queue_.size()) >= buffer_flush_size_) ||
                (!is_logging_ && !data_queue_.empty())) {
                flush_queue();
            }
        }

        RCLCPP_INFO(this->get_logger(), "로깅 스레드 종료됨");
    }

    // 멤버 변수
    std::string log_directory_;
    int sampling_rate_;
    int max_queue_size_;
    int buffer_flush_size_;
    std::atomic<bool> is_logging_;
    std::atomic<bool> thread_exit_;
    std::atomic<int> data_points_count_;
    std::string current_filename_;
    std::unique_ptr<std::ofstream> current_file_;
    std::chrono::system_clock::time_point start_time_;
    std::chrono::system_clock::time_point logging_start_time_;
    std::chrono::steady_clock::time_point next_sample_time_;
    std::chrono::nanoseconds sample_period_;
    bool use_realtime_priority_ = false;

    std::vector<double> interval_times_;

    // 데이터 관리
    std::deque<DataPoint> data_queue_;
    DataPoint current_data_;

    // 뮤텍스 및 조건 변수
    std::mutex data_mutex_;
    std::mutex file_mutex_;
    std::mutex logging_mutex_;
    std::condition_variable logging_cv_;

    // 스레드
    std::thread logging_thread_;

    // 구독자
    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementData>::SharedPtr displacement_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temperature_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::FanCommand>::SharedPtr fan_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::BackIntention>::SharedPtr intention_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_logging_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_logging_sub_;

    // 발행자
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logging_status_pub_;

    // 타이머
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
