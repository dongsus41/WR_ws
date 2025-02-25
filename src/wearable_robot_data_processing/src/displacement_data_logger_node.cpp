// displacement_data_logger_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/displacement_raw_data.hpp>
#include <fstream>
#include <ctime>
#include <iomanip>

class DisplacementDataLogger : public rclcpp::Node
{
public:
    DisplacementDataLogger() : Node("displacement_data_logger")
    {
        // CSV 파일 생성 및 헤더 작성
        setupCSVFile();

        // displacement_raw_data 토픽 구독
        subscription_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementRawData>(
            "displacement_raw_data", 10,
            std::bind(&DisplacementDataLogger::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Displacement Data Logger has started. Saving to: %s", csv_filename_.c_str());
    }

    ~DisplacementDataLogger()
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
        ss << "displacement_data_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".csv";
        csv_filename_ = ss.str();

        // CSV 파일 열기
        csv_file_.open(csv_filename_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
            return;
        }

        // CSV 헤더 작성
        csv_file_ << "timestamp,can_id";
        for (int i = 0; i < 10; ++i) {
            csv_file_ << ",displacement" << i;
        }
        csv_file_ << std::endl;
    }

    void topic_callback(const wearable_robot_interfaces::msg::DisplacementRawData::SharedPtr msg)
    {
        if (!csv_file_.is_open()) return;

        // 타임스탬프를 초 단위로 변환
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // CSV 데이터 작성
        csv_file_ << std::fixed << std::setprecision(6)
                 << timestamp << ","
                 << msg->can_id << ","
                 << msg->displacement0 << ","
                 << msg->displacement1 << ","
                 << msg->displacement2 << ","
                 << msg->displacement3 << ","
                 << msg->displacement4 << ","
                 << msg->displacement5 << ","
                 << msg->displacement6 << ","
                 << msg->displacement7 << ","
                 << msg->displacement8 << ","
                 << msg->displacement9
                 << std::endl;

        // 파일을 즉시 저장 (버퍼링 방지)
        csv_file_.flush();
    }

    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementRawData>::SharedPtr subscription_;
    std::ofstream csv_file_;
    std::string csv_filename_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplacementDataLogger>());
    rclcpp::shutdown();
    return 0;
}
