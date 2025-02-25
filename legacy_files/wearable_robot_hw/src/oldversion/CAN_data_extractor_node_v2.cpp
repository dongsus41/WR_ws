#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/can_data.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <unistd.h>
#include <array>
#include <thread>
#include <atomic>
#include <iomanip>
#include <sstream>

class CANDataExtractor : public rclcpp::Node
{
public:
    CANDataExtractor() : Node("CAN_data_extractor"), should_run_(true)
    {
        publisher_100_ = this->create_publisher<wearable_robot_interfaces::msg::CANData>("CAN_data_100", 10);
        publisher_401_ = this->create_publisher<wearable_robot_interfaces::msg::CANData>("CAN_data_401", 10);
        setup_CAN();
        // CAN 읽기 스레드 시작
        CAN_read_thread_ = std::thread(&CANDataExtractor::CAN_read_loop, this);
    }

    ~CANDataExtractor()
    {
        should_run_.store(false);
        if (CAN_read_thread_.joinable()) {
            CAN_read_thread_.join();
        }
        if (CAN_socket_ >= 0) {
            close(CAN_socket_);
        }
    }

private:
    rclcpp::Publisher<wearable_robot_interfaces::msg::CANData>::SharedPtr publisher_100_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::CANData>::SharedPtr publisher_401_;
    int CAN_socket_ = -1;
    std::thread CAN_read_thread_;
    std::atomic<bool> should_run_;

    void setup_CAN()
    {
        // SocketCAN 소켓 생성 및 바인딩
        const char* ifname = "can0";
        CAN_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (CAN_socket_ < 0) {
            throw std::runtime_error("Error creating CAN socket");
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, ifname);
        if (ioctl(CAN_socket_, SIOCGIFINDEX, &ifr) < 0) {
            throw std::runtime_error("Error getting CAN interface index");
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(CAN_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            throw std::runtime_error("Error binding CAN socket");
        }

        // CAN FD 프레임 지원 설정
        int enable_canfd = 1;
        if (setsockopt(CAN_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
            throw std::runtime_error("Error setting CAN FD support");
        }

        RCLCPP_INFO(this->get_logger(), "CAN interface setup completed");
    }


    void process_CAN_100(const struct canfd_frame& frame)
    {
        auto message = wearable_robot_interfaces::msg::CANData();
        message.header.stamp = this->now();
        message.can_id = 0x100;

        // 10개의 변위센서 데이터 (각 2바이트)
        for (int i = 0; i < 10; ++i) {
            uint16_t value = *reinterpret_cast<const uint16_t*>(&frame.data[2*i]);
            message.data[i] = value;
        }

        // 4개의 IMU 데이터 (각각 roll, pitch, yaw - 2바이트씩)
        for (int i = 10; i < 22; ++i) {
            int16_t value = *reinterpret_cast<const int16_t*>(&frame.data[2*i]);
            message.data[i] = value;
        }

        publisher_100_->publish(message);
    }

    void process_CAN_401(const struct canfd_frame& frame)
    {
        auto message = wearable_robot_interfaces::msg::CANData();
        message.header.stamp = this->now();
        message.can_id = 0x401;

        // 처음 8바이트는 1바이트씩 처리
        for (int i = 0; i < 8; ++i) {
            message.data[i] = static_cast<uint8_t>(frame.data[i]);
        }

        // 온도 데이터 처리 (8-15바이트, 14비트 데이터)
        for (int i = 0; i < 4; ++i) {  // 4개의 2바이트 묶음 처리
            int data_index = 8 + i;     // message.data에서의 인덱스
            int byte_index = 8 + i*2;   // frame.data에서의 인덱스

            // 2바이트를 하나의 16비트 값으로 조합 (리틀 엔디안)
            uint16_t raw_value = static_cast<uint16_t>(frame.data[byte_index]) |
                                (static_cast<uint16_t>(frame.data[byte_index + 1]) << 8);

            // 14비트 마스킹 및 부호 처리
            int16_t temp_value = raw_value & 0x3FFF;
            if (temp_value & 0x2000) {
                temp_value |= 0xC000;
            }

            message.data[data_index] = temp_value;
        }

        // 마지막 8바이트 처리 (16-23바이트를 2바이트씩)
        for (int i = 0; i < 4; ++i) {  // 4개의 2바이트 묶음 처리
            int data_index = 12 + i;    // message.data에서의 인덱스
            int byte_index = 16 + i*2;  // frame.data에서의 인덱스

            uint16_t value = static_cast<uint16_t>(frame.data[byte_index]) |
                            (static_cast<uint16_t>(frame.data[byte_index + 1]) << 8);

            message.data[data_index] = value;
        }

        // 남은 배열 요소들은 0으로 설정
        for (int i = 16; i < 24; ++i) {
            message.data[i] = 0;
        }

        publisher_401_->publish(message);
    }


    void CAN_read_loop()
    {
        while (rclcpp::ok() && should_run_.load()) {
            struct canfd_frame frame;
            ssize_t nbytes = read(CAN_socket_, &frame, sizeof(struct canfd_frame));

            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame");
                continue;
            }

            // 표준 CAN ID 마스킹 (11비트)
            uint32_t CAN_id = frame.can_id & 0x7FF;

            switch(CAN_id) {
                case 0x100:
                    if (frame.len == 48) {
                        process_CAN_100(frame);
                    }
                    break;
                case 0x401:
                    if (frame.len == 24) {
                        process_CAN_401(frame);
                    }
                    break;
                default:
                    RCLCPP_DEBUG(this->get_logger(), "Unhandled CAN ID: 0x%X", CAN_id);
                    break;
            }
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANDataExtractor>());
    rclcpp::shutdown();
    return 0;
}
