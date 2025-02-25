// expired Version

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
        publisher_ = this->create_publisher<wearable_robot_interfaces::msg::CANData>("CAN_data", 10);
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
    rclcpp::Publisher<wearable_robot_interfaces::msg::CANData>::SharedPtr publisher_;
    int CAN_socket_ = -1;
    std::thread CAN_read_thread_;
    std::atomic<bool> should_run_;

    void setup_CAN()
    {
        std::array<const char*, 5> setup_commands = {
            "sudo modprobe can",
            "sudo modprobe can_raw",
            "sudo modprobe mttcan",
            "sudo ip link set can0 down",
            "sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 berr-reporting on fd on"
        };

        for (const auto& cmd : setup_commands) {
            int result = std::system(cmd);
            if (result != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute command: %s", cmd);
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully executed: %s", cmd);
            }
        }

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

    void CAN_read_loop()
    {
        while (rclcpp::ok() && should_run_.load()) {
            struct canfd_frame frame;
            ssize_t nbytes = read(CAN_socket_, &frame, sizeof(struct canfd_frame));

            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame");
                continue;
            }

            if (static_cast<size_t>(nbytes) < sizeof(struct canfd_frame)) {
                RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame received");
                continue;
            }

            // 표준 CAN ID 마스킹 (11비트)
            uint32_t CAN_id = frame.can_id & 0x7FF;

            if (CAN_id == 0x100 && frame.len == 48) {
                auto message = wearable_robot_interfaces::msg::CANData();
                message.header.stamp = this->now();
                message.can_id = CAN_id;

                std::stringstream ss;
                for (int i = 0; i < 10; ++i) {
                    uint16_t value = *reinterpret_cast<uint16_t*>(&frame.data[2*i]);
                    message.data[i] = value;
                    ss << value << "/t";
                }
                for (int i = 10; i < 22; ++i) {
                    int16_t value = *reinterpret_cast<int16_t*>(&frame.data[2*i]);
                    message.data[i] = value;
                    ss << value << "/t";
                }

                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published CAN data: %s", ss.str().c_str());
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
