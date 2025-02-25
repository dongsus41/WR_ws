// CAN_setting_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <array>

class CANSettingNode : public rclcpp::Node
{
public:
    CANSettingNode() : Node("CAN_setting_node")
    {
        setup_CAN();
    }

private:
    void setup_CAN()
    {
        // CAN 설정 명령어 실행
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
                rclcpp::shutdown();  // 설정 실패시 종료
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully executed: %s", cmd);
            }
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface setup completed");
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANSettingNode>());
    rclcpp::shutdown();
    return 0;
}
