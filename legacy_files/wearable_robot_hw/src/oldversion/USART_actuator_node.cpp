#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <libserial/SerialPort.h>
#include <chrono>
#include <thread>

class USARTActuatorNode : public rclcpp::Node
{
public:
    USARTActuatorNode() : Node("USART_actuator_node"), serial_port_()
    {
        // // YAML 파라미터 설정에서 값을 가져옴
        // this->declare_parameter("actuator_port", "/dev/ttyACM0");
        // this->declare_parameter("actuator_baudrate", 1000000);
        // port_ = this->get_parameter("actuator_port").as_string();
        // baudrate_ = this->get_parameter("actuator_baudrate").as_int();

        // 온도 데이터 발행자 생성
        temp_pub_ = this->create_publisher<wearable_robot_interfaces::msg::TemperatureData>("temperature_data", 10);

        // PWM 제어 구독자 생성
        pwm_sub_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10, std::bind(&USARTActuatorNode::pwmCallback, this, std::placeholders::_1));

        // 팬 제어 구독자 생성
        fan_sub_ = this->create_subscription<wearable_robot_interfaces::msg::FanCommand>(
            "fan_command", 10, std::bind(&USARTActuatorNode::fanCallback, this, std::placeholders::_1));

        // 타이머 설정 (온도 읽기용, 100ms 간격)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&USARTActuatorNode::timerCallback, this));

        // 시리얼 포트 설정 및 연결
        setupSerialPort();
    }


private:
    void setupSerialPort()
    {
        try {
            serial_port_.Open("/dev/ttyACM0");
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_1000000);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
            rclcpp::shutdown();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error setting up serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }

    // 온도 데이터 읽기 및 발행 콜백 함수
    void timerCallback()
    {
        if (!serial_port_.IsOpen()) {
            RCLCPP_WARN(this->get_logger(), "Serial port is not open. Attempting to reconnect...");
            setupSerialPort();
            return;
        }

        try {
            std::string data;
            serial_port_.ReadLine(data);
            float temp = std::stof(data)/100;

            // 온도센서 추가시 아래 코드 적용
            // std::array<float, 4> temps;
            // for (int i = 0; i < 4; i++) {
            //     std::string data;
            //     serial_port_.ReadLine(data, '\n', 100);
            //     temps[i] = std::stof(data);
            // }

            auto msg = wearable_robot_interfaces::msg::TemperatureData();
            msg.header.stamp = this->now();
            msg.temperature[0] = temp;

            // 온도센서 추가시 아래 코드 적용
            // for (int i = 0; i < 4; i++) {
            //     msg.temperature[i] = temps[i];
            // }

            temp_pub_->publish(msg);
        } catch (const LibSerial::ReadTimeout& e) {
            RCLCPP_WARN(this->get_logger(), "Timeout while reading temperature data");
        } catch (const std::invalid_argument& e) {
            RCLCPP_WARN(this->get_logger(), "Invalid temperature data received");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading temperature data: %s", e.what());
        }
    }

    // PWM 제어 콜백 함수
    void pwmCallback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        if (!serial_port_.IsOpen()) {
            RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
            return;
        }

        try {
            std::string command = "A " + std::to_string(msg->pwm[0]) + "\n";
            serial_port_.Write(command);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending PWM command: %s", e.what());
        }
    }

    // 팬 제어 콜백 함수
    void fanCallback(const wearable_robot_interfaces::msg::FanCommand::SharedPtr msg)
    {
        auto start = this->now();
            RCLCPP_INFO(this->get_logger(), "Received fan command at %ld.%ld",
                        start.seconds(), start.nanoseconds());

        if (!serial_port_.IsOpen()) {
            RCLCPP_WARN(this->get_logger(), "Serial port is not open.");
            return;
        }

        try {
            std::string command = msg->fan[0] ? "F1\n" : "F0\n";
            serial_port_.Write(command);

            std::string response;
            serial_port_.ReadLine(response, '\n', 100);  // 100ms 타임아웃

            auto end = this->now();
            RCLCPP_INFO(this->get_logger(), "Command executed. Total time: %f ms",
                        (end - start).nanoseconds() / 1e6);
            RCLCPP_INFO(this->get_logger(), "Hardware response: %s", response.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending fan command: %s", e.what());
        }
    }

    LibSerial::SerialPort serial_port_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_pub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::FanCommand>::SharedPtr fan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string port_;
    int baudrate_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<USARTActuatorNode>());
    rclcpp::shutdown();
    return 0;
}

