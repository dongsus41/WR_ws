// body_control_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/displacement_data.hpp>
#include <wearable_robot_interfaces/msg/joint_state.hpp>
#include <wearable_robot_interfaces/msg/intension_data.hpp>
#include <cmath>

class BodyControlNode : public rclcpp::Node
{
public:
    BodyControlNode() : Node("body_control_node")
    {
        // 변위 센서 데이터 구독
        displacement_sub_ = this->create_subscription<wearable_robot_interfaces::msg::DisplacementData>(
            "displacement_data", 10,
            std::bind(&BodyControlNode::displacement_callback, this, std::placeholders::_1));

        // 의도 인식 데이터 구독
        intention_sub_ = this->create_subscription<wearable_robot_interfaces::msg::IntensionData>(
            "intention_data", 10,
            std::bind(&BodyControlNode::intention_callback, this, std::placeholders::_1));

        // 관절 각도 명령 발행
        joint_state_pub_ = this->create_publisher<wearable_robot_interfaces::msg::JointState>(
            "joint_state_command", 10);

        // 파라미터 초기화
        initializeParams();

        RCLCPP_INFO(this->get_logger(), "Body Control Node has been started");
    }

private:
    void initializeParams()
    {
        // 기구학 파라미터
        this->declare_parameter("shoulder_link_length", 0.3);
        this->declare_parameter("elbow_link_length", 0.25);
        
        // 의도인식 관련 파라미터
        this->declare_parameter("intention_assist.up_angle", 1.2);    // elbow 들어올리기 보조 각도 (rad)
        this->declare_parameter("intention_assist.down_angle", 0); // elbow 내리기 보조 각도 (rad)
        this->declare_parameter("intention_assist.speed", 0.4);       // 보조 동작 속도 (rad/s)

        // 파라미터 로드
        shoulder_link_length_ = this->get_parameter("shoulder_link_length").as_double();
        elbow_link_length_ = this->get_parameter("elbow_link_length").as_double();
        assist_up_angle_ = this->get_parameter("intention_assist.up_angle").as_double();
        assist_down_angle_ = this->get_parameter("intention_assist.down_angle").as_double();
        assist_speed_ = this->get_parameter("intention_assist.speed").as_double();
    }

    void intention_callback(const wearable_robot_interfaces::msg::IntensionData::SharedPtr msg)
    {
        current_intention_ = msg->intention_id;
        intention_timestamp_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Received intention: %d", current_intention_);
    }

    void displacement_callback(const wearable_robot_interfaces::msg::DisplacementData::SharedPtr msg)
    {
        auto joint_msg = wearable_robot_interfaces::msg::JointState();
        joint_msg.header = msg->header;

        // 기본 관절 각도 계산
        double base_r_shoulder = calculateShoulderAngle(msg->displacement0, msg->displacement1);
        double base_l_shoulder = calculateShoulderAngle(msg->displacement2, msg->displacement3);
        double base_r_elbow = calculateElbowAngle(msg->displacement4, msg->displacement5);
        double base_l_elbow = calculateElbowAngle(msg->displacement6, msg->displacement7);

        // 의도인식 결과에 따른 보조 각도 계산
        double assist_angle = calculateAssistAngle();

        // 최종 관절 각도 계산 (기본 각도 + 보조 각도)
        joint_msg.r_shoulder_angle = base_r_shoulder + assist_angle;
        joint_msg.l_shoulder_angle = base_l_shoulder + assist_angle;
        joint_msg.r_elbow_angle = base_r_elbow;
        joint_msg.l_elbow_angle = base_l_elbow;

        // 관절 각도 제한
        joint_msg.r_shoulder_angle = limitAngle(joint_msg.r_shoulder_angle, -M_PI/2, M_PI/2);
        joint_msg.l_shoulder_angle = limitAngle(joint_msg.l_shoulder_angle, -M_PI/2, M_PI/2);
        joint_msg.r_elbow_angle = limitAngle(joint_msg.r_elbow_angle, 0.0, M_PI);
        joint_msg.l_elbow_angle = limitAngle(joint_msg.l_elbow_angle, 0.0, M_PI);

        // 관절 각도 명령 발행
        joint_state_pub_->publish(joint_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Joint angles (with assist) - R_Shoulder: %.2f, L_Shoulder: %.2f, R_Elbow: %.2f, L_Elbow: %.2f",
            joint_msg.r_shoulder_angle,
            joint_msg.l_shoulder_angle,
            joint_msg.r_elbow_angle,
            joint_msg.l_elbow_angle);
    }

    double calculateAssistAngle()
    {
        // 의도인식 시간 체크 (오래된 의도는 무시)
        auto time_diff = this->now() - intention_timestamp_;
        if (time_diff.seconds() > 1.0) {  // 1초 이상 지난 의도는 무시
            return 0.0;
        }

        double target_angle = 0.0;
        switch (current_intention_) {
            case 7:  // 들어올리기
                target_angle = assist_up_angle_;
                break;
            case 8:  // 내리기
                target_angle = assist_down_angle_;
                break;
            default:
                return 0.0;
        }

        // 현재 보조 각도에서 목표 각도로 부드럽게 전환
        if (current_assist_angle_ < target_angle) {
            current_assist_angle_ += assist_speed_;
            if (current_assist_angle_ > target_angle) current_assist_angle_ = target_angle;
        } else if (current_assist_angle_ > target_angle) {
            current_assist_angle_ -= assist_speed_;
            if (current_assist_angle_ < target_angle) current_assist_angle_ = target_angle;
        }

        return current_assist_angle_;
    }

    double calculateShoulderAngle(double sensor1, double sensor2)
    {
        // 간단한 기구학 모델을 사용한 어깨 각도 계산
        double sensor_diff = sensor2 - sensor1;
        double angle = atan2(sensor_diff, shoulder_link_length_);
        return angle;
    }

    double calculateElbowAngle(double sensor1, double sensor2)
    {
        // 간단한 기구학 모델을 사용한 팔꿈치 각도 계산
        double sensor_diff = sensor2 - sensor1;
        double angle = atan2(sensor_diff, elbow_link_length_);
        return angle;
    }

    double limitAngle(double angle, double min, double max)
    {
        if (angle < min) return min;
        if (angle > max) return max;
        return angle;
    }

    rclcpp::Subscription<wearable_robot_interfaces::msg::DisplacementData>::SharedPtr displacement_sub_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::IntensionData>::SharedPtr intention_sub_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::JointState>::SharedPtr joint_state_pub_;
    
    // 기구학 파라미터
    double shoulder_link_length_;
    double elbow_link_length_;
    
    // 의도인식 관련 변수
    uint8_t current_intention_ = 0;
    rclcpp::Time intention_timestamp_ = rclcpp::Time(0, 0);
    double current_assist_angle_ = 0.0;
    
    // 보조 동작 파라미터
    double assist_up_angle_;    // 들어올리기 보조 각도
    double assist_down_angle_;  // 내리기 보조 각도
    double assist_speed_;       // 보조 동작 속도
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyControlNode>());
    rclcpp::shutdown();
    return 0;
}