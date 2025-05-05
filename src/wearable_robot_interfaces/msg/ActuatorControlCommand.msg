std_msgs/Header header
uint8[] pwm            # 6채널 PWM 값
uint8[] fan            # 6채널 팬 상태
uint8[] pid_enable     # 6채널 PID 제어 활성화 상태
uint16[] target_temp   # 6채널 목표 온도 (0.25도 단위)
