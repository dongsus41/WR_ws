import can
import sys

def send_message(can_id, data_length):
    bus = can.interface.Bus(channel='can0', bustype='socketcan', fd=True)
    data = [i for i in range(data_length)]
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False, is_fd=True)
    try:
        bus.send(msg)
        print(f"송신: CAN FD 프레임 (ID: 0x{can_id:X}, 데이터: {data})")
    except can.CanError as e:
        print("메시지 송신 실패:", e)

def receive_message(timeout=5):
    bus = can.interface.Bus(channel='can0', bustype='socketcan', fd=True)
    print(f"{timeout}초 동안 메시지 대기 중...")
    msg = bus.recv(timeout)
    if msg is None:
        print("수신: 지정 시간 내에 메시지 없음")
    else:
        print(f"수신: CAN FD 프레임 (ID: 0x{msg.arbitration_id:X}, 데이터: {list(msg.data)})")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("사용법: python3 can_test.py [tx|rx] [CAN_ID (16진수)] [tx인 경우: 데이터 길이 (옵션)]")
        sys.exit(1)

    mode = sys.argv[1]
    can_id = int(sys.argv[2], 16)

    if mode == "tx":
        data_length = int(sys.argv[3]) if len(sys.argv) >= 4 else 8
        send_message(can_id, data_length)
    elif mode == "rx":
        receive_message(timeout=5)
    else:
        print("모드 오류: tx 또는 rx를 사용하십시오.")
