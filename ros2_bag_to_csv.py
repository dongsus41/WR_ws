
#!/usr/bin/env python3

import sqlite3
import os
import argparse
import yaml
import pandas as pd
import numpy as np
from datetime import datetime
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def connect_to_ros2_bag(bag_path):
    """ROS2 bag 파일(SQLite DB)에 연결"""
    db_path = os.path.join(bag_path, 'ros2_bag.db3')
    if not os.path.exists(db_path):
        # bag이 단일 파일이 아닌 디렉토리 형태인지 확인
        metadata_path = os.path.join(bag_path, 'metadata.yaml')
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)
                # 데이터베이스 파일명이 다를 수 있음
                if 'rosbag2_bagfile_information' in metadata:
                    db_name = metadata['rosbag2_bagfile_information'].get('relative_file_paths', ['ros2_bag.db3'])[0]
                    db_path = os.path.join(bag_path, db_name)

    if not os.path.exists(db_path):
        raise FileNotFoundError(f"ROS2 bag 데이터베이스 파일을 찾을 수 없습니다: {db_path}")

    return sqlite3.connect(db_path)


def get_topics_from_ros2_bag(conn):
    """ROS2 bag에서 사용 가능한 토픽과 메시지 타입 가져오기"""
    cursor = conn.cursor()
    cursor.execute("SELECT name, type FROM topics")
    topics = {name: type for name, type in cursor.fetchall()}
    return topics


def get_topic_id(conn, topic_name):
    """토픽 이름으로 토픽 ID 가져오기"""
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    result = cursor.fetchone()
    if result is None:
        return None
    return result[0]


def extract_topic_data_to_dataframe(conn, topic_name):
    """토픽 데이터를 추출하여 DataFrame으로 변환"""
    # 토픽 ID와 메시지 타입 가져오기
    topic_id = get_topic_id(conn, topic_name)
    if topic_id is None:
        print(f"토픽 '{topic_name}'이 bag 파일에 존재하지 않습니다.")
        return None

    # 토픽 메시지 타입 가져오기
    cursor = conn.cursor()
    cursor.execute("SELECT type FROM topics WHERE id = ?", (topic_id,))
    msg_type = cursor.fetchone()[0]
    msg_class = get_message(msg_type)

    # 모든 메시지 가져오기
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,))
    messages = cursor.fetchall()

    if not messages:
        print(f"토픽 '{topic_name}'에 메시지가 없습니다.")
        return None

    # 메시지를 처리하여 데이터 추출
    data = []

    for timestamp, msg_data in messages:
        # 타임스탬프를 초 단위로 변환
        timestamp_sec = timestamp / 10**9

        # 메시지 역직렬화
        msg = deserialize_message(msg_data, msg_class)

        # 메시지 타입에 따라 데이터 추출
        row = {'timestamp': timestamp_sec}

        # 메시지 타입별 처리
        if 'DisplacementData' in msg_type:
            if hasattr(msg, 'displacement') and len(msg.displacement) > 1:
                row['displacement_1'] = float(msg.displacement[1])

        elif 'TemperatureData' in msg_type:
            if hasattr(msg, 'temperature') and len(msg.temperature) > 5:
                row['temperature_4'] = float(msg.temperature[4])
                row['temperature_5'] = float(msg.temperature[5])

        elif 'ActuatorCommand' in msg_type:
            if hasattr(msg, 'pwm') and len(msg.pwm) > 5:
                row['pwm_4'] = int(msg.pwm[4])
                row['pwm_5'] = int(msg.pwm[5])

        # 추가할 수 있는 다른 메시지 타입들
        # (target_temperature 토픽이 어떤 형식인지에 따라 처리)
        elif 'FanCommand' in msg_type:
            if hasattr(msg, 'fan') and len(msg.fan) > 5:
                row['fan_4'] = 1 if msg.fan[4] else 0
                row['fan_5'] = 1 if msg.fan[5] else 0

        # 다른 메시지 타입에 대한 처리 추가 (필요시)
        # else:
        #     # 기본 처리 또는 로그 기록
        #     print(f"처리되지 않은 메시지 타입: {msg_type}")

        # 추출된 데이터가 있으면 추가
        if len(row) > 1:  # timestamp 외에 다른 데이터가 있으면
            data.append(row)

    # DataFrame 생성
    if data:
        df = pd.DataFrame(data)
        print(f"토픽 '{topic_name}'에서 {len(df)}개의 메시지를 추출했습니다.")
        return df
    else:
        print(f"토픽 '{topic_name}'에서 추출된 데이터가 없습니다.")
        return None


def merge_dataframes_by_timestamp(dataframes, interpolation_method='nearest'):
    """
    여러 DataFrame을 타임스탬프를 기준으로 병합

    Args:
        dataframes: {column_prefix: dataframe} 형식의 딕셔너리
        interpolation_method: 데이터 보간 방법 ('linear', 'nearest', 'cubic' 등)

    Returns:
        병합된 DataFrame
    """
    if not dataframes:
        return None

    # 모든 DataFrame에 대한 타임스탬프 범위 찾기
    all_timestamps = set()
    for df in dataframes.values():
        if df is not None and 'timestamp' in df.columns:
            all_timestamps.update(df['timestamp'].values)

    if not all_timestamps:
        print("모든 DataFrame에 타임스탬프 데이터가 없습니다.")
        return None

    # 타임스탬프 정렬
    all_timestamps = sorted(all_timestamps)

    # 기준 DataFrame 생성
    merged_df = pd.DataFrame({'timestamp': all_timestamps})

    # 각 DataFrame을 타임스탬프를 기준으로 병합
    for prefix, df in dataframes.items():
        if df is not None and 'timestamp' in df.columns:
            # 타임스탬프 열 외의 다른 열
            data_columns = [col for col in df.columns if col != 'timestamp']

            # 각 데이터 열에 대해 타임스탬프 기준으로 보간
            for col in data_columns:
                # 인덱스를 타임스탬프로 설정하여 보간 준비
                temp_df = df[['timestamp', col]].set_index('timestamp')

                # 새로운 타임스탬프에 대한 값 보간
                interp_series = np.interp(
                    all_timestamps,
                    df['timestamp'].values,
                    df[col].values,
                    left=np.nan,
                    right=np.nan
                )

                # 병합된 DataFrame에 보간된 값 추가
                merged_df[col] = interp_series

    return merged_df


def process_ros2_bag_to_single_csv(bag_path, output_file, topic_list=None):
    """
    ROS2 bag 파일에서 지정된 토픽 데이터를 추출하여 시간 동기화 후 단일 CSV로 저장

    Args:
        bag_path: ROS2 bag 파일 또는 디렉토리 경로
        output_file: 출력 CSV 파일 경로
        topic_list: 처리할 토픽 목록 (None이면 기본 토픽 사용)
    """
    # ROS2 bag DB 연결
    conn = connect_to_ros2_bag(bag_path)

    # 사용 가능한 토픽 확인
    available_topics = get_topics_from_ros2_bag(conn)
    print(f"사용 가능한 토픽: {list(available_topics.keys())}")

    # 기본 토픽 목록 설정 (사용자가 지정하지 않은 경우)
    if topic_list is None:
        # 기본 토픽: 변위, 온도, 목표 온도, PWM 상태
        topic_list = [
            '/displacement_data',
            '/temperature_data',
            '/target_temperature',
            '/actuator_command'
        ]
        # 사용 가능한 토픽 중에서만 사용
        topic_list = [t for t in topic_list if t in available_topics]

    print(f"처리할 토픽: {topic_list}")

    # 각 토픽별 데이터 추출
    dataframes = {}

    for topic in topic_list:
        if topic in available_topics:
            df = extract_topic_data_to_dataframe(conn, topic)
            if df is not None:
                # 토픽 이름에서 '/'를 제거하고 접두사로 사용
                prefix = topic.replace('/', '_')[1:]
                dataframes[prefix] = df

    # 연결 종료
    conn.close()

    if not dataframes:
        print("추출된 데이터가 없습니다.")
        return

    # DataFrame 병합
    merged_df = merge_dataframes_by_timestamp(dataframes)

    if merged_df is None:
        print("데이터 병합에 실패했습니다.")
        return

    # 인접한 타임스탬프 간 차이가 너무 큰 경우 NaN으로 설정
    # (선택적 단계, 필요에 따라 주석 해제)
    # max_time_diff = 0.1  # 100ms
    # merged_df['timestamp_diff'] = merged_df['timestamp'].diff()
    # cols = [col for col in merged_df.columns if col != 'timestamp' and col != 'timestamp_diff']
    # for col in cols:
    #     merged_df.loc[merged_df['timestamp_diff'] > max_time_diff, col] = np.nan
    # merged_df.drop('timestamp_diff', axis=1, inplace=True)

    # NaN 값 처리 (선택적 단계, 필요에 따라 주석 해제)
    # merged_df = merged_df.fillna(method='ffill')  # 앞의 값으로 채우기

    # CSV로 저장
    merged_df.to_csv(output_file, index=False, float_format='%.6f')
    print(f"데이터가 성공적으로 {output_file}에 저장되었습니다. 총 {len(merged_df)}개의 행이 생성되었습니다.")


def main():
    parser = argparse.ArgumentParser(description='ROS2 bag 파일에서 특정 데이터를 추출하여 시간 동기화 후 단일 CSV 파일로 저장')
    parser.add_argument('bag_path', help='변환할 ROS2 bag 파일 또는 디렉토리 경로')
    parser.add_argument('-o', '--output', default='combined_data.csv', help='출력 CSV 파일 경로')
    parser.add_argument('-t', '--topics', nargs='+', help='변환할 토픽 목록 (지정하지 않으면 기본 토픽 사용)')

    args = parser.parse_args()

    process_ros2_bag_to_single_csv(args.bag_path, args.output, args.topics)


if __name__ == '__main__':
    main()
