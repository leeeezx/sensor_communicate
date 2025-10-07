import csv
import sys
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

def dual_topic_to_csv(bag_path: str, force_topic: str, pose_topic: str, output_csv: str):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with Reader(bag_path) as reader:
        with open(output_csv, 'w', newline='') as csvfile:
            writer = None
            
            for connection, timestamp, rawdata in reader.messages():
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                # 首次写入时创建表头
                if writer is None:
                    fieldnames = [
                        'timestamp_sec',
                        'topic',
                        'force_z',
                        'tau_fx', 'tau_fy', 'tau_fz',
                        'tau_mx', 'tau_my', 'tau_mz'
                    ]
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                
                if connection.topic == force_topic:
                    # 处理力传感器数据（假设是 Float64）
                    writer.writerow({
                        'timestamp_sec': timestamp / 1e9,
                        'topic': force_topic,
                        'force_z': msg.data,
                    })
                
                elif connection.topic == pose_topic:
                    data = list(msg.data)
                    data += [0.0] * (6 - len(data))
                    writer.writerow({
                        'timestamp_sec': timestamp / 1e9,
                        'topic': pose_topic,
                        'tau_fx': data[0],
                        'tau_fy': data[1],
                        'tau_fz': data[2],
                        'tau_mx': data[3],
                        'tau_my': data[4],
                        'tau_mz': data[5],
                    })

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 ros_bag_record.py <bag_folder> <force_topic> <pose_topic> <output.csv>")
        print("Example: python3 ros_bag_record.py ./bag_data force_sensor_z realtime_robot_pose output.csv")
        sys.exit(1)
    
    bag_folder = sys.argv[1]
    force_topic = sys.argv[2]
    pose_topic = sys.argv[3]
    output_csv = sys.argv[4]
    
    dual_topic_to_csv(bag_folder, force_topic, pose_topic, output_csv)
    print(f"Saved force and pose data to {output_csv}")