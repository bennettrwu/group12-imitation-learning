#!/usr/bin/env python3
"""
Transcribe ROS1 bags into a structured dataset (RGB + steering only).

Collect data in simulator using:
  rosbag record /ackermann_cmd /oak/rgb/camera_info /oak/rgb/image_raw /oak/rgb/image_raw/compressed

Usage:
  python3 bag_to_dataset.py <bag_or_dir> <output_dir> [--max-dt 0.1]

Output per bag:
  <output_dir>/<sequence>/
    front_camera/rgb/000000.jpg
    controls/data.csv               (steering_angle, speed, acceleration, jerk, steering_angle_velocity)
    calibration/rgb_camera_info.yaml
    calibration/timestamps.csv      (frame_idx, rgb_ts, ctrl_ts)

RGB source priority: /oak/rgb/image_raw/compressed  (falls back to /oak/rgb/image_raw)
Steering source:     /ackermann_cmd  (AckermannDriveStamped)
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
import rosbag

TOPIC_RGB_COMPRESSED = '/oak/rgb/image_raw/compressed'
TOPIC_RGB_RAW        = '/oak/rgb/image_raw'
TOPIC_CAM_INFO       = '/oak/rgb/camera_info'
TOPIC_ACKERMANN      = '/ackermann_cmd'

REQUIRED_TOPICS = {TOPIC_ACKERMANN}   # RGB presence is checked after source selection


def open_reader(bag_path: str) -> rosbag.Bag:
    return rosbag.Bag(bag_path, 'r')


def get_topic_types(reader: rosbag.Bag) -> dict:
    return {name: info.msg_type for name, info in reader.get_type_and_topic_info().topics.items()}


def decode_compressed_rgb(data: bytes) -> np.ndarray:
    arr = np.frombuffer(data, np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def decode_raw_rgb(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image message to a BGR numpy array."""
    dtype = np.uint8
    img = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width, -1)
    if msg.encoding in ('rgb8', 'rgb16'):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'mono8':
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # bgr8 and bayer variants can be used as-is or handled downstream
    return img


def camera_info_to_dict(msg) -> dict:
    return {
        'image_width':  msg.width,
        'image_height': msg.height,
        'camera_name':  msg.header.frame_id,
        'camera_matrix':             {'rows': 3, 'cols': 3, 'data': list(msg.K)},
        'distortion_model':          msg.distortion_model,
        'distortion_coefficients':   {'rows': 1, 'cols': len(msg.D), 'data': list(msg.D)},
        'rectification_matrix':      {'rows': 3, 'cols': 3, 'data': list(msg.R)},
        'projection_matrix':         {'rows': 3, 'cols': 4, 'data': list(msg.P)},
    }


def nearest(timestamps: list, query_ns: int) -> tuple:
    """Return (index, timestamp_ns) of closest entry to query_ns."""
    if not timestamps:
        return -1, -1
    idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - query_ns))
    return idx, timestamps[idx]


def collect_messages(reader: rosbag.Bag, topics: list) -> dict:
    """Read all messages into per-topic lists of (timestamp_ns, msg)."""
    buckets: dict = {t: [] for t in topics}
    for topic, msg, ts in reader.read_messages(topics=topics):
        buckets[topic].append((ts.to_nsec(), msg))
    return buckets


def process_bag(bag_path: str, output_dir: str, max_dt_ns: int):
    bag_path = str(bag_path)
    seq_name = Path(bag_path).stem
    out      = Path(output_dir) / seq_name

    reader      = open_reader(bag_path)
    topic_types = get_topic_types(reader)

    missing = REQUIRED_TOPICS - set(topic_types)
    if missing:
        print(f"  [SKIP] missing required topics: {missing}")
        return

    # Determine RGB source
    if TOPIC_RGB_COMPRESSED in topic_types:
        rgb_topic  = TOPIC_RGB_COMPRESSED
        compressed = True
        print(f"  RGB source: {TOPIC_RGB_COMPRESSED} (compressed)")
    elif TOPIC_RGB_RAW in topic_types:
        rgb_topic  = TOPIC_RGB_RAW
        compressed = False
        print(f"  RGB source: {TOPIC_RGB_RAW} (raw)")
    else:
        print(f"  [SKIP] no RGB topic found (need {TOPIC_RGB_COMPRESSED} or {TOPIC_RGB_RAW})")
        return

    topics_to_read = [rgb_topic, TOPIC_ACKERMANN, TOPIC_CAM_INFO]
    print(f"  Reading messages...")
    buckets    = collect_messages(reader, topics_to_read)
    reader.close()

    rgb_msgs  = buckets.get(rgb_topic,       [])
    ctrl_msgs = buckets.get(TOPIC_ACKERMANN, [])
    cam_msgs  = buckets.get(TOPIC_CAM_INFO,  [])

    if not rgb_msgs:
        print(f"  [SKIP] RGB topic present but empty")
        return
    if not ctrl_msgs:
        print(f"  [SKIP] {TOPIC_ACKERMANN} present but empty")
        return

    rgb_ts  = [t for t, _ in rgb_msgs]
    ctrl_ts = [t for t, _ in ctrl_msgs]

    # Create output directories
    (out / 'front_camera' / 'rgb').mkdir(parents=True, exist_ok=True)
    (out / 'controls').mkdir(parents=True, exist_ok=True)
    (out / 'calibration').mkdir(parents=True, exist_ok=True)

    # Save calibration
    if cam_msgs:
        info = camera_info_to_dict(cam_msgs[0][1])
        with open(out / 'calibration' / 'rgb_camera_info.yaml', 'w') as f:
            yaml.dump(info, f)
    else:
        print(f"  [WARN] {TOPIC_CAM_INFO} not found — calibration file will be skipped")

    # Synchronize: RGB frames as reference clock, match nearest steering command
    ts_rows   = []
    ctrl_rows = []
    frame_idx = 0
    print(f"  Synchronizing {len(rgb_msgs)} RGB frames...")
    for rgb_ts_i, rgb_msg in rgb_msgs:
        ci, cts = nearest(ctrl_ts, rgb_ts_i)
        if abs(cts - rgb_ts_i) > max_dt_ns:
            continue

        name = f'{frame_idx:06d}'

        # RGB
        if compressed:
            img = decode_compressed_rgb(bytes(rgb_msg.data))
        else:
            img = decode_raw_rgb(rgb_msg)
        if img is None:
            continue
        cv2.imwrite(str(out / 'front_camera' / 'rgb' / f'{name}.jpg'), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Collect steering row (written to CSV after the loop)
        # breakpoint()
        drive = ctrl_msgs[ci][1]
        ctrl_rows.append([
            cts,
            drive.steering_angle,
            drive.steering_angle_velocity,
            drive.speed,
            drive.acceleration,
            drive.jerk,
        ])
        ts_rows.append([frame_idx, rgb_ts_i, cts])
        frame_idx += 1

    # Controls CSV
    with open(out / 'controls' / 'data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_ns',
                         'steering_angle_rad', 'steering_angle_velocity_rad_s',
                         'speed_mps', 'acceleration_mps2', 'jerk_mps3'])
        writer.writerows(ctrl_rows)

    # Timestamps index
    with open(out / 'calibration' / 'timestamps.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_idx', 'rgb_ts', 'ctrl_ts'])
        writer.writerows(ts_rows)

    print(f"  Saved {frame_idx} synchronized frames → {out}")


def main():
    parser = argparse.ArgumentParser(description='Convert ROS1 bags to dataset format')
    parser.add_argument('input',      help='Bag file or directory of bags')
    parser.add_argument('output_dir', help='Output dataset directory')
    parser.add_argument('--max-dt',   type=float, default=0.1,
                        help='Max sync time difference in seconds (default: 0.1)')
    args = parser.parse_args()

    max_dt_ns = int(args.max_dt * 1e9)
    inp = Path(args.input)

    if inp.is_dir():
        bags = sorted(inp.glob('*.bag'))
    elif inp.suffix == '.bag':
        bags = [inp]
    else:
        bags = []

    if not bags:
        print(f"No .bag files found at {inp}")
        sys.exit(1)

    for bag in bags:
        print(f"\nProcessing: {bag.name}")
        try:
            process_bag(bag, args.output_dir, max_dt_ns)
        except Exception as e:
            print(f"  [ERROR] {e}")


if __name__ == '__main__':
    main()