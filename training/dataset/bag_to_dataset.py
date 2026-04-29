#!/usr/bin/env python3
"""
Transcribe ROS2 bags into a structured dataset.

Usage:
  python3 bag_to_dataset.py <bag_or_dir> <output_dir> [--max-dt 0.1]

Output per bag:
  <output_dir>/<sequence>/
    front_camera/rgb/000000.jpg
    front_camera/depth/000000.png   (16-bit PNG, millimeters)
    lidar/000000.bin                (N x 4 float32: x y z intensity)
    gnss/data.csv
    controls/data.csv               (steering, speed, accel, brake)
    calibration/camera_info.yaml
    calibration/timestamps.csv      (frame_idx, lidar_ts, rgb_ts, depth_ts, gnss_ts, ctrl_ts)
"""

import argparse
import csv
import math
import os
import struct
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC_RGB        = '/oak/rgb/image_raw/compressed'
TOPIC_DEPTH      = '/oak/stereo/image_raw/compressedDepth'
TOPIC_LIDAR      = '/ouster/points'
TOPIC_NAVSATFIX  = '/navsatfix'
TOPIC_PVTGEODETIC= '/pvtgeodetic'
TOPIC_INSNAVGEOD = '/insnavgeod'
TOPIC_CAM_RGB    = '/oak/rgb/camera_info'
TOPIC_CAM_DEPTH  = '/oak/stereo/camera_info'
TOPIC_STEER      = '/pacmod/steering_rpt'
TOPIC_SPEED      = '/pacmod/vehicle_speed_rpt'
TOPIC_ACCEL      = '/pacmod/accel_rpt'
TOPIC_BRAKE      = '/pacmod/brake_rpt'
TOPIC_SHIFT      = '/pacmod/shift_rpt'

SHIFT_NAMES = {0: 'PARK', 1: 'REVERSE', 2: 'NEUTRAL', 3: 'FORWARD', 5: 'BETWEEN', 6: 'ERROR', 7: 'NONE'}

REQUIRED_TOPICS  = {TOPIC_RGB, TOPIC_DEPTH, TOPIC_LIDAR}


def detect_storage_id(bag_path: str) -> str:
    p = Path(bag_path)
    if list(p.glob('*.mcap')):
        return 'mcap'
    return 'sqlite3'


def open_reader(bag_path: str):
    storage_id = detect_storage_id(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_topic_types(reader) -> dict:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def decode_compressed_rgb(data: bytes) -> np.ndarray:
    arr = np.frombuffer(data, np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def decode_compressed_depth(data: bytes) -> np.ndarray:
    arr = np.frombuffer(data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is not None and img.dtype == np.uint16:
        return img
    # Skip 12-byte ConfigHeader (older image_transport format)
    arr = np.frombuffer(data[12:], np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)


def pointcloud2_to_xyzi(msg) -> np.ndarray:
    fields = {f.name: f.offset for f in msg.fields}
    step   = msg.point_step
    n      = msg.width * msg.height
    raw    = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    out = np.empty((n, 4), dtype=np.float32)
    for col, name in enumerate(['x', 'y', 'z', 'intensity']):
        off = fields[name]
        out[:, col] = np.frombuffer(
            np.ascontiguousarray(raw[:, off:off + 4]).tobytes(), dtype=np.float32
        )

    valid = np.isfinite(out[:, :3]).all(axis=1) & (np.linalg.norm(out[:, :3], axis=1) > 0)
    return out[valid]


def camera_info_to_dict(msg) -> dict:
    return {
        'image_width':  msg.width,
        'image_height': msg.height,
        'camera_name':  msg.header.frame_id,
        'camera_matrix':      {'rows': 3, 'cols': 3, 'data': list(msg.k)},
        'distortion_model':   msg.distortion_model,
        'distortion_coefficients': {'rows': 1, 'cols': len(msg.d), 'data': list(msg.d)},
        'rectification_matrix':   {'rows': 3, 'cols': 3, 'data': list(msg.r)},
        'projection_matrix':      {'rows': 3, 'cols': 4, 'data': list(msg.p)},
    }


def nearest(timestamps: list, query_ns: int) -> tuple[int, int]:
    """Return (index, timestamp_ns) of closest entry to query_ns."""
    if not timestamps:
        return -1, -1
    idx = min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - query_ns))
    return idx, timestamps[idx]


def collect_messages(reader, topic_types: dict) -> dict:
    """Read all messages into per-topic lists of (timestamp_ns, msg)."""
    buckets: dict[str, list] = {t: [] for t in topic_types}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in topic_types:
            continue
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        buckets[topic].append((ts, msg))
    return buckets


def process_bag(bag_path: str, output_dir: str, max_dt_ns: int):
    bag_path = str(bag_path)
    seq_name = Path(bag_path).name
    out      = Path(output_dir) / seq_name

    reader      = open_reader(bag_path)
    topic_types = get_topic_types(reader)

    missing = REQUIRED_TOPICS - set(topic_types)
    if missing:
        print(f"  [SKIP] missing topics: {missing}")
        return

    print(f"  Reading messages...")
    buckets = collect_messages(reader, topic_types)

    rgb_msgs    = buckets.get(TOPIC_RGB,         [])
    depth_msgs  = buckets.get(TOPIC_DEPTH,       [])
    lidar_msgs  = buckets.get(TOPIC_LIDAR,       [])
    navsat_msgs = buckets.get(TOPIC_NAVSATFIX,   [])
    pvt_msgs    = buckets.get(TOPIC_PVTGEODETIC, [])
    ins_msgs    = buckets.get(TOPIC_INSNAVGEOD,  [])
    cam_rgb     = buckets.get(TOPIC_CAM_RGB,     [])
    cam_depth   = buckets.get(TOPIC_CAM_DEPTH,   [])
    steer_msgs  = buckets.get(TOPIC_STEER,       [])
    speed_msgs  = buckets.get(TOPIC_SPEED,       [])
    accel_msgs  = buckets.get(TOPIC_ACCEL,       [])
    brake_msgs  = buckets.get(TOPIC_BRAKE,       [])
    shift_msgs  = buckets.get(TOPIC_SHIFT,       [])

    # Use navsatfix if available (100 Hz), fall back to pvtgeodetic (2 Hz, rad→deg)
    if navsat_msgs:
        gnss_msgs  = navsat_msgs
        gnss_source = 'navsatfix'
    elif pvt_msgs:
        gnss_msgs  = pvt_msgs
        gnss_source = 'pvtgeodetic'
    else:
        print(f"  [SKIP] no GNSS topic found (need /navsatfix or /pvtgeodetic)")
        return
    print(f"  GNSS source: {gnss_source} ({len(gnss_msgs)} messages)")

    has_controls = all([steer_msgs, speed_msgs, accel_msgs, brake_msgs, shift_msgs])
    if not has_controls:
        print(f"  [WARN] control topics missing or empty — controls/data.csv will be skipped")

    if not all([rgb_msgs, depth_msgs, lidar_msgs]):
        print(f"  [SKIP] one or more required topics are empty")
        return

    rgb_ts   = [t for t, _ in rgb_msgs]
    depth_ts = [t for t, _ in depth_msgs]
    gnss_ts  = [t for t, _ in gnss_msgs]
    ins_ts   = [t for t, _ in ins_msgs]
    steer_ts = [t for t, _ in steer_msgs]
    speed_ts = [t for t, _ in speed_msgs]
    accel_ts = [t for t, _ in accel_msgs]
    brake_ts = [t for t, _ in brake_msgs]
    shift_ts = [t for t, _ in shift_msgs]

    # Create output directories
    (out / 'front_camera' / 'rgb').mkdir(parents=True, exist_ok=True)
    (out / 'front_camera' / 'depth').mkdir(parents=True, exist_ok=True)
    (out / 'lidar').mkdir(parents=True, exist_ok=True)
    (out / 'gnss').mkdir(parents=True, exist_ok=True)
    (out / 'controls').mkdir(parents=True, exist_ok=True)
    (out / 'calibration').mkdir(parents=True, exist_ok=True)

    # Save calibration
    if cam_rgb:
        info = camera_info_to_dict(cam_rgb[0][1])
        with open(out / 'calibration' / 'rgb_camera_info.yaml', 'w') as f:
            yaml.dump(info, f)
    if cam_depth:
        info = camera_info_to_dict(cam_depth[0][1])
        with open(out / 'calibration' / 'depth_camera_info.yaml', 'w') as f:
            yaml.dump(info, f)

    # Save controls CSV (all messages, not just synchronized)
    if has_controls:
        with open(out / 'controls' / 'data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_ns',
                             'steering_output_rad', 'steering_cmd_rad', 'steering_manual_rad',
                             'vehicle_speed_mps',
                             'accel_output', 'accel_cmd', 'accel_manual',
                             'brake_output', 'brake_cmd', 'brake_manual',
                             'shift_output', 'shift_name'])
            # Use steering timestamps as the reference for the controls CSV
            for ts, msg in steer_msgs:
                _, spd_ts = nearest(speed_ts, ts)
                _, acc_ts = nearest(accel_ts, ts)
                _, brk_ts = nearest(brake_ts, ts)
                _, shf_ts = nearest(shift_ts, ts)
                spd_msg = speed_msgs[speed_ts.index(spd_ts)][1]
                acc_msg = accel_msgs[accel_ts.index(acc_ts)][1]
                brk_msg = brake_msgs[brake_ts.index(brk_ts)][1]
                shf_msg = shift_msgs[shift_ts.index(shf_ts)][1]
                writer.writerow([
                    ts,
                    msg.output, msg.command, msg.manual_input,
                    spd_msg.vehicle_speed,
                    acc_msg.output, acc_msg.command, acc_msg.manual_input,
                    brk_msg.output, brk_msg.command, brk_msg.manual_input,
                    shf_msg.output, SHIFT_NAMES.get(shf_msg.output, 'UNKNOWN'),
                ])

    # Save GNSS CSV — heading/pitch/roll from insnavgeod if available
    with open(out / 'gnss' / 'data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_ns',
                         'latitude_deg', 'longitude_deg', 'altitude_m',
                         'heading_deg', 'pitch_deg', 'roll_deg',
                         'gnss_source'])
        for ts, msg in gnss_msgs:
            if gnss_source == 'navsatfix':
                lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
            else:
                lat, lon, alt = math.degrees(msg.latitude), math.degrees(msg.longitude), msg.height
            ii, _ = nearest(ins_ts, ts)
            ins_msg = ins_msgs[ii][1] if ins_msgs else None
            writer.writerow([
                ts, lat, lon, alt,
                ins_msg.heading if ins_msg else '',
                ins_msg.pitch   if ins_msg else '',
                ins_msg.roll    if ins_msg else '',
                gnss_source,
            ])

    # Synchronize using LiDAR as reference
    ts_rows = []
    frame_idx = 0
    print(f"  Synchronizing {len(lidar_msgs)} LiDAR frames...")
    for lidar_ts, lidar_msg in lidar_msgs:
        ri,  rts  = nearest(rgb_ts,  lidar_ts)
        di,  dts  = nearest(depth_ts, lidar_ts)
        gi,  gts  = nearest(gnss_ts,  lidar_ts)
        sti, sts  = nearest(steer_ts,  lidar_ts) if has_controls else (-1, lidar_ts)

        diffs = [abs(rts - lidar_ts), abs(dts - lidar_ts), abs(gts - lidar_ts)]
        if has_controls:
            diffs.append(abs(sts - lidar_ts))
        if max(diffs) > max_dt_ns:
            continue

        name = f'{frame_idx:06d}'

        # RGB
        rgb = decode_compressed_rgb(bytes(rgb_msgs[ri][1].data))
        if rgb is None:
            continue
        cv2.imwrite(str(out / 'front_camera' / 'rgb'   / f'{name}.jpg'), rgb,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Depth
        depth = decode_compressed_depth(bytes(depth_msgs[di][1].data))
        if depth is None:
            continue
        cv2.imwrite(str(out / 'front_camera' / 'depth' / f'{name}.png'), depth)

        # LiDAR
        points = pointcloud2_to_xyzi(lidar_msg)
        points.tofile(str(out / 'lidar' / f'{name}.bin'))

        ts_rows.append([frame_idx, lidar_ts, rts, dts, gts, sts if has_controls else ''])
        frame_idx += 1

    # Timestamps index
    with open(out / 'calibration' / 'timestamps.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame_idx', 'lidar_ts', 'rgb_ts', 'depth_ts', 'gnss_ts', 'ctrl_ts'])
        writer.writerows(ts_rows)

    print(f"  Saved {frame_idx} synchronized frames → {out}")


def main():
    parser = argparse.ArgumentParser(description='Convert ROS2 bags to dataset format')
    parser.add_argument('input',      help='Bag file or directory of bags')
    parser.add_argument('output_dir', help='Output dataset directory')
    parser.add_argument('--max-dt',   type=float, default=0.1,
                        help='Max sync time difference in seconds (default: 0.1)')
    args = parser.parse_args()

    max_dt_ns = int(args.max_dt * 1e9)
    inp = Path(args.input)

    bags = sorted(inp.glob('*/metadata.yaml')) if inp.is_dir() else [inp / 'metadata.yaml']
    bags = [b.parent for b in bags if b.exists()]
    if not bags and inp.is_dir():
        # Maybe the dir itself is a bag
        bags = [inp] if (inp / 'metadata.yaml').exists() else []
    if not bags:
        print(f"No bags found at {inp}")
        sys.exit(1)

    for bag in bags:
        print(f"\nProcessing: {bag.name}")
        try:
            process_bag(bag, args.output_dir, max_dt_ns)
        except Exception as e:
            print(f"  [ERROR] {e}")


if __name__ == '__main__':
    main()
