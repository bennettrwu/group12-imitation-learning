#!/usr/bin/env python3
"""
Imitation-learning lane following driver for the GEM e4.

Run:
    ros2 run gem_lane_following lane_following \
        --ros-args -p model_path:=/path/to/best.ts.pt -p seq_len:=3
"""

import collections
from typing import Optional

import cv2
import numpy as np
import pygame
import scipy.signal as signal

import rclpy
import torch
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from pacmod2_msgs.msg import (
    PositionWithSpeed,
    VehicleSpeedRpt,
    GlobalCmd,
    SystemCmdFloat,
    SystemCmdInt,
)

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# Initialize pygame for joystick
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick connected (LB+RB engages, LB disengages)")
joystick = pygame.joystick.Joystick(0)
joystick.init()


class PID:
    def __init__(self, kp, ki, kd, wg=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def reset(self):
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e):
        if self.last_t is None:
            dt = 0.0
            de = 0.0
        else:
            dt = t - self.last_t
            de = (e - self.last_e) / dt if dt > 0.0 else 0.0

        self.iterm += e * dt
        if self.wg is not None:
            self.iterm = max(min(self.iterm, self.wg), -self.wg)

        self.last_e = e
        self.last_t = t

        return self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter:
    def __init__(self, cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = signal.butter(order, normal_cutoff, btype="low", analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)

    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted[0]


def preprocess(jpg_bytes: bytes, image_size: int) -> torch.Tensor:
    arr = np.frombuffer(jpg_bytes, np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed on CompressedImage payload")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (image_size, image_size), interpolation=cv2.INTER_AREA)
    f = rgb.astype(np.float32) / 255.0
    f = (f - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(f.transpose(2, 0, 1))  # HWC -> CHW


class LaneFollowing(Node):
    def __init__(self):
        super().__init__("lane_following_node")

        # Parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("seq_len", 3)
        self.declare_parameter("target_speed", 0.6)
        self.declare_parameter("image_size", 224)
        self.declare_parameter("camera_topic", "/oak/rgb/image_raw/compressed")
        self.declare_parameter("rate_hz", 20.0)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        if not model_path:
            raise RuntimeError(
                "model_path parameter is required (path to a .ts.pt TorchScript file)"
            )

        self.seq_len = int(self.get_parameter("seq_len").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        camera_topic = str(self.get_parameter("camera_topic").value)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.jit.load(model_path, map_location=self.device).eval()
        self.get_logger().info(
            f"Loaded TorchScript model: {model_path}  device={self.device}  seq_len={self.seq_len}"
        )

        self.frame_buffer: collections.deque = collections.deque(maxlen=self.seq_len)
        self.predicted_steering = 0.0  # rad
        self.has_prediction = False

        self.speed = 0.0
        self.pacmod_enable = False

        self.max_accel = 0.5
        self.pid_speed = PID(0.6, 0.0, 0.1, wg=10)
        self.speed_filter = OnlineFilter(1.2, 30, 4)

        self.create_subscription(CompressedImage, camera_topic, self.image_callback, 10)
        self.create_subscription(Bool, "/pacmod/enabled", self.enable_callback, 10)
        self.create_subscription(
            VehicleSpeedRpt, "/pacmod/vehicle_speed_rpt", self.speed_callback, 10
        )

        self.global_pub = self.create_publisher(GlobalCmd, "/pacmod/global_cmd", 10)
        self.gear_pub = self.create_publisher(SystemCmdInt, "/pacmod/shift_cmd", 10)
        self.brake_pub = self.create_publisher(SystemCmdFloat, "/pacmod/brake_cmd", 10)
        self.accel_pub = self.create_publisher(SystemCmdFloat, "/pacmod/accel_cmd", 10)
        self.turn_pub = self.create_publisher(SystemCmdInt, "/pacmod/turn_cmd", 10)
        self.steer_pub = self.create_publisher(
            PositionWithSpeed, "/pacmod/steering_cmd", 10
        )

        self.global_cmd = GlobalCmd(enable=False, clear_override=True)
        self.gear_cmd = SystemCmdInt(command=2)  # NEUTRAL
        self.brake_cmd = SystemCmdFloat(command=0.0)
        self.accel_cmd = SystemCmdFloat(command=0.0)
        self.turn_cmd = SystemCmdInt(command=1)  # no signal
        self.steer_cmd = PositionWithSpeed(
            angular_position=0.0, angular_velocity_limit=4.0
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.get_logger().info(
            f"Lane-following ready. target_speed={self.target_speed} m/s, rate={self.rate_hz} Hz"
        )

    def image_callback(self, msg: CompressedImage):
        try:
            tensor = preprocess(bytes(msg.data), self.image_size)
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return
        self.frame_buffer.append(tensor)
        if len(self.frame_buffer) < self.seq_len:
            return
        seq = torch.stack(list(self.frame_buffer)).unsqueeze(0).to(self.device)
        with torch.no_grad():
            pred = self.model(seq)
        self.predicted_steering = float(pred.squeeze().item())
        self.has_prediction = True

    def speed_callback(self, msg: VehicleSpeedRpt):
        self.speed = self.speed_filter.get_data(msg.vehicle_speed)

    def enable_callback(self, msg: Bool):
        self.pacmod_enable = msg.data

    def check_joystick_enable(self):
        pygame.event.pump()
        try:
            lb = joystick.get_button(6)
            rb = joystick.get_button(7)
        except pygame.error:
            self.get_logger().warn("Joystick read failed")
            return 2
        if lb and rb:
            # enable
            return 1
        elif lb and not rb:
            # disable
            return 0
        # others
        return 2

    def control_loop(self):
        joy_enable = self.check_joystick_enable()

        if joy_enable == 1 and not self.pacmod_enable:
            # joystick enable when vehicle disbaled
            self.global_cmd.enable = True
            self.global_cmd.clear_override = True
            self.global_pub.publish(self.global_cmd)

            self.gear_cmd.command = 3
            self.gear_pub.publish(self.gear_cmd)

            self.brake_cmd.command = 0.0
            self.brake_pub.publish(self.brake_cmd)

            self.accel_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)

            self.turn_cmd.command = 3
            self.turn_pub.publish(self.turn_cmd)

            self.get_logger().info("Vehicle enabled and forward gear engaged")

        elif joy_enable == 0 and self.pacmod_enable:
            # joystick disable when vehicle enbaled
            self.global_cmd.enable = False
            self.global_pub.publish(self.global_cmd)

            self.turn_cmd.command = 1
            self.turn_pub.publish(self.turn_cmd)

            self.get_logger().info("Vehicle disabled")

        elif joy_enable != 0 and self.pacmod_enable:
            if not self.has_prediction:
                return

            self.steer_cmd.angular_position = self.predicted_steering
            self.steer_pub.publish(self.steer_cmd)

            now = self.get_clock().now().nanoseconds * 1e-9
            speed_error = self.target_speed - self.speed
            if abs(speed_error) < 0.05:
                speed_error = 0.0
            throttle = self.pid_speed.get_control(now, speed_error)
            throttle = max(0.0, min(throttle, self.max_accel))

            self.accel_cmd.command = throttle
            self.brake_cmd.command = 0.0
            self.accel_pub.publish(self.accel_cmd)
            self.brake_pub.publish(self.brake_cmd)

            self.global_cmd.enable = True
            self.global_pub.publish(self.global_cmd)


def main(args=None):
    rclpy.init(args=args)
    lan_following = LaneFollowing()
    rclpy.spin(lan_following)
    lan_following.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
