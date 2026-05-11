#!/usr/bin/env python3
import rospy
import torch
import torchvision.transforms.functional as TF
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge

CHECKPOINTS = {
    "cnn":      "/home/guests1/host/gem_simulation_ws/checkpoints/cnn/best.ts.pt",
    "cnn_lstm": "/home/guests1/host/gem_simulation_ws/checkpoints/cnn_lstm/best.ts.pt",
    "cnn_node": "/home/guests1/host/gem_simulation_ws/checkpoints/cnn_node/best.ts.pt",
}
SEQ_LENS       = {"cnn": 1, "cnn_lstm": 3, "cnn_node": 3}
IMAGE_TOPIC    = "/oak/rgb/image_raw"
STEERING_TOPIC = "/ackermann_cmd"
IMG_H, IMG_W   = 224, 224

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD  = [0.229, 0.224, 0.225]

class ImitationLearningNode:
    def __init__(self):
        model_name  = rospy.get_param("~model", "cnn_lstm")
        self.speed  = rospy.get_param("~speed", 1.0)
        self.model_name = model_name
        self.seq_len    = SEQ_LENS[model_name]
        rospy.loginfo(f"[IL] Loading {model_name} (seq_len={self.seq_len})")
        self.device = torch.device("cpu")
        self.model  = torch.jit.load(CHECKPOINTS[model_name], map_location=self.device)
        self.model.eval()
        rospy.loginfo(f"[IL] Model ready on {self.device}")
        self.bridge       = CvBridge()
        self.frame_buffer = deque(maxlen=self.seq_len)
        self.pub = rospy.Publisher(STEERING_TOPIC, AckermannDrive, queue_size=1)
        rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"[IL] Subscribed to {IMAGE_TOPIC}, speed={self.speed} m/s")

    def preprocess(self, ros_image):
        img = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="rgb8")
        img = cv2.resize(img, (IMG_W, IMG_H))
        tensor = torch.from_numpy(img.astype(np.float32) / 255.0).permute(2, 0, 1)
        tensor = TF.normalize(tensor, mean=IMAGENET_MEAN, std=IMAGENET_STD)
        return tensor

    def image_callback(self, msg):
        try:
            self.frame_buffer.append(self.preprocess(msg))
            if len(self.frame_buffer) < self.seq_len:
                return
            seq = torch.stack(list(self.frame_buffer)).unsqueeze(0).to(self.device)
            with torch.no_grad():
                steering_rad = self.model(seq).item()
            cmd = AckermannDrive()
            cmd.steering_angle = float(steering_rad)
            cmd.speed          = float(self.speed)
            self.pub.publish(cmd)
            rospy.loginfo_throttle(1.0, f"[IL] {self.model_name} steering={steering_rad:.4f}rad speed={self.speed}m/s")
        except Exception as e:
            rospy.logerr(f"[IL] Inference error: {e}")

if __name__ == "__main__":
    rospy.init_node("imitation_learning_inference")
    node = ImitationLearningNode()
    rospy.spin()
