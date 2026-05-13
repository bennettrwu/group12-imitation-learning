#!/usr/bin/env python3
"""
Record (x, y, yaw) of the GEM in the Gazebo world frame and write a CSV
in the format pure_pursuit_sim.py expects.

Polls /gazebo/get_model_state at a fixed rate and only appends a point
if it's at least MIN_SPACING meters from the previous one. This avoids
piling up thousands of points while the car is stopped.

Usage:
  rosrun gem_pure_pursuit_sim record_waypoints.py _out:=/path/to/wps_highbay.csv

Args (rosparam ~):
  out          output csv path (default: ./wps_<timestamp>.csv)
  rate         polling rate in Hz (default: 20)
  min_spacing  min meters between successive points (default: 0.1)
  model_name   Gazebo model name (default: gem_e4)
"""

import csv
import math
import os
import time

import rospy
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


def main():
    rospy.init_node("record_waypoints")

    out_path     = rospy.get_param("~out", f"./wps_{int(time.time())}.csv")
    rate_hz      = float(rospy.get_param("~rate", 20.0))
    min_spacing  = float(rospy.get_param("~min_spacing", 0.1))
    model_name   = rospy.get_param("~model_name", "gem_e4")

    rospy.loginfo(f"[rec] out={out_path} rate={rate_hz}Hz min_spacing={min_spacing}m model={model_name}")
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)

    rate = rospy.Rate(rate_hz)
    last_xy = None
    n_written = 0

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        while not rospy.is_shutdown():
            try:
                resp = get_state(model_name=model_name, relative_entity_name="")
            except rospy.ServiceException as e:
                rospy.logwarn_throttle(2.0, f"[rec] get_model_state failed: {e}")
                rate.sleep()
                continue
            if not resp.success:
                rospy.logwarn_throttle(2.0, f"[rec] get_model_state not success: {resp.status_message}")
                rate.sleep()
                continue

            x = resp.pose.position.x
            y = resp.pose.position.y
            q = resp.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            if last_xy is not None:
                dx = x - last_xy[0]
                dy = y - last_xy[1]
                if math.hypot(dx, dy) < min_spacing:
                    rate.sleep()
                    continue

            writer.writerow([round(x, 4), round(y, 4), round(yaw, 4), 0.0, 0.0])
            f.flush()
            n_written += 1
            last_xy = (x, y)
            if n_written % 50 == 0:
                rospy.loginfo(f"[rec] {n_written} points  last=({x:.2f},{y:.2f}, yaw={yaw:+.2f})")
            rate.sleep()

    rospy.loginfo(f"[rec] wrote {n_written} points to {out_path}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
