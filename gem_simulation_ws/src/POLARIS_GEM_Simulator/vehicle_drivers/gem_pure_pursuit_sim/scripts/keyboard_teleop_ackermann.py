#!/usr/bin/env python3
"""
Keyboard teleop that publishes AckermannDrive on /ackermann_cmd.

Controls:
  w / s   speed up / slow down (and reverse)
  a / d   steer left / right
  space   zero speed and steering
  q       quit

Speed and steering decay toward zero when no key is pressed so the car
doesn't run away if you stop typing.
"""

import sys
import select
import termios
import tty
import threading

import rospy
from ackermann_msgs.msg import AckermannDrive


SPEED_STEP   = 0.5    # m/s per keypress
STEER_STEP   = 0.05   # rad per keypress
SPEED_MAX    = 5.0
STEER_MAX    = 0.61   # ~35 deg
SPEED_DECAY  = 0.0    # set >0 to auto-coast back to 0 when no keys pressed
STEER_DECAY  = 0.2    # rad/s back to center
PUBLISH_HZ   = 30.0


def drain_keys() -> list:
    """Read every char currently buffered on stdin. Prevents typematic
    backlog from building up when a key is held for several seconds."""
    keys = []
    while True:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            break
        keys.append(sys.stdin.read(1))
    return keys


def main():
    rospy.init_node("keyboard_teleop_ackermann")
    pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=1)
    rate = rospy.Rate(PUBLISH_HZ)

    speed = 0.0
    steer = 0.0
    lock = threading.Lock()

    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print(__doc__)
    print(f"[teleop] publishing on /ackermann_cmd at {PUBLISH_HZ:.0f} Hz")

    try:
        last_t = rospy.get_time()
        quit_requested = False
        while not rospy.is_shutdown():
            keys = drain_keys()
            if keys:
                with lock:
                    for key in keys:
                        if key == "w":
                            speed = min(SPEED_MAX, speed + SPEED_STEP)
                        elif key == "s":
                            speed = max(-SPEED_MAX, speed - SPEED_STEP)
                        elif key == "a":
                            steer = min(STEER_MAX, steer + STEER_STEP)
                        elif key == "d":
                            steer = max(-STEER_MAX, steer - STEER_STEP)
                        elif key == " ":
                            speed = 0.0
                            steer = 0.0
                        elif key == "q":
                            quit_requested = True
                            break
            if quit_requested:
                break

            now = rospy.get_time()
            dt = now - last_t
            last_t = now
            with lock:
                if SPEED_DECAY > 0.0:
                    if speed > 0:   speed = max(0.0, speed - SPEED_DECAY * dt)
                    elif speed < 0: speed = min(0.0, speed + SPEED_DECAY * dt)
                if STEER_DECAY > 0.0:
                    if steer > 0:   steer = max(0.0, steer - STEER_DECAY * dt)
                    elif steer < 0: steer = min(0.0, steer + STEER_DECAY * dt)

                msg = AckermannDrive()
                msg.speed = float(speed)
                msg.steering_angle = float(steer)
                pub.publish(msg)

            sys.stdout.write(f"\r[teleop] speed={speed:+.2f}  steer={steer:+.3f}   ")
            sys.stdout.flush()
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop = AckermannDrive()
        pub.publish(stop)
        print("\n[teleop] stopped.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
