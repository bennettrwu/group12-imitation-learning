#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_sim.py                                                                  
# Description: pure pursuit controller for GEM vehicle in Gazebo                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 07/10/2021                                                                
# Date last modified: 07/15/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_pure_pursuit_sim pure_pursuit_sim.py                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(20)

        self.look_ahead = 3    # meters
        self.wheelbase  = 3.42 # meters
        self.goal       = 0

        self.read_waypoints() # read waypoints

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0 
        self.ackermann_msg.steering_angle          = 0.0

        self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)


    # import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/wps.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_yaw = [float(point[2]) for point in path_points]

    def get_gem_pose(self):

        rospy.wait_for_service('/gazebo/get_model_state')
        
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem_e4')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q      = model_state.pose.orientation
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x,4), round(y,4), round(yaw,4)


    def start_pp(self):

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)
        n = len(self.path_points_x)

        while not rospy.is_shutdown():

            # get current position and orientation in the world frame
            curr_x, curr_y, curr_yaw = self.get_gem_pose()
            cos_y, sin_y = np.cos(curr_yaw), np.sin(curr_yaw)

            # advance the goal forward along the path (modulo n, so a closed
            # loop wraps cleanly) until we find a waypoint that's at least
            # look_ahead away and in front of the vehicle. Searching forward
            # from self.goal avoids latching onto earlier waypoints when the
            # path passes near itself.
            idx = self.goal
            for _ in range(n):
                dx = self.path_points_x[idx] - curr_x
                dy = self.path_points_y[idx] - curr_y
                fwd = dx * cos_y + dy * sin_y
                dist = math.hypot(dx, dy)
                if dist >= self.look_ahead and fwd > 0:
                    break
                idx = (idx + 1) % n
            self.goal = idx

            # goal point in the vehicle frame: +x is forward, +y is left
            gvcx = self.path_points_x[self.goal] - curr_x
            gvcy = self.path_points_y[self.goal] - curr_y
            goal_x_veh_coord = gvcx * cos_y + gvcy * sin_y
            goal_y_veh_coord = gvcy * cos_y - gvcx * sin_y

            # true look-ahead distance to the goal
            L = math.hypot(gvcx, gvcy)

            # alpha is the bearing to the lookahead point in the vehicle frame
            alpha   = math.atan2(goal_y_veh_coord, goal_x_veh_coord)
            k       = 0.285
            angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L)
            angle   = angle_i*2
            angle   = round(np.clip(angle, -0.61, 0.61), 3)

            ct_error = round(goal_y_veh_coord, 3)

            print("Crosstrack Error: " + str(ct_error))

            # implement constant pure pursuit controller
            self.ackermann_msg.speed          = 2.8
            self.ackermann_msg.steering_angle = angle
            self.ackermann_pub.publish(self.ackermann_msg)

            self.rate.sleep()

def pure_pursuit():

    rospy.init_node('pure_pursuit_sim_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()

