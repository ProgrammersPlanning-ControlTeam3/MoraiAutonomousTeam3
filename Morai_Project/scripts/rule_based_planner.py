#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from math import cos, sin, pi, sqrt, pow, atan2
import numpy as np
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList, EventInfo, Lamps
from morai_msgs.srv import MoraiEventCmdSrv
from morai_msgs.msg import CtrlCmd
import os
import sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *
sys.path.insert(0, '/home/ubuntu/final_project/src')
#print(sys.path)

class rule_based_planner:
    def __init__(self):
        rospy.init_node('rule_based_planner', anonymous=True)
        
        # Initialize all necessary attributes first
        self.is_global_path = False
        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_object = False

        # Ensure dependencies are ready before using them
        self.target_velocity = 40  # Assuming a default or parameterized value
        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)
        self.pure_pursuit = pure_pursuit()
        self.object_detector = object_detector()

        # Initialize subscribers and publishers
        self.initialize_subscribers()
        self.initialize_publishers()

        # Now it's safe to call wait_for_resources, as all dependencies are initialized
        self.wait_for_resources()

        # Proceed to the main loop or other initialization
        self.main_loop()

    def initialize_subscribers(self):
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)

    def initialize_publishers(self):
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd_0', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

    def wait_for_resources(self):
        rospy.loginfo("Waiting for global path data")
        while not rospy.is_shutdown():
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                rospy.loginfo("Received global path data and processed velocity profile")
                break
            time.sleep(0.1)

    def main_loop(self):
        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.process_control_logic()
            rate.sleep()

    def process_control_logic(self):
        self.current_waypoint = self.pure_pursuit.get_current_waypoint(self.status_msg, self.global_path)
        self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
        steering = self.pure_pursuit.calc_pure_pursuit()
        self.ctrl_cmd_msg.steering = steering if self.pure_pursuit.is_look_forward_point else 0.0
        output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
        self.adjust_control_command(output)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def adjust_control_command(self, output):
        if output > 0.0:
            self.ctrl_cmd_msg.accel = output
            self.ctrl_cmd_msg.brake = 0.0
        else:
            self.ctrl_cmd_msg.accel = 0.0
            self.ctrl_cmd_msg.brake = -output

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def object_callback(self, msg):
        self.object_msg = msg
        self.is_object = True

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def odom_callback(self, msg):
        self.current_position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
        _, _, self.vehicle_yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.is_odom = True

    def path_callback(self, msg):
        self.path = msg
        self.is_path = True



class object_detector:
    def __init__(self):

        rospy.init_node('object_detector', anonymous=True)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)

    def object_callback(self, msg):
        self.is_object = True
        self.object_msg = msg
    
    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def nearest_cost(self):
    
        npc_positions = np.array([(object.position.x, object.position.y) for object in self.object_msg.npc_list])
        ego_position = np.array([self.status_msg.position.x, self.status_msg.position.y])

        squared_distances = np.sum((npc_positions - ego_position) ** 2, axis=1)
        if squared_distances.size == 0:
            nearest_dis = 90000.0
            heading_difference = 0.0
        else:
            nearest_index = np.argmin(squared_distances)
            nearest_dis = squared_distances[nearest_index]
            heading_difference = abs(self.object_msg.npc_list[nearest_index].heading - self.status_msg.heading)

        return nearest_dis, heading_difference
        
class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []

            for box in range(-point_num, point_num):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                x_list.append([-2 * x, -2 * y, 1])
                y_list.append((-x * x) - (y * y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a * a + b * b - c)

            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class pure_pursuit:
    def __init__(self):


        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        self.is_path = False
        self.is_odom = False
        self.is_status = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.355  # Hyeondai Ioniq (hev)
        self.lfd = 3
        self.min_lfd = 5
        self.max_lfd = 80  # default 30
        self.lfd_gain = 1.2  # default 0.78

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.is_status = True
        self.status_msg = msg
        self.global_path = msg
        self.is_global_path = True

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        currnet_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self, ):

        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain

        if self.lfd < self.min_lfd:
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd:
            self.lfd = self.max_lfd

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)
        
        return steering
    
class object_detector:
    def __init__(self):

        rospy.init_node('object_detector', anonymous=True)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)

    def object_callback(self, msg):
        self.is_object = True
        self.object_msg = msg
    
    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def nearest_cost(self):
    
        npc_positions = np.array([(object.position.x, object.position.y) for object in self.object_msg.npc_list])
        ego_position = np.array([self.status_msg.position.x, self.status_msg.position.y])

        squared_distances = np.sum((npc_positions - ego_position) ** 2, axis=1)
        if squared_distances.size == 0:
            nearest_dis = 90000.0
            heading_difference = 0.0
        else:
            nearest_index = np.argmin(squared_distances)
            nearest_dis = squared_distances[nearest_index]
            heading_difference = abs(self.object_msg.npc_list[nearest_index].heading - self.status_msg.heading)

        return nearest_dis, heading_difference
        
class pidControl:
    def __init__(self):
        self.p_gain = 0.3  # defalt 0.3
        self.i_gain = 0.00
        self.d_gain = 0.1  # defalt 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

import traceback

def main():
    try:
        rospy.init_node('rule_based_planner', anonymous=True)
        planner = rule_based_planner()
        rospy.spin()
    except Exception as e:
        rospy.logerr("Unhandled exception in the main loop: %s", str(e))
        rospy.logerr("Traceback: %s", traceback.format_exc())
        rospy.signal_shutdown(str(e))

if __name__ == '__main__':
    main()
