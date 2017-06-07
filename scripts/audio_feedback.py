#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Header, ColorRGBA
import random
from visualization_msgs.msg import Marker
import pyttsx
from os import system
import numpy as np

class AudioFeedback(object):
    def __init__(self):
        rospy.init_node('audio_feedback')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.x = None
        self.y = None
        self.yaw = None
        self.start = False

    def process_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def run(self):
        lps = 10    #   loops per second!
        r = rospy.Rate(lps)
        has_spoken = False
        while not rospy.is_shutdown():

            if self.x and not self.start:
                self.start = True
                jump_t = 0.5    #   duration (s) jump measurements are sampled
                dist_t = 1.0    #                movement
                rot_t = 1.0     #                rotation
                distance = [0] * int(dist_t * lps)
                rotation = [0] * int(rot_t * lps)
                zd = [0] * int(jump_t * lps)
                last_beep = rospy.Time.now()
                beep_frequency = 1
                last_whoosh = rospy.Time.now()
                whoosh_frequency = 1
                x_curr, y_curr, z_curr, yaw_curr = self.x, self.y, self.z, self.yaw

            if self.x and self.start:
                x_prev = x_curr
                y_prev = y_curr
                x_curr = self.x
                y_curr = self.y

                change = math.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)
                distance = [change] + distance[:9]
                total_distance = sum(distance)
                if total_distance > 1 and rospy.Time.now() - last_beep > rospy.Duration(beep_frequency):
                    system('aplay /home/jryan/catkin_ws/src/tango_test/scripts/beep.wav')
                    distance = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    last_beep = rospy.Time.now()

                z_prev = z_curr
                z_curr = self.z
                change = (z_curr - z_prev)
                zd = [change] + zd[:4]
                total_jump = sum(zd)
                if total_jump > 0.1:
                    system('aplay /home/jryan/catkin_ws/src/tango_test/scripts/jomp.wav')
                    zd = [0, 0, 0, 0, 0]

                yaw_prev = yaw_curr
                yaw_curr = self.yaw

                change = yaw_curr - yaw_prev
                if abs(change) > math.pi:
                    change -= 2*math.pi*np.sign(change)
                rotation = [change] + rotation[:9]
                total_rotation = sum(rotation)

                if abs(total_rotation) > math.pi and rospy.Time.now() - last_whoosh > rospy.Duration(whoosh_frequency):
                    system('aplay /home/jryan/catkin_ws/src/tango_test/scripts/ding.wav')
                    rotation = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    last_whoosh = rospy.Time.now()

                print "Distance: " + str(round(total_distance, 3))
                print "Rotation: " + str(round(total_rotation, 3))

            r.sleep()

if __name__ == '__main__':
    node = AudioFeedback()
    node.run()
