#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl

class PlaneFitter(object):
    def __init__(self):
        rospy.init_node('plane_fiter')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)

    def process_cloud(self, msg):
        P = np.asarray([[p.x, p.y, p.z] for p in msg.points])
        print "got a cloud", P.shape
        pass


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = PlaneFitter()
    node.run()
