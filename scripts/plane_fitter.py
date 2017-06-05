#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl
from threading import Lock
import time
import copy

class PlaneFitter(object):
    def __init__(self):
        rospy.init_node('plane_fiter')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)
        self.m = Lock()
        self.pub = rospy.Publisher('/plane_finder', PointCloud, queue_size=10)
        self.CurrP = None;
        self.cloud_plane = None;
        self.actualP = None;
        #self.newP = False;

    def process_cloud(self, msg):
        self.m.acquire()
        self.CurrP = np.asarray([(p.x, p.y, p.z) for p in msg.points[::10]], dtype = np.float32)
        self.actualP = msg
        self.actualP.points = self.actualP.points[::10]
        print "got a cloud", self.CurrP.shape
        self.m.release()


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.CurrP is None:
                #Setting Point Cloud
                self.m.acquire()
                self.UseP = pcl.PointCloud(self.CurrP) #Move To PointCloud2 Eventually!
                actualPCopy = self.actualP

                    #Filtering
                fil = self.UseP.make_passthrough_filter()
                fil.set_filter_field_name("z")
                fil.set_filter_limits(0, 5.5)
                cloud_filtered = fil.filter()

                print "filtered data size: " + str(cloud_filtered.size)

                for i in range(4):

                    #Plane
                    seg = cloud_filtered.make_segmenter()
                    seg.set_optimize_coefficients(True)
                    seg.set_model_type(pcl.SACMODEL_PLANE)
                    seg.set_method_type(pcl.SAC_RANSAC)
                    seg.set_distance_threshold(.03)
                    try:
                        indices, plane_model = seg.segment()
                        self.cloud_plane = cloud_filtered.extract(indices, negative=False) #Extracting plane points
                        actualPCopy.points = [actualPCopy.points[indx] for indx in indices]
                        self.pub.publish(actualPCopy)
                        print "plane model: " + str(plane_model)
                        print "plane_size: " + str(self.cloud_plane.size)
                        cloud_filtered = cloud_filtered.extract(indices, negative=True)
                        #self.cloud_plane.to_file("cloud_plane.pcd")
                    except Exception as inst:
                        print inst

                self.CurrP = None
                self.UseP = None
                self.m.release()

                #self.newP = True
            #if self.newP:
            #
            #self.newP = False
            r.sleep()

if __name__ == '__main__':
    node = PlaneFitter()
    node.run()
