#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl
import tf
from threading import Lock
import time
import copy

class PlaneFitter(object):
    def __init__(self):
        rospy.init_node('floor_finder')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)
        self.listener = tf.TransformListener()  #   Starts listening

    def find_distance(self, P, printing = True):
        x_vals = P[:, 0]
        y_vals = P[:, 1]
        z_vals = P[:, 2]
        add_mat = np.asarray([[1], [1], [0]])
        M = np.matmul(abs(P), add_mat) ** 2 #   List of distances from center of camera frame
        min_index = np.argmin(M)    #   Index of point closest to center of frame
        if printing:
            print "Center point xy: (%s, %s)" % (P[min_index, 0], P[min_index, 1])
            #print "Max x: %s" % min(x_vals)
            #print "Max y: %s" % max(x_vals)
            print "Distance: %s" % round(P[min_index, 2], 2)
        return round(P[min_index, 2], 2)

    def depth_to_odom(self, points):
        t_vals = (self.listener.lookupTransform('depth_camera', 'odom', rospy.Time.now())[0])
        r_quat = (self.listener.lookupTransform('depth_camera', 'odom', rospy.Time.now())[1])
        r_mat = tf.transformations.quaternion_matrix(r_quat)
        len_points = points.shape[0]
        ones = np.full((len_points, 1), 1)
        print points.size, ones.size
        P_cam = np.concatenate([points, ones], 1)
        P_odom = np.matmul(P_cam, r_mat)
        return P_odom[:, :3]

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

    def find_height(self, room_cloud, cam_cloud):
        z_vals = room_cloud[:, 2]
        floor_height = min(z_vals)
        cam_origin = self.depth_to_odom([[0, 0, 0]])
        cam_height = cam_origin[2]
        return cam_height - floor_height

    def process_cloud(self, msg):
        P = np.asarray([[p.x, p.y, p.z] for p in msg.points])   #   Point cloud from depth cam
        #print "got a cloud", P.shape, M.shape
        d = self.find_distance(P, True)
        room_cloud = self.depth_to_odom(P)
        h = self.find_height(room_cloud, P)
        print "Height: %s" % h

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
