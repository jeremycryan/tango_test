#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl
import tf
from threading import Lock
import time
#import copy
import tf

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
        self.pub_transformed = rospy.Publisher('/cloud_transformed', PointCloud, queue_size=10)

        self.listener = tf.TransformListener()
        self.CurrP = None
        self.cloud_plane = None
        self.actualP = None
        self.planetrynum = 5
        self.verticalityThreshold = .4
        self.t = 0.0
        self.robustThreshold = 100;
        #self.newP = False;

    def process_cloud(self, msg):
        self.m.acquire()
        self.actualP = msg
        self.actualP.points = self.actualP.points[::10]
        #self.CurrP = np.asarray([(p.x, p.y, p.z) for p in self.actualP.points], dtype = np.float32)
        #print "got a cloud", self.CurrP.shape
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
            if not self.actualP is None:
                #Setting Point Cloud
                self.m.acquire()
                #set_pose
                self.CurrP, actualPCopy = self.depth_to_odom(self.actualP)

                if actualPCopy is None:
                    pass
                else:
                    #skip = False
                    cloud_filtered = pcl.PointCloud(self.CurrP) #Move To PointCloud2 Eventually!
                    print "cloudsize: ", str(cloud_filtered.size)
                    #fil = self.UseP.make_passthrough_filter()
                    #fil.set_filter_field_name("z")
                    #fil.set_filter_limits(0, 999)
                    #cloud_filtered = self.UseP#fil.filter()
                    #print type(cloud_filtered), cloud_filtered, cloud_filtered[0]

                    #print "filtered data size: " + str(cloud_filtered.size)

                    #Plane
                    seg = cloud_filtered.make_segmenter()
                    seg.set_optimize_coefficients(True)
                    seg.set_model_type(pcl.SACMODEL_PLANE)
                    seg.set_method_type(pcl.SAC_RANSAC)
                    seg.set_distance_threshold(.03)
                    #seg.set
                    #self.CurrP = cloud_filtered.to_array
                    try:
                        for i in range(self.planetrynum):
                            print i
                            indices, plane_model = seg.segment()
                            if len(indices) < self.robustThreshold:
                                print len(indices)
                                print("ran out of robust planes")
                                break
                            if abs(plane_model[2]) < self.verticalityThreshold:
                                self.cloud_plane = cloud_filtered.extract(indices, negative=False) #Extracting plane points
                                actualPCopy.points  = [actualPCopy.points[indx] for indx in indices]
                                self.pub.publish(actualPCopy)
                                #print "Plane Num " + str(i)
                                print "plane model: " + str(plane_model)
                                print "plane_size: " + str(self.cloud_plane.size)
                                print "wall_distance: " + str(plane_model[3])
                                #cloud_filtered = cloud_filtered.extract(indices, negative=True)
                                break
                            else:
                                for a in indices:
                                    actualPCopy.points[a] = 0
                                for a in indices:
                                    actualPCopy.points.remove(0)
                                cloud_filtered = pcl.PointCloud(np.asarray([(p.x, p.y, p.z) for p in actualPCopy.points], dtype = np.float32))
                                seg = cloud_filtered.make_segmenter()
                                seg.set_optimize_coefficients(True)
                                seg.set_model_type(pcl.SACMODEL_PLANE)
                                seg.set_method_type(pcl.SAC_RANSAC)
                                seg.set_distance_threshold(.03)
                                continue
                            #self.cloud_plane.to_file("cloud_plane.pcd")
                    except Exception as inst:
                        print inst, i

                self.actualP = None
                self.UseP = None
                self.m.release()

                #self.newP = True
            #if self.newP:
            #
            #self.newP = False
            r.sleep()
    def depth_to_odom(self, cloud):
        """        self.t = self.listener.getLatestCommonTime('/depth_camera', '/odom')
        t_vals, r_quat = self.listener.lookupTransform("/depth_camera", "/odom", self.t)
        transform = self.listener.fromTranslationRotation(t_vals, r_quat)
        m = len(points)
        d4points = np.hstack((points, np.ones((m,1))))
        newpoints = np.matmul(transform, d4points.T)
        for i in range(m):
            cloud.points[i].x = d4points[i][0]
            cloud.points[i].y = d4points[i][1]
            cloud.points[i].z = d4points[i][2]
        return points, cloud"""

        try:
            if np.mean([p.z for p in cloud.points]) > 10**3:
                return None, None
            newcloud = self.listener.transformPointCloud('/odom', cloud)
            if np.isnan(np.mean([p.z for p in newcloud.points])):
                return None, None
            self.pub_transformed.publish(newcloud)
            points = np.asarray([(p.x, p.y, p.z) for p in newcloud.points], dtype = np.float32)
        except Exception as inst:
            print inst
            newcloud = None
            points = None
        return points, newcloud
        """
        t_vals = (self.listener.lookupTransform('/depth_camera', '/odom', self.t))[0]
        r_quat = (self.listener.lookupTransform('/depth_camera', '/odom', self.t))[1]
        r_mat = tf.transformations.quaternion_matrix(r_quat)
        m = len(pointlist)
        ones = np.full((m, 1), 1)
        print len(pointlist), len(pointlist[0].), ones.shape
        P_cam = np.hstack([pointlist, ones])
        P_odom = np.matmul(P_cam, r_mat)
        return P_odom[:,:3]"""

if __name__ == '__main__':
    node = PlaneFitter()
    node.run()
