#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: nosaka
# @brief: 点群と姿勢の2つ1セットのデータを作成するコード

# General
import os
import random
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ROS
import rospy
import rospkg
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import InteractiveMarkerFeedback
from cv_bridge import CvBridge, CvBridgeError
from detectron2_ros.msg import Result

START_NUM = 1 # start_number of savefile

class DatasetCollect():
    def __init__(self):
        # common
        self.cv_bridge = CvBridge()
        self.PCD_NUM = 512
        self.DATASET_PATH = rospkg.RosPack().get_path("im_teacher")+"/dataset"
        # Image Callback
        self.texturing_img = []
        # Mask R-CNN Callback
        self.bounding_box_list = []
        self.class_ids = []
        self.class_names = []
        self.scores = []
        self.masks = []
        # PointCloud Callback
        self.pc_x_list = []
        self.pc_y_list = []
        self.pc_z_list = []
        #self.pc_w_list = []
        self.pc_data_width  = 0
        self.pc_data_height = 0
        # InteractiveMarker Callback
        self.pos = []
        self.quat = []
        self.euler = []
        # sub
        rospy.Subscriber("/texturing_image", Image, self.image_CB)
        rospy.Subscriber("/textured_point_cloud", PointCloud2, self.pointcloud_CB)
        rospy.Subscriber("/knuckle_detectron2_ros/result", Result, self.maskrcnn_CB)
        rospy.Subscriber("/simple_marker/feedback", InteractiveMarkerFeedback, self.iMarker_CB)

    def image_CB(self, msg):
        self.texturing_img = self.cv_bridge.imgmsg_to_cv2(msg)

    def pointcloud_CB(self, msg):
        pc_x_list = []
        pc_y_list = []
        pc_z_list = []
        #pc_w_list = []
        
        for p in pc2.read_points(msg):
            pc_x_list.append(p[0])
            pc_y_list.append(p[1])
            pc_z_list.append(p[2])
            #pc_w_list.append(p[3])
        
        self.pc_x_list = pc_x_list
        self.pc_y_list = pc_y_list
        self.pc_z_list = pc_z_list
        #self.pc_z_list = pc_z_list
        
        self.pc_data_width  = msg.width
        self.pc_data_height = msg.height

    def maskrcnn_CB(self, msg):
        self.class_ids = msg.class_ids
        self.class_names = msg.class_names
        self.scores = msg.scores
        self.masks = msg.masks

    def iMarker_CB(self, msg):
        self.pos = msg.pose.position
        self.quat = msg.pose.orientation
        self.euler = tf.transformations.euler_from_quaternion((self.quat.x,
                                                               self.quat.y,
                                                               self.quat.z,
                                                               self.quat.w))

    def cut_out(self):
        global START_NUM
        
        obj_id = 999
        choise_object = 999
        judge = True
        
        index_numbers = []
        mask_input_image, mask_input_image_erode, binary_mask = [], [], []
        
        pc_x_list = []
        pc_y_list = []
        pc_z_list = []
        #pc_w_list = []
        
        pointcloud_x = []
        pointcloud_y = []
        pointcloud_z = []
        #pointcloud_w = []
        pointcloud_xyz = []
        
        euler = []
        
        erode_kernel = np.ones((5, 5), np.uint8)
        
        if "knuckle_ellipse" in self.class_names:
            index_numbers = [i for i, x in enumerate(self.class_names) if x == "knuckle_ellipse"]
        else:
            pass
        
        # IndexError point
        choise_object = index_numbers[0] 
        obj_id = self.class_ids[choise_object]
        
        mask_input_image = self.cv_bridge.imgmsg_to_cv2(self.masks[choise_object], desired_encoding = "8UC1")
        mask_input_image_erode = cv2.erode(mask_input_image, erode_kernel, iterations = 1)
        
        edge = cv2.bitwise_xor(mask_input_image, mask_input_image_erode)
        # TypeError point
        binary_mask = cv2.bitwise_and(self.texturing_img, self.texturing_img, mask = mask_input_image)
        
        pc_x_list = np.array(self.pc_x_list)
        pc_y_list = np.array(self.pc_y_list)
        pc_z_list = np.array(self.pc_z_list)
        #pc_w_list = np.array(self.pc_w_list)
        
        # ValueError point
        pc_x_list = pc_x_list.reshape(1024, 1280)
        pc_y_list = pc_y_list.reshape(1024, 1280)
        pc_z_list = pc_z_list.reshape(1024, 1280)
        #pc_w_list = pc_w_list.reshape(1024, 1280)
        
        euler = self.euler
        
        # cut out the pointcloud only in the detected area
        for h in range(edge.shape[0]):
            for w in range(edge.shape[1]):
                if (edge[h][w] == 255
                    and np.isnan(pc_x_list[h][w]) == False
                    and np.isnan(pc_y_list[h][w]) == False
                    and np.isnan(pc_z_list[h][w]) == False):
                    #and np.isnan(pc_w_list[h][w]) == False):
                    pointcloud_x.append(pc_x_list[h][w])
                    pointcloud_y.append(pc_y_list[h][w])
                    pointcloud_z.append(pc_z_list[h][w])
                    #pointcloud_w.append(pc_w_list[h][w])
        
        pointcloud_xyz = np.stack([pointcloud_x, pointcloud_y, pointcloud_z], axis = 1)
        pointcloud_xyz_downsample = self.random_downsampling(pointcloud_xyz, self.PCD_NUM)
        euler = np.array(euler)
        
        print("pcd_shape: "+str(pointcloud_xyz.shape)+" -> "+str(pointcloud_xyz_downsample.shape)+
              "euler_shape: "+str(euler.shape))
        print("Number: "+str(START_NUM))
        
        #self.pcd_plot(pointcloud_xyz_downsample)
        
        if pointcloud_xyz_downsample.shape == (self.PCD_NUM, 3) and euler.shape == (3,):
            np.save(self.DATASET_PATH+"/pointcloud"+"/pcd_"+str(START_NUM),
                    pointcloud_xyz_downsample)
            np.save(self.DATASET_PATH+"/pose"+"/pose_"+str(START_NUM),
                    euler)
            START_NUM += 1
            judge = True
            return judge
        else:
            judge = False
            return judge

    def random_downsampling(self, pcd, n):
        elm_stock = []
        lng = len(pcd)
        
        for i in range(lng):
            elm_stock.append(i)
        
        elm_stock = random.sample(elm_stock, n)
        elm_stock.sort()
        
        pcd_downsample = [pcd[i] for i in elm_stock]
        pcd_downsample = np.array(pcd_downsample)
        return pcd_downsample

    def pcd_plot(self, pcd):
        fig = plt.figure()
        ax = Axes3D(fig)
        
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        ax.plot(pcd[:,0], pcd[:,1], pcd[:,2], "o", ms = 4, mew = 0.5)
        plt.show()

    def main(self):
        r = rospy.Rate(3)
        
        if os.path.exists(self.DATASET_PATH) == False:
            os.makedirs(self.DATASET_PATH+"/pointcloud")
            os.makedirs(self.DATASET_PATH+"/pose")
        
        while not rospy.is_shutdown(): 
            try:
                key = raw_input("Please Key Input ")
                if key == "j":
                    judge = self.cut_out()
                    if judge == True:
                        rospy.loginfo("Success\n")
                    else:
                        rospy.loginfo("Failed\n")
                elif key == "f":
                    rospy.loginfo("Finish")
                    break
                else:
                    rospy.loginfo("Please put 'j' or 'f'...")
            except IndexError:
                rospy.loginfo("IndexError\n")
            except TypeError:
                rospy.loginfo("TypeError\n")
            except ValueError:
                rospy.loginfo("ValueError\n")
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("dataset_collect", anonymous = True)
    dcl = DatasetCollect()
    dcl.main()
