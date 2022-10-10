#!/usr/bin/env python3



# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
import cv2
from pcl_helper import *
from geometry_msgs.msg import PointStamped
import tf

#Reading raw rgb and depth image
def segment_yellow_cube(frame):
    # Converts images from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Here we are defining range of yellow cube in HSV
    # This will create a mask for all objects within the defined range found in the frame.
    lower_yellow = np.array([10,150,100])
    upper_yellow = np.array([30,255,255])
    # lower_yellow = np.array([80,150,100])
    # upper_yellow = np.array([140,255,255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #Denoising the generated mask using opening and dilation
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations = 3)
    # We find the contour with the largest area and make a bounding box around it
    contours = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    # prin  t(ret_len(ret_contours))
    # contours,hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    try:
        x,y,w,h = cv2.boundingRect(contours[1][0])
    except Exception as e:
        print("No object found")
        x,y,w,h = 0,0,0,0
    # print(contours, (x,y,w,h))
    cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),3)
    cv2.circle(frame, (x+int(w/2),y+int(h/2)), radius=5, color=(0, 0, 255), thickness=-1)
    # x,y,w,h = 0,0,0,0
    cv2.imshow('mask', mask)
    cv2.imshow('rgb',frame)
    cv2.waitKey(40)
    
    return (mask, [x,y,w,h])


class PointCloudFilter:
    def __init__(self):
        #Initialize Pointcloud Subscriber
        self.pointcloud2_sub = rospy.Subscriber("/head_camera/depth_registered/points",PointCloud2, self.ros_to_pcl, queue_size=1)


    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """

        rgb_frame = []
        xyz_frame = []
        for data in pc2.read_points(ros_cloud, skip_nans=False):
            xyz_frame.append((data[0], data[1], data[2]))
            rgb_frame.append((float_to_rgb(data[3])))

        rgb_frame = np.array(rgb_frame).reshape(ros_cloud.height, ros_cloud.width, 3).astype('uint8')
        rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        xyz_frame = np.array(xyz_frame).reshape(ros_cloud.height, ros_cloud.width, 3).astype('float')
        np.nan_to_num(xyz_frame, copy = False)

        info =segment_yellow_cube(rgb_frame)
        x,y,w,h = info[1]

        listener = tf.TransformListener()
        listener.waitForTransform('head_camera_rgb_optical_frame','/map', rospy.Time(0), rospy.Duration(1))
        
        cube_point = PointStamped()
        cube_point.header.frame_id = 'head_camera_rgb_optical_frame'
        cube_point.header.stamp = rospy.Time(0)
        cube_point.point.x = xyz_frame[y+int(h/2), x + int(w/2)][0]
        cube_point.point.y = xyz_frame[y+int(h/2), x + int(w/2)][1]
        cube_point.point.z = xyz_frame[y+int(h/2), x + int(w/2)][2]
        
        # (trans, rot) = listener.lookupTransform('head_camera_depth_frame', 'base_link', rospy.Time(0))
        # print(trans, rot)
        p = listener.transformPoint('/map', cube_point)
        print(p)


if __name__=="__main__":
    rospy.init_node('pcl_filter',anonymous=True)
    PointCloudFilter()
    rospy.spin()
