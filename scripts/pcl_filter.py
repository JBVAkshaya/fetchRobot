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


class PointCloudFilter:
    def __init__(self):
        #Initialize Pointcloud Subscriber
        self.pointcloud2_sub = rospy.Subscriber("/head_camera/depth_registered/points",PointCloud2, self.ros_to_pcl, queue_size=1)

        self.flag = 0


    def ros_to_pcl(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """

        if self.flag == 0:
            points_list=[]
            for data in pc2.read_points(ros_cloud, skip_nans=True):
                points_list.append([data[0], data[1], data[2], data[3]])

            pcl_data = pcl.PointCloud_PointXYZRGB()
            pcl_data.from_list(points_list)
            print(len(points_list))
            print(pcl_data)
            self.flag = 1
        # return pcl_data


if __name__=="__main__":
    rospy.init_node('pcl_filter',anonymous=True)
    PointCloudFilter()
    rospy.spin()
