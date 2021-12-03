from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
import rospy
from pcl_helper import *
# import cv2 as cv
import numpy as np
import tf
# def ros_to_pcl(ros_cloud):
#    ""
# " Converts a ROS PointCloud2 message to a pcl PointXYZRGB

# Args:
#    ros_cloud(PointCloud2): ROS PointCloud2 message

# Returns:
#    pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud ""
# "
# points_list = []

# for data in pc2.read_points(ros_cloud, skip_nans = True):
#    points_list.append([data[0], data[1], data[2], data[3]])

# pcl_data = pcl.PointCloud_PointXYZRGB()
# pcl_data.from_list(points_list)

# return pcl_data
import cv2
import numpy as np
 
#Reading raw rgb and depth image
def segment_yellow_cube(frame):
    # Converts images from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Here we are defining range of yellow cube in HSV
    # This will create a mask for all objects within the defined range found in the frame.
    lower_yellow = np.array([10,150,100])
    upper_yellow = np.array([30,255,255])

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


def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    listener = tf.TransformListener()
    listener.waitForTransform('head_camera_link','/map', rospy.Time(0), rospy.Duration(1))
    rgb_list = []
    xyz_list = []
    pc = pc2.read_points(data, skip_nans = False)
    for i in pc:
        xyz_list.append((i[0], i[1], i[2]))
        rgb_list.append((float_to_rgb(i[3])))
        
    rgb_list = np.array(rgb_list).reshape(data.height, data.width, 3).astype('uint8')
    rgb_list = cv2.cvtColor(rgb_list, cv2.COLOR_RGB2BGR)
    xyz_list = np.array(xyz_list).reshape(data.height, data.width, 3).astype('float')
    np.nan_to_num(xyz_list, copy = False)
    # xyz_list = (xyz_list - np.min(xyz_list))/(np.max(xyz_list) - np.min(xyz_list))*255
    # # np.res
    info =segment_yellow_cube(rgb_list)
    x,y,w,h = info[1]
    cube_point = PointStamped()
    cube_point.header.frame_id = 'head_camera_depth_frame'
    cube_point.header.stamp = rospy.Time(0)
    cube_point.point.x = xyz_list[y+int(h/2), x + int(w/2)][0]
    cube_point.point.y = xyz_list[y+int(h/2), x + int(w/2)][1]
    cube_point.point.z = xyz_list[y+int(h/2), x + int(w/2)][2]
    
    # (trans, rot) = listener.lookupTransform('head_camera_depth_frame', 'base_link', rospy.Time(0))
    # print(trans, rot)
    p = listener.transformPoint('/map', cube_point)
    print(p)
    # print(xyz_list[y+int(h/2), x + int(w/2)])
    # cv2.imshow('rgb',rgb_list)
    # cv.waitKey(1000)
    # print(xyz_list)

rospy.init_node('listener', anonymous = True)
rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2, callback_pointcloud)
rospy.spin()