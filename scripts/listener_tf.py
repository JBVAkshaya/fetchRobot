

#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface,PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import tf
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import PointStamped

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


from moveit_msgs.msg import MoveItErrorCodes, RobotState
import moveit_commander

if __name__ == '__main__':
    rospy.init_node("hi")
    listener = tf.TransformListener()
    listener.waitForTransform('/base_link','/map', rospy.Time(0), rospy.Duration(1))
    
    cube_point = PointStamped()
    cube_point.header.frame_id = '/base_link'
    cube_point.header.stamp = rospy.Time(0)
    cube_point.point.x = -2.8
    cube_point.point.y = -0.6
    cube_point.point.z = 0.35
    # (trans, rot) = listener.lookupTransform('head_camera_depth_frame', 'base_link', rospy.Time(0))
    # print(trans, rot)
    p = listener.transformPoint('/map', cube_point)

    print("in map link: ", p, tf.transformations.quaternion_from_euler(0.0, np.pi/10, 0.0))


