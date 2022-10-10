

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

    planning_scene_table = PlanningSceneInterface("/map")
        
    while 1:
        # planning_scene_table = planning_scene_interface("/map")
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = [2.0,1.0,0.71]

        p = PoseStamped()
        p.header.frame_id = "/map"
        p.pose.position.x = 0.634
        p.pose.position.y = 0.034
        p.pose.position.z =  0.35

        quat_box =  tf.transformations.quaternion_from_euler(0.0, 0.0, 1.95)
        p.pose.orientation.x = quat_box[0]
        p.pose.orientation.y = quat_box[1]
        p.pose.orientation.z = quat_box[2]
        p.pose.orientation.w = quat_box[3]

        o = planning_scene_table.makeSolidPrimitive("table_col", primitive, p.pose, frame_id="/map")
        planning_scene_table.sendUpdate(o, None)
        # planning_scene_table.removeAttachedObject('/map',"table_colin")

        # This is the wrist link not the gripper itself
        # quats = get_quaternion_from_euler(0.0,0.0,0.0)




