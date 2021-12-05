

#!/usr/bin/env python

# wave.py: "Wave" the fetch gripper
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
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

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # print(move_group.get_end_effector_link())
    # # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground


    planning_scene = PlanningSceneInterface("/map")
    # primitive = SolidPrimitive()
    # primitive.type = primitive.BOX
    # primitive.dimensions = [2.0,1.0,0.71]

    # p = PoseStamped()
    # p.header.frame_id = "/map"
    # p.pose.position.x = 0.0
    # p.pose.position.y = 0.0
    # p.pose.position.z =  0.35

    # quat_box =  tf.transformations.quaternion_from_euler(0.0, 0.0, 0.75)
    # p.pose.orientation.x = quat_box[0]
    # p.pose.orientation.y = quat_box[1]
    # p.pose.orientation.z = quat_box[2]
    # p.pose.orientation.w = quat_box[3]

    # planning_scene.addSolidPrimitive("table_colin", primitive, p.pose, frame_id="/map")
    # planning_scene.removeAttachedObject('/map',"table_colin")

    # primitive1 = SolidPrimitive()
    # primitive1.type = primitive.BOX
    # primitive1.dimensions = [2.0,1.0,0.71]

    # p1 = PoseStamped()
    # p1.header.frame_id = "/base_link"
    # p1.pose.position.x = 0.0
    # p1.pose.position.y = 0.0
    # p1.pose.position.z =  1.0

    # quat_box1 =  tf.transformations.quaternion_from_euler(0.0, 0.0, 0.75)
    # p1.pose.orientation.x = quat_box1[0]
    # p1.pose.orientation.y = quat_box1[1]
    # p1.pose.orientation.z = quat_box1[2]
    # p1.pose.orientation.w = quat_box1[3]

    # planning_scene.addSolidPrimitive("table_karina", primitive1, p1.pose)

    planning_scene.addBox("table",2,1,0.71, 3.05,0.22,0.35)



    planning_scene.removeCollisionObject("table_colin")
    # planning_scene.removeCollisionObject("my_back_ground")
    # planning_scene.removeCollisionObject("my_right_ground")
    # planning_scene.removeCollisionObject("my_left_ground")
    # planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    # planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    # planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    # planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # rospy.Publisher('planning_scene', planning_scene, queue_size=10)

    # This is the wrist link not the gripper itself
    # quats = get_quaternion_from_euler(0.0,0.0,0.0)




    # gripper_frame = 'wrist_roll_link'
    # # Position and rotation of two "wave end poses"
    # # robot = RobotState()  
    # # print(robot.pose)
    # group_name = "arm_with_torso"
    # group = moveit_commander.MoveGroupCommander(group_name)
    # print("here: ",group.get_current_pose())

    # listener = tf.TransformListener()
    # listener.waitForTransform('/map','/base_link', rospy.Time(0), rospy.Duration(1))
    
    # cube_point = PointStamped()
    # cube_point.header.frame_id = '/map'
    # cube_point.header.stamp = rospy.Time(0)
    # cube_point.point.x = 0.046
    # cube_point.point.y = 0.46
    # cube_point.point.z = 1.0
    # # (trans, rot) = listener.lookupTransform('head_camera_depth_frame', 'base_link', rospy.Time(0))
    # # print(trans, rot)
    # p = listener.transformPoint('/base_link', cube_point)

    # print("in base link: ", p, tf.transformations.quaternion_from_euler(0.0, np.pi/10, 0.0))



    # quats = tf.transformations.quaternion_from_euler(0.0, np.pi/4, 0.0)
    # # # ### Figure out how to get Quaternion
    # # # # gripper_poses = [Pose(Point(0.96,0.0,0.78),
    # # # #                     Quaternion( 0.0,0.0,0.0,1.0))]
    # # #                     #   Quaternion(0.173, -0.693, -0.242, 0.657))] #,

    # # gripper_poses = [Pose(Point(0.53,-0.145,0.84+0.5),
    # #                     Quaternion( quats[0],quats[1], quats[2], quats[3]))]
    # gripper_poses = [Pose(Point(0.75,0.056,1),
    #                     Quaternion( quats[0],quats[1], quats[2], quats[3]))]
    
    # # # gripper_poses =                 [Pose(Point(0.047, 0.545, 1.822),

    # # ### Points in map frame
    # # # gripper_poses = [Pose(Point(0.05, 0.55, 1.825),
    # # #                       Quaternion(-0.274, -0.701, 0.173, 0.635))]

    # # # # Construct a "pose_stamped" message as required by moveToPose
    
    
    # gripper_pose_stamped = PoseStamped()
    # gripper_pose_stamped.header.frame_id = 'base_link'

    # # while not rospy.is_shutdown():
    # for pose in gripper_poses:
    #     # Finish building the Pose_stamped message
    #     # If the message stamp is not current it could be ignored
    #     gripper_pose_stamped.header.stamp = rospy.Time.now()
    #     # Set the message pose
    #     gripper_pose_stamped.pose = pose

    #     # Move gripper frame to the pose specified
    #     move_group.moveToPose(gripper_pose_stamped, gripper_frame)
    #     result = move_group.get_move_action().get_result()
    #     # print("state:",move_group.get_move_action.get_state())

    #     if result:
    #         # Checking the MoveItErrorCode
    #         if result.error_code.val == MoveItErrorCodes.SUCCESS:
    #             rospy.loginfo("Hello there!")
    #         else:
    #             # If you get to this point please search for:
    #             # moveit_msgs/MoveItErrorCodes.msg
    #             rospy.logerr("Arm goal in state: %s",
    #                           move_group.get_move_action().get_state())
    #     else:
    #         rospy.logerr("MoveIt! failure no result returned.")

    # # This stops all arm movement goals
    # # It should be called when a program is exiting so movement stops
    # move_group.get_move_action().cancel_all_goals()

    # print("here: ",group.get_current_pose())


### Posible Pose of ee for grabing the cube
# pose: 
#   position: 
#     x: 0.526640434474
#     y: -0.14574979736
#     z: 0.875196833975
#   orientation: 
#     x: 0.0198842106793
#     y: 0.0279655075013
#     z: 0.0237472179888
#     w: 0.999128929715)


#### Grasp pose w.r.t base link
# pose: 
#   position: 
#     x: 0.530299766859
#     y: -0.144110748845
#     z: 0.909038515128
#   orientation: 
#     x: -0.000140213682545
#     y: 0.37787028227
#     z: 0.00440368078627
#     w: 0.925848064054

#### Cube pose
# x = 0.046
# y = 0.46
# z = 0.84
