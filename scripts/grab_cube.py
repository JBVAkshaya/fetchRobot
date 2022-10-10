#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
import numpy as np

from moveit_msgs.msg import MoveItErrorCodes, RobotState
import moveit_commander

from moveit_python import MoveGroupInterface,PlanningSceneInterface
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
import tf
from shape_msgs.msg import SolidPrimitive

import actionlib
import control_msgs.msg
import rospy
import sys, time
#import tmc_control_msgs.msg
# HSR uses: tmc_control_msgs.msg.GripperApplyEffortActionGoal (?)

CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
ACTION_SERVER = 'gripper_controller/gripper_action'

import argparse
import subprocess
import sys
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface


class MoveItThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["roslaunch", "fetch_moveit_config", "move_group.launch", "--wait"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_moveit_running():
    output = subprocess.check_output(["rosnode", "info", "move_group"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class TuckThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                if move_thread:
                    move_thread.stop()

                # On success quit
                # Stopping the MoveIt thread works, however, the action client
                # does not get shut down, and future tucks will not work.
                # As a work-around, we die and roslaunch will respawn us.
                # rospy.signal_shutdown("done")
                # sys.exit(0)
                return

    def stop(self):
        if self.client:
            self.client.get_move_action().cancel_goal()
        # Stopping the MoveIt thread works, however, the action client
        # does not get shut down, and future tucks will not work.
        # As a work-around, we die and roslaunch will respawn us.
        rospy.signal_shutdown("failed")
        sys.exit(0)

class TuckArmTeleop:

    def __init__(self):
        self.tuck_button = rospy.get_param("~tuck_button", 6)  # default button is the down button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.tucking = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.tucking:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't tuck
                rospy.loginfo("Stopping tuck thread")
                self.tuck_thread.stop()
            return
        try:
            if msg.buttons[self.tuck_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Tuck the arm
                    self.tucking = True
                    rospy.loginfo("Starting tuck thread")
                    self.tuck_thread = TuckThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("tuck_button is out of range")

class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35   # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_SERVER, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.
        The `goal` has type:
            <class 'control_msgs.msg._GripperCommandGoal.GripperCommandGoal'>
        with a single attribute, accessed via `goal.command`, which consists of:
            position: 0.0
            max_effort: 0.0
        by default, and is of type:
            <class 'control_msgs.msg._GripperCommand.GripperCommand'>
        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(10))


def wait_for_time():
    """Wait for simulated time to begin.
    A useful method. Note that rviz will display the ROS Time in the bottom left
    corner. For Gazebo, just click the play button if it's paused to start.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def heading(yaw):
	"""A helper function to getnerate quaternions from yaws."""
	q = quaternion_from_euler(0, 0, yaw)
	return Quaternion(*q)

### target point
#  x: 0.0464204209962
#   y: 0.4572592885
#   z: 0.844115513654

if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('move')

	# Make a goal.  This has a coordinate frame, a time stamp, and a target pose.  The Target pose is a
	# Point (where the z component should be zero) and a Quaternion (for the orientation).
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = Pose(Point(-0.37-0.25,0.08, 0), heading((np.pi/180.0)*20.0))

	# An action client for move base
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	move_base.wait_for_server()

	# Send the goal ot move base, and wait for a result.  We can put a timeout on the wait, so that we don't
	# get stuck here for unreachable goals.  Once we're done, get the state from move base.
	move_base.send_goal(goal)
	success = move_base.wait_for_result(rospy.Duration(120))
	state = move_base.get_state()

	# Did everything work as expects?
	if success and state == GoalStatus.SUCCEEDED:
		print('Made it!')
        
        # Create move group interface for a fetch robot
        move_group = MoveGroupInterface("arm_with_torso", "map")
    
        planning_scene = PlanningSceneInterface("/map")

        gripper_frame = 'wrist_roll_link'
        group_name = "arm_with_torso"
        
        quats = tf.transformations.quaternion_from_euler(0.0, np.pi/10, 0.0)
        gripper_poses = [Pose(Point(0.1-0.3,0.21,0.85),
                            Quaternion( quats[0],quats[1], quats[2], quats[3])),
                            Pose(Point(0.1-0.2,0.21,0.85),
                            Quaternion( quats[0],quats[1], quats[2], quats[3])),
                            Pose(Point(0.1-0.1,0.21,0.85),
                            Quaternion( quats[0],quats[1], quats[2], quats[3])),
                            Pose(Point(0.1,0.21,0.85),
                            Quaternion( quats[0],quats[1], quats[2], quats[3]))]
        
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = '/map'

        wait_for_time()
        time_delay = 1
        use_delay = True

        print("Now forming the gripper")
        gripper = Gripper()
        if use_delay:
            time.sleep(time_delay)

        gripper.open()
        print("gripper now open")
        if use_delay:
            time.sleep(time_delay)

        # while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            # Move gripper frame to the pose specified
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()
            # print("state:",move_group.get_move_action.get_state())

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Hello there!")

                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")
        wait_for_time()
        time_delay = 1
        use_delay = True

        print("Now forming the gripper")
        gripper = Gripper()
        if use_delay:
            time.sleep(time_delay)

        gripper.close()
        print("gripper now closed")
        if use_delay:
            time.sleep(time_delay)

        ### Tuck arm
        rospy.loginfo("Tucking the arm")
        TuckThread()

        time_delay = 10
        if use_delay:
            time.sleep(time_delay)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(-0.37-0.25,0.08, 0), heading((np.pi/180.0)*200.0))

        # An action client for move base
        move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base.wait_for_server()

        # Send the goal ot move base, and wait for a result.  We can put a timeout on the wait, so that we don't
        # get stuck here for unreachable goals.  Once we're done, get the state from move base.
        move_base.send_goal(goal)
        success = move_base.wait_for_result(rospy.Duration(120))
        state = move_base.get_state()

        # Create move group interface for a fetch robot
        move_group1 = MoveGroupInterface("arm_with_torso", "base_link")

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        planning_scene1 = PlanningSceneInterface("base_link")
        # planning_scene.removeCollisionObject("my_front_ground")
        # planning_scene.removeCollisionObject("my_back_ground")
        # planning_scene.removeCollisionObject("my_right_ground")
        # planning_scene.removeCollisionObject("my_left_ground")
        # planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        # planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        # planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        # planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # This is the wrist link not the gripper itself
        gripper_frame = 'wrist_roll_link'
        # Position and rotation of two "wave end poses"
        gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                            Quaternion(0.173, -0.693, -0.242, 0.657)),
                        Pose(Point(0.047, 0.545, 1.822),
                            Quaternion(-0.274, -0.701, 0.173, 0.635))]

        # Construct a "pose_stamped" message as required by moveToPose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        while not rospy.is_shutdown():
            for pose in gripper_poses:
                # Finish building the Pose_stamped message
                # If the message stamp is not current it could be ignored
                gripper_pose_stamped.header.stamp = rospy.Time.now()
                # Set the message pose
                gripper_pose_stamped.pose = pose

                # Move gripper frame to the pose specified
                move_group1.moveToPose(gripper_pose_stamped, gripper_frame)
                result = move_group1.get_move_action().get_result()

                if result:
                    # Checking the MoveItErrorCode
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Hello there!")
                    else:
                        # If you get to this point please search for:
                        # moveit_msgs/MoveItErrorCodes.msg
                        rospy.logerr("Arm goal in state: %s",
                                    move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        move_group1.get_move_action().cancel_all_goals()

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        move_group.get_move_action().cancel_all_goals()
    # else:
	# 	move_base.cancel_goal()
	# 	print('Problem...')
