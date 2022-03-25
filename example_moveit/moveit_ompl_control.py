#!/usr/bin/env python

#####################################################
##      MoveIt Control Toolkit Using Python        ##
##                                                 ##
##   Capabilities Demonstrated                     ##
##   * Individual Joint Control                    ##
##   * Cartesian Waypoint Motion                   ##
##   * Quanterion Position Naviation               ##
##   * Add/Remove Collision Object (pickup object) ##
##   * Saving Motion Plan as BAG                   ##
##   * Execute BAGged motion plan                  ##
##   * Motoman IO Control (gripper on/off)         ##
##                                                 ##
#####################################################

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Author: Adam Buynak

#####################################################

### IMPORTS
#
# `moveit_commander` namespace allows Python MoveIt interfaces.
# Includes a `MoveGroupCommander`_, `PlanningSceneInterface`_, and `RobotCommander`_ class
# 
# Additional imports allow used for support, ROS messages, and etc.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#####################################################
## SUPPORT CLASSES AND FUNCTIONS
##
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class moveManipulator(object):
  """moveManipulator Class"""

  def __init__(self, group_name):
    super(moveManipulator, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)

    # Setup Variables needed for Moveit_Commander
    self.object_name = ''
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = group_name         # CHANGE THIS TO MATCH YOUR ROBOT'S MOVEIT CONFIG!
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.planning_frame = self.move_group.get_planning_frame()
    self.eef_link = self.move_group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()

  def set_vel(self,max_vel):
    ## Wrapper for Moveit Commander's max_velocity
    ## Allowed range...   0.0 <= max_vel <= 1.0
    self.move_group.set_max_velocity_scaling_factor(max_vel)
  def set_accel(self,max_accel):
    ## Wrapper for Moveit Commander's max_acceleration
    ## Allowed range...   0.0 <= max_vel <= 1.0
    self.move_group.set_max_acceleration_scaling_factor(max_accel)

  def goto_all_zeros(self):
    ## Go to "ALL-Zeros" position
    ## Get Current Position & Go to "All-Zeros" Position
    ## Trajectory Type: JOINT MOTION defined by joint position

    # Get Current Position
    joint_goal = self.move_group.get_current_joint_values()

    # Define "All-Zeros" Position
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # Send action to move-to defined position
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def plan_cartesian_throw_path(self, scale=1):
    ## Plan Cartesian Path to throw object

    # Specify a list of waypoints
    waypoints = []

    # Example - Commented Out
    #wpose = self.move_group.get_current_pose().pose
    #wpose.position.z += scale * 0.1  # Move up (z)
    #wpose.position.x += scale * 0.8  # Forward (x)
    #waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 5 cm
    # which is why we will specify 0.05 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def goto_Euler_Orient(self,pose):
    ## GOTO Pose Using Cartesian + Quaternion Pose

    # Get Current Orientation in Quanternion Format
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
    #q_poseCurrent = self.move_group.get_current_pose().pose.orientation
    #print(q_poseCurrent)

    # Using Quaternion's for Angle
    # Conversion from Euler(rotx,roty,rotz) to Quaternion(x,y,z,w)
    # Euler Units: RADIANS
    # http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
    # http://wiki.ros.org/tf2/Tutorials/Quaternions
    # http://docs.ros.org/en/api/geometry_msgs/html/msg/Quaternion.html

    # Convert Euler Orientation Request to Quanternion
    q_orientGoal = quaternion_from_euler(pose[3],pose[4],pose[5],axes='sxyz')

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    pose_goal.orientation.x = q_orientGoal[0]
    pose_goal.orientation.y = q_orientGoal[1]
    pose_goal.orientation.z = q_orientGoal[2]
    pose_goal.orientation.w = q_orientGoal[3]

    self.move_group.set_pose_target(pose_goal)

    ## Call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def goto_Quant_Orient(self,pose):
    ## GOTO Pose Using Cartesian + Quaternion Pose

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]

    self.move_group.set_pose_target(pose_goal)

    ## Call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def goto_joint_posn(self,joint_goal):
    ## Go to Joint Defined position
    ## Get Current Position & Go to "All-Zeros" Position
    ## Trajectory Type: JOINT MOTION defined by joint position

    # Send action to move-to defined position
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def send_io(self, address, request):
    ## Wrapper for rosservice to open/close gripper using Read/Write IO

    # Wait for ros services to come up
    rospy.wait_for_service('read_single_io')
    rospy.wait_for_service('write_single_io')

    # Create Handle for Service Proxy's
    try:
        read_single_io = rospy.ServiceProxy('read_single_io', ReadSingleIO)
        write_single_io = rospy.ServiceProxy('write_single_io', WriteSingleIO)
    except rospy.ServiceException as e:
        print("Gripper IO Service Call failed: %s"%e)

    # Send 'Write' IO Message
    try:
      write_status = write_single_io(address, request)
    except:
      print("An exception occured. Unable to write to Single IO.")


    # Call Read Service to check current position
    read_status = read_single_io(address).value
    if read_status:
        print('IO is True')
    else:
        print('IO is False')

    return read_status

  def execute_plan(self, plan):
    ## Execute a Plan
    ## Use execute if you would like the robot to follow a plan that has already been computed:
    self.move_group.execute(plan, wait=True)

  def add_object(self, timeout=4):
    ## Add object Element to Collision Scene

    # Create object
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "link_t"
    object_pose.pose.orientation.w = 1.0
    object_pose.pose.position.x = 0.07 # slightly above the end effector
    self.object_name = "object"

    # Add object to scene
    self.scene.add_box(self.object_name, object_pose, size=(0.05, 0.3, 0.1))

    # Alternively, Use Mesh of Object. (mixed success with this. See moveit webpage)
    #self.scene.add_mesh(self.object_name, object_pose, filename="$(find object)/meshes/object-model.stl", size=(1,1,1))

    return self.wait_for_state_update(object_is_known=True, timeout=timeout)
  def attach_object(self, timeout=4):
    ## Attaching object to the Robot
    grasping_group = 'manipulator'
    touch_links = self.robot.get_link_names(group=grasping_group)

    # Attach object to Robot EEF
    self.scene.attach_box(self.eef_link, self.object_name, touch_links=touch_links)
    #self.scene.attach_mesh(self.eef_link, self.object_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(object_is_attached=True, object_is_known=False, timeout=timeout)
  def detach_object(self, timeout=4):
    ## Detaching object from the Robot
    self.scene.remove_attached_object(self.eef_link, name=self.object_name)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(object_is_known=True, object_is_attached=False, timeout=timeout)
  def remove_object(self, timeout=4):
    ## Removing Objects from the Planning Scene
    ## **Note:** The object must be detached before we can remove it from the world
    self.scene.remove_world_object(self.object_name)

  def wait_for_state_update(self, object_is_known=False, object_is_attached=False, timeout=4):
    ## wait_for_scene_update
    ## This helps with collision planning.

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the object is in attached objects
      attached_objects = self.scene.get_attached_objects([self.object_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the object is in the scene.
      # Note that attaching the object will remove it from known_objects
      is_known = self.object_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (object_is_attached == is_attached) and (object_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

