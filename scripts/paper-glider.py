#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of The Ohio State University nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Adam Buynak

################################################################################

## IMPORTS
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

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

# Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler


## BEGIN MAIN CODE FUNCTIONS
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


class ThrowingArm(object):
  """ThrowingArm Class"""
  def __init__(self):
    super(ThrowingArm, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('node_ThrowingArm', anonymous=True)

    # Setup Variables needed for Moveit_Commander
    self.glider_name = ''
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "bot_mh5"
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
  def goto_airplane_pickup(self):
    ## Go to "Airplane Pickup" position
    ## Trajectory Type: JOINT MOTION defined by joint position

    # Get Current Position
    joint_goal = self.move_group.get_current_joint_values()

    # Define "Airplane Pickup" Position
    joint_goal[0] = -1.41
    joint_goal[1] = 1.08
    joint_goal[2] = 0.21
    joint_goal[3] = 0
    joint_goal[4] = -0.70
    joint_goal[5] = 0

    # Send action to move-to defined position
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  def plan_cartesian_throw_path(self, scale=1):
    ## Plan Cartesian Path to throw glider

    # Specify a list of waypoints
    waypoints = []

    wpose = self.move_group.get_current_pose().pose
    wpose.position.z += scale * 0.1  # Move up (z)
    wpose.position.x += scale * 0.8  # Forward (x)
    waypoints.append(copy.deepcopy(wpose))



    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.05,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def goto_Quant_Orient(self,pose):
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

  def act_gripper(self, request):
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
      write_status = write_single_io(10010, request)
    except:
      print("An exception occured. Unable to write to Single IO.")


    # Call Read Service to check current position
    read_status = read_single_io(10011).value
    if read_status:
        print('Gripper is Closed')
    else:
        print('Gripper is Open')

    return read_status

  def execute_plan(self, plan):
    ## Execute a Plan
    ## Use execute if you would like the robot to follow a plan that has already been computed:
    self.move_group.execute(plan, wait=True)
  def wait_for_state_update(self, glider_is_known=False, glider_is_attached=False, timeout=4):
    ## wait_for_scene_update
    ## Serves to ensure that the paper-airplane simulated object has been attached to simulated robot model
    ## This helps with collision planning. Not as important for paper-glider project, but is best practice.

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the glider is in attached objects
      attached_objects = self.scene.get_attached_objects([self.glider_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the glider is in the scene.
      # Note that attaching the glider will remove it from known_objects
      is_known = self.glider_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (glider_is_attached == is_attached) and (glider_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_glider(self, timeout=4):
    ## Add Glider Element to Collision Scene

    # Create glider
    glider_pose = geometry_msgs.msg.PoseStamped()
    glider_pose.header.frame_id = "link_t"
    glider_pose.pose.orientation.w = 1.0
    glider_pose.pose.position.x = 0.07 # slightly above the end effector
    self.glider_name = "glider"

    # Add Glider to scene
    self.scene.add_box(self.glider_name, glider_pose, size=(0.05, 0.3, 0.1))
    #self.scene.add_mesh(self.glider_name, glider_pose, filename="$(find paper-glider)/meshes/glider-model.stl", size=(1,1,1))

    return self.wait_for_state_update(glider_is_known=True, timeout=timeout)
  def attach_glider(self, timeout=4):
    ## Attaching Glider to the Robot
    grasping_group = 'bot_mh5'
    touch_links = self.robot.get_link_names(group=grasping_group)

    # Attach Glider to Robot EEF
    self.scene.attach_box(self.eef_link, self.glider_name, touch_links=touch_links)
    #self.scene.attach_mesh(self.eef_link, self.glider_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(glider_is_attached=True, glider_is_known=False, timeout=timeout)
  def detach_glider(self, timeout=4):
    ## Detaching Glider from the Robot
    self.scene.remove_attached_object(self.eef_link, name=self.glider_name)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(glider_is_known=True, glider_is_attached=False, timeout=timeout)
  def remove_glider(self, timeout=4):
    ## Removing Objects from the Planning Scene
    ## **Note:** The object must be detached before we can remove it from the world
    self.scene.remove_world_object(self.glider_name)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(glider_is_attached=False, glider_is_known=False, timeout=timeout)




def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "     AIMS Lab Demo: The Paper Glider Throwing Arm!        "
    print "----------------------------------------------------------"
    print "Program Developed by ACBUYNAK. Fall 2020"
    print "Press Enter to advance script when prompted."
    print ""
    print "============ Initialize the MH5 Robot & Go-To All-Zeros Position"
    robot = ThrowingArm()
    robot.set_accel(0.2)
    robot.set_vel(0.2)

    raw_input('Go to All-Zeros Position <enter>')
    robot.goto_all_zeros()



    print "============ Pickup Airplane"
    raw_input('Move to PreDefined Airplane Pickup <enter>')

    #Pickup w/ Standoff
    pose_pickupGlider = [0.1,-0.50,0.3,0,radians(90),radians(90)]
    robot.goto_Quant_Orient(pose_pickupGlider)

    #Descend onto Glider
    pose_pickupGlider = [0.1,-0.50,0.18,0,radians(90),radians(90)]
    robot.goto_Quant_Orient(pose_pickupGlider)

    #Use Gripper to Pikcup Glider
    raw_input('Actuate Gripper and Throw! <enter>')
    robot.act_gripper(1)
    robot.add_glider()
    robot.attach_glider()

    #Liftup off table
    pose_pickupGlider = [0.1,-0.50,0.3,0,radians(90),radians(90)]
    robot.goto_Quant_Orient(pose_pickupGlider)



    print "============ Throwing Start Position"
    #raw_input('Move to Throw Position <enter>')
    pose_throwStart = [0,-0.95,1.40,0,-0.6,radians(90)]
    robot.goto_joint_posn(pose_throwStart)


    print "============ Throw Glider"
    #raw_input('Throw Glider <enter>')
    robot.set_accel(0.99)
    robot.set_vel(0.99)
    #throwPlan, throwFraction = robot.plan_cartesian_throw_path()
    #robot.execute_plan(throwPlan)
    pose_throwEnd = [0,0.33,1.4,0.1,0.1,radians(90)]
    robot.goto_joint_posn(pose_throwEnd)

    #raw_input('Release Glider <enter>')
    robot.act_gripper(0)
    robot.detach_glider()
    robot.remove_glider()

    raw_input('Per Best Practices, return to All-Zeros Joint Position <enter>')
    robot.goto_all_zeros()

    print "============ Demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
