#!/usr/bin/env python3


import rospy
import moveit_ompl_control		# Use Custom Helper Class
from moveit_ompl_control import moveManipulator

# Pulled over from support class
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



def run_program(rb):
    print("Executing " + __file__)

    # Table Corner
    rb.goto_joint_posn([-1.1, 0.51, -1.20, 0.05, 0.14, 1.05])
    rb.goto_Quant_Orient([0.11, -0.21, 0.20, 1, -3.72, 0.00, 0.00])


    # Build Short Cartesian Path
    waypoints = []

    wpose = rb.move_group.get_current_pose().pose
    wpose.position.y -= 0.5
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.3
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.4
    waypoints.append(copy.deepcopy(wpose))


    # Build path @ step size resolution (0.01m)
    (plan, fraction) = rb.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)


    # Execute Cartesian Trajectory Plan
    rb.move_group.execute(plan, wait=True)



if __name__ == "__main__":

    # Init a ros node
    rospy.init_node('myMover', anonymous=True)

    # Init MoveIt Commander
    rb = moveManipulator('gp7')

    # Configure Motion Planner
    rb.set_vel(0.2)
    rb.set_accel(0.2)

    # Move Robot to Home
    rb.goto_all_zeros()

    # Run Mover Program
    run_program(rb)

