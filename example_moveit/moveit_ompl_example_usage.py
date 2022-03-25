#!/usr/bin/env python3


import rospy
import moveit_ompl_control		# Use Custom Helper Class


main():

  try:
    print("")
    print("----------------------------------------------------------")
    print("           MoveIt Control Toolkit Using Python            ")
    print("----------------------------------------------------------")
    print("Example developed by ACBUYNAK. Spring 2021")
    print("Note: You will be unable to interrup program if running\n from terminal. Alternatively use IDLE console.")
    print("Press Enter to advance script when prompted.")
    print("")


    ## Initial Values & Controls
    # ############################
    rospy.init_node('myMover', anonymous=False)
    robot = moveManipulator('mh5l')
    robot.set_accel(0.2)
    robot.set_vel(0.2)


    ## User Input
    # ############################
    raw_input('Use this to pause the python script before continuing. Press <enter> to continue')


    ## Path Planning & Execution
    # ############################

    # Move to MFG Default Position: All-Zeros
    raw_input('Go to All-Zeros Position <enter>')
    robot.goto_all_zeros()

    #  Cartesian Pose Instruction
    raw_input('Go to Example Cart Pose <enter>')
    pose_cart = [0.3,-0.4,0.8,0,radians(90),0]
    robot.goto_Quant_Orient(pose_cart)

    # Example Joint Pose Instruction
    raw_input('Go to Example Joint Pose <enter>')
    pose_joint = [0.2,0.2,0.2,0.2,0.2,0.2]
    robot.goto_joint_posn(pose_joint)


    ## IO Control
    # ############################
    robot.send_io(10011,1)     # Sending true to IO address 10011

    
    ## Pickup Objects + Collision Objects
    # ############################
    robot.add_object()
    robot.attach_object()
    robot.detach_object()
    robot.remove_object()


    ## Return to ALL-ZEROS for Best Practices
    # ############################
    raw_input('Per Best Practices, return to All-Zeros Joint Position <enter>')
    robot.goto_all_zeros()

    print("============ Complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()