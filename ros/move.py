#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def initialize_moveit():
    """Initialize the moveit_commander and rospy nodes."""
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_to_position', anonymous=True)

    # Instantiate necessary MoveIt classes
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander("panda_arm")
    group_hand = moveit_commander.MoveGroupCommander("panda_hand")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    return robot, scene, group_arm, group_hand, display_trajectory_publisher

def move_to_position(group_arm, position = (0.3, 0, 0.6), orientation=(1e-6, -1.0, 0, 0)):
    """Move the robot's end-effector to a specified position with orientation."""
    # Create the goal pose
    pose_goal = geometry_msgs.msg.Pose()
    x, y, z = position
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.w = orientation[0]
    pose_goal.orientation.x = orientation[1]
    pose_goal.orientation.y = orientation[2]
    pose_goal.orientation.z = orientation[3]

    group_arm.set_pose_target(pose_goal)

    plan = group_arm.go(wait=True)

    group_arm.stop()

    group_arm.clear_pose_targets()

def open_gripper(group_hand):
    """Open the robot's gripper."""

    joint_goal = group_hand.get_current_joint_values()
    joint_goal[0] = 0.04 
    joint_goal[1] = 0.04  

    group_hand.go(joint_goal, wait=True)
    group_hand.stop()

def close_gripper(group_hand):
    """Close the robot's gripper."""

    joint_goal = group_hand.get_current_joint_values()
    joint_goal[0] = 0.00  
    joint_goal[1] = 0.00  

    group_hand.go(joint_goal, wait=True)
    group_hand.stop()

def shutdown_moveit():
    """Shut down MoveIt cleanly."""
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

def main():
    robot, scene, group_arm, group_hand, display_trajectory_publisher = initialize_moveit()


   

    open_gripper(group_hand)

    move_to_position(
        group_arm,
        position = (0.46, -0.49, 0),
        orientation = (1e-6, -1.0, 0.0, 0)
    )

    close_gripper(group_hand)

    move_to_position(
        group_arm,
        position = (0.5, 0.2, 0.6),
        orientation = (1e-6, -1.0, 0.0, 0)
    )



    shutdown_moveit()

if __name__ == "__main__":
    main()
