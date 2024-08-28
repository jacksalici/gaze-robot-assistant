#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class RobotController:
    def __init__(self):
        """Initialize the moveit_commander, rospy nodes, and the robot controller."""
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_robot_controller', anonymous=True)

        # Instantiate necessary MoveIt classes
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.group_hand = moveit_commander.MoveGroupCommander("panda_hand")

        # Create a DisplayTrajectory publisher for RViz visualization
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        rospy.sleep(2)  # Allow time for setup

    def move_to_position(self, position = (0.3, 0, 0.6), orientation=(1e-6, -1.0, 0, 0)):
        """Move the robot's end-effector to a specified position with orientation."""
        pose_goal = geometry_msgs.msg.Pose()
        x, y, z = position
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = orientation[0]
        pose_goal.orientation.x = orientation[1]
        pose_goal.orientation.y = orientation[2]
        pose_goal.orientation.z = orientation[3]

        # Set the goal pose for the end-effector
        self.group_arm.set_pose_target(pose_goal)

        # Plan and execute the motion
        self.group_arm.go(wait=True)
        self.group_arm.stop()

        # Clear the pose target after planning with it
        self.group_arm.clear_pose_targets()

    def open_gripper(self):
        """Open the robot's gripper."""
        joint_goal = self.group_hand.get_current_joint_values()
        joint_goal[0] = 0.04  
        joint_goal[1] = 0.04  
        self.group_hand.go(joint_goal, wait=True)
        self.group_hand.stop()

    def close_gripper(self):
        """Close the robot's gripper."""
        joint_goal = self.group_hand.get_current_joint_values()
        joint_goal[0] = 0.01 
        joint_goal[1] = 0.01  

        self.group_hand.go(joint_goal, wait=True)
        self.group_hand.stop()

    def add_table(self, timeout=40):
        """Add a table object to the planning scene."""
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0.516428
        table_pose.pose.position.y = 0.000584
        table_pose.pose.position.z = 0.2099115

        box_name = "table_franka"
        self.scene.add_box(box_name, table_pose, size=(0.453157, 0.642049, 0.419823))


    def add_stone(self, timeout=40):
        """Add a stone object to the planning scene."""
        stone_pose = geometry_msgs.msg.PoseStamped()
        stone_pose.header.frame_id = self.robot.get_planning_frame()
        stone_pose.pose.orientation.w = 1.0
        stone_pose.pose.position.x = 0.500286
        stone_pose.pose.position.y = -0.221972
        stone_pose.pose.position.z = 0.475172

        box_name = "stone"
        self.scene.add_box(box_name, stone_pose, size=(0.025, 0.032, 0.064))


    def remove_table(self, timeout=40):
        """Remove the table object from the planning scene."""
        box_name = "table_franka"
        self.scene.remove_world_object(box_name)

  

    def shutdown_moveit(self):
        """Shut down MoveIt cleanly."""
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

def main():
    # Initialize the robot controller
    controller = RobotController()

    # Add a table and a stone to the scene
    controller.add_table()

    controller.open_gripper()

    controller.move_to_position(
        position = (0.5, -0.22, 0.6),
        orientation = (1e-6, -1.0, 0.0, 0)
    )

    controller.close_gripper()

    controller.move_to_position(
        position = (0.5, 0.2, 0.6),
        orientation = (1e-6, -1.0, 0.0, 0)
    )

    controller.shutdown_moveit()

if __name__ == "__main__":
    main()
