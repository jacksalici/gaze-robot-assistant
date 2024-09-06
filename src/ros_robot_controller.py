#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import subprocess 
import time
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

import toml, math

config = toml.load("config.toml")

def yaw_to_quaternion(yaw, yaw_shift=1.57):
    half_yaw = (yaw+yaw_shift) / 2
    return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]

class RobotController:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RobotController, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):

        """Initialize the moveit_commander, rospy nodes, and the robot controller."""
        #self.launch_simulation(boxes) 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_robot_controller', anonymous=True)
 
        # Instantiate necessary MoveIt classes
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.group_hand = moveit_commander.MoveGroupCommander("panda_hand")        

        self.add_rviz_model("table")

        # Create a DisplayTrajectory publisher for RViz visualization
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        rospy.sleep(2)  # Allow time for setup

    def init_boxes(self, boxes_position, boxes_yaws):
        #boxes
        for i, box in enumerate(boxes_position):
            ret = self.spawn_gazebo_model(f"box{i}", config["box_sdf_path"], box, orientation=yaw_to_quaternion(boxes_yaws[i], -0.76))
            self.add_rviz_model(f"box{i}", config["box_stl_path"], box, orientation=yaw_to_quaternion(boxes_yaws[i], 0.76))

    def move_to_position(self, position = (0.3, 0, 0.6), orientation=(1e-6, -1.0, 0, 0)):
        """Move the robot's end-effector to a specified position with orientation."""
        pose_goal = geometry_msgs.msg.Pose()
        x, y, z = position
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]


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

    def add_rviz_model(self, name, model_path=False, position = [0.5, 0.0, 0.2], size=[0.45, 1.0, 0.42 ], orientation = [0.0, 0.0, 0.0, 1.0] ):
        """Add an object to the planning scene."""
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.robot.get_planning_frame()
 
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        if not model_path:
            
            self.scene.add_box(name, pose, size)

        else:

            self.scene.add_mesh(name, pose, model_path)


    def remove_rviz_model(self, name):
        """Remove the object from the planning scene."""
        self.scene.remove_world_object(name)

    def delete_gazebo_model(self, model_name):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp = delete_model_service(model_name)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def spawn_gazebo_model(self, model_name, model_file, position, orientation = [0,0,0,1]):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            model_xml = ""
            with open(model_file, "r") as model_file:
                model_xml = model_file.read()

            pose = Pose()
            pose.position = Point(*position)
            pose.orientation = Quaternion(*orientation)

            resp = spawn_model_service(model_name, model_xml, "/", pose, "world")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    

    def shutdown_moveit(self):
        """Shut down MoveIt cleanly."""
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
 



def main():
    import argparse

    # Argument parser setup
    parser = argparse.ArgumentParser(description='Control robot actions: add, move, or remove objects in the scene.')
    parser.add_argument('--add', action='store_true', help='Add objects to the scene')
    parser.add_argument('--move', action='store_true', help='Move the robot')
    parser.add_argument('--remove', action='store_true', help='Remove objects from the scene')
    
    args = parser.parse_args()

    # Initialize the robot controller
    controller = RobotController()

    if args.add:
        table_height = 0.4

        # Add a table and a stone to the scene
        boxes = [[0.5, -0.25, table_height], [0.5, 0.25, table_height]] 
        controller.init_boxes(boxes_position=boxes, boxes_yaws=[0.0, 0.0])

        controller.move_to_position(
            position=(0.5, 0.2, 0.6),
            orientation=(1e-6, -1.0, 0.0, 0)
        )

    if args.remove:
        for i in range (config['n_boxes']):
            controller.delete_gazebo_model(f"box{i}")        
            controller.remove_rviz_model(f"box{i}")

        controller.remove_rviz_model("table")


    if args.move:
        controller.move_to_position(
            position=(0.5, 0, 0.64),
            orientation=[-0.3656115, -0.9307495, -0.005395, 0.0021192]
        )

if __name__ == "__main__":
    main()

