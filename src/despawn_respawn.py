#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model_service(model_name)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def spawn_model(model_name, model_file, position):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        model_xml = ""
        with open(model_file, "r") as model_file:
            model_xml = model_file.read()

        pose = Pose()
        pose.position = Point(*position)
        pose.orientation = Quaternion(0, 0, 0, 1)  # No rotation

        resp = spawn_model_service(model_name, model_xml, "/", pose, "world")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

if __name__ == "__main__":
    rospy.init_node('despawn_and_spawn_objects')

    # List of models to delete
    models_to_delete = ['pick_tray', 'place_tray', 'stone']

    # Delete models
    for model in models_to_delete:
        success = delete_model(model)
        if success:
            rospy.loginfo(f"Successfully deleted model: {model}")
        else:
            rospy.logwarn(f"Failed to delete model: {model}")

    # Model files (paths to SDF files)
    model_files = {
        'pick_tray': '/home/cpavone/robot_franka/src/franka_ros/franka_gazebo/models/pick_tray/model.sdf',
        'place_tray': '/home/cpavone/robot_franka/src/franka_ros/franka_gazebo/models/place_tray/model.sdf',
        'stone': '/home/cpavone/robot_franka/src/franka_ros/franka_gazebo/models/stone/model.sdf',
    }

    # New positions for models (x, y, z)
    new_positions = {
        'pick_tray': (0.508578, -0.215704, 0.419773),
        'place_tray': (0.504103, 0.219654, 0.419764),
        'stone': (0.500286, -0.221972, 0.475172),
    }

    rospy.sleep(1)

    # Spawn models at new positions
    for model, position in new_positions.items():
        success = spawn_model(model, model_files[model], position)
        if success:
            rospy.loginfo(f"Successfully spawned model: {model} at position {position}")
        else:
            rospy.logwarn(f"Failed to spawn model: {model}")

    rospy.loginfo("Despawn and respawn process complete.")
