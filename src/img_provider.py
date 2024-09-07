from detection_and_pose_estimation import DetectAndPoseEstimator
import cv2
import numpy as np
import os
import tomllib
import time

from cobot_boxes import *
import matplotlib.pyplot as plt
from socket_com import Client

import re, glob


def resize_image(image, max_width, max_height):
    h, w = image.shape[:2]
    scale = min(max_width / w, max_height / h)
    return cv2.resize(image, (int(w * scale), int(h * scale)))

if __name__ == "__main__":
    config = tomllib.load(open("config.toml", "rb"))
    BOX_POSITION_SAVED = False

    boxes = {}
    cobot = None
    
    
    socket_client = Client(config["server_ip"])
    cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)

    plt.ion()
    fig, ax = plt.subplots()

    ax.set_xlim(-1, +1)
    ax.set_ylim(-0.6, +0.6)
    
    imgs_folder, npzs_folder = config["imgs_folder"], config["gaze_output_folder"]
    g = glob.glob(os.path.join(imgs_folder, '*.jpg'))
    for img_file in g:
        img_id = re.search(r'\d+', img_file).group()
            
        try:
            npz_file = np.load(os.path.join(npzs_folder, f'img{img_id}.npz'))
        except:
            continue
                
        fx, fy, cx, cy = npz_file["rbg_camera_intrinsic"]

        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        D = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    
        detectAndPoseEstimator = DetectAndPoseEstimator(K, D)
        frame = cv2.imread(os.path.join(imgs_folder, img_file))
        
        corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(frame)
        frame = detectAndPoseEstimator.drawAllFrames(frame, corners, marker_ids, rvecs, tvecs)
        
        #################################################################################
        ##### generate once the box positions, update every time the robot position #####
        #################################################################################
         
        if len(marker_ids) - 1 == config["n_boxes"] and BOX_POSITION_SAVED == False: # when all the markers (the boxes plus the cobot's one) are seen initialize the boxes objects
            boxes, cobot = generateBoxes(marker_ids, config["robot_aruco_id"], rvecs, tvecs)
            os.system("say 'boxes generated' &")
            BOX_POSITION_SAVED = True
            
            msg = CobotSocketMessage(
                init=1,
                trigger_robot=0,
                glasses_position=[0,0,0],
                target_position=[0,0,0],
                boxes_position= [box.getPositionInRobotFrame().tolist() for _, box in boxes.items()],
                boxes_yaws = [box.getRotationInRobotFrame().tolist()[2] for _, box in boxes.items() ]
            )
            
            socket_client.send_message(dumps_CobotSocketMessage(msg))
            
            
        elif BOX_POSITION_SAVED:
            updateCobot(cobot, marker_ids, rvecs, tvecs)
                    
        
        gaze_center_in_rgb_frame = npz_file["gaze_center_in_rgb_frame"] 
        frame = cv2.circle(frame, gaze_center_in_rgb_frame, 2, (255, 255, 0), 2)
        
        if cobot:
                
            ##########################
            ##### 2D sym PLOTTER #####
            ##########################
            
            ax.clear() 
            ax.set_xlim(-1, +1)
            ax.set_ylim(-1, +1)


            for id, box in boxes.items():
                    p = box.getPositionInRobotFrame()
                    ax.scatter(p[0], p[1], marker='s', color='blue')
                    ax.text(p[0] + 0.005, p[1] + 0.005, f'{id}', fontsize=12, color='blue')  
                    
            cobot_position = cobot.getPositionInRobotFrame()
            ax.scatter(cobot_position[0], cobot_position[1], marker='^', color='black')
            glasses_position = cobot.trasformInRobotFrame(np.array([0,0,0]))
            ax.scatter(glasses_position[0], glasses_position[1], marker='o', color='red') 
        
            gaze_center_in_robot_frame = cobot.trasformInRobotFrame(gaze_center_in_rgb_frame)
            ax.scatter(gaze_center_in_robot_frame[0], gaze_center_in_robot_frame[1], marker='x', color='red') 
                
            ##########################################
            ##### check which box is being gazed #####
            ##########################################
                        
            for id, box in boxes.items():
                trigger = box.isGazed(gaze_center_in_robot_frame)
                    
                if trigger:
                    msg = CobotSocketMessage(
                        init=0,
                        trigger_robot=1,
                        glasses_position=gaze_center_in_robot_frame.tolist(),
                        target_position=box.getPositionInRobotFrame().tolist(),
                        boxes_position= [box.getPositionInRobotFrame().tolist() for _, box in boxes.items()],
                        boxes_yaws = [box.getRotationInRobotFrame().tolist()[2] for _, box in boxes.items() ]
                    )
                    os.system(f"say 'cobot triggered toward box {id}' &")
                    socket_client.send_message(dumps_CobotSocketMessage(msg))
                        
        plt.draw()

        
        
        
        screen_width, screen_height = 1920, 1080  # Example screen resolution
        h, w = frame.shape[:2]
        if w > screen_width or h > screen_height:
            frame = resize_image(frame, screen_width, screen_height)
        cv2.imshow("RGB", frame)
        cv2.waitKey()

    cv2.destroyAllWindows()
