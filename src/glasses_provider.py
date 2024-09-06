from aria_glasses_utils.common import *
from aria_glasses_utils.BetterAriaProvider import (
    Streams,
    BetterAriaProvider,
    CustomCalibration,
)
from aria_glasses_utils.BetterEyeGaze import BetterEyeGaze

import aria.sdk as aria

from detection_and_pose_estimation import DetectAndPoseEstimator
import cv2
import numpy as np
import os
import tomllib
import time

from cobot_boxes import *
import matplotlib.pyplot as plt
from socket_com import Client


def main():
    
    ################################
    ##### INIT ARIA and SOCKET #####
    ################################
    
    BOX_POSITION_SAVED = False

    config = tomllib.load(open("config.toml", "rb"))

    cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)

    provider = BetterAriaProvider(live=True)

    custom_calibrations, device_calibration = provider.get_calibration()

    fx, fy, cx, cy = custom_calibrations[
        Streams.RGB
    ].rotated_pinhole_calib.projection_params()

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    D = np.array([0, 0, 0, 0, 0], dtype=np.float32)

    detectAndPoseEstimator = DetectAndPoseEstimator(K, D)

    eye_gaze = BetterEyeGaze(
        custom_calibrations,
        device_calibration,
        correct_distorsion=True,
        rotate_image=True,
        init_inference=True,
    )
    rbg2cpf_camera_extrinsic = provider.calibration_device.get_transform_cpf_sensor(
                    Streams.RGB.label()
                ).to_matrix()
    
    last_img = [[0, 0], [0, 0]]
    et_image = [[0, 0], [0, 0]]

    boxes = {}
    cobot = None
    
    
    socket_client = Client(config["server_ip"])
    
    plt.ion()
    fig, ax = plt.subplots()

    ax.set_xlim(-1, +1)
    ax.set_ylim(-0.6, +0.6)
    
    #####################
    ##### MAIN LOOP #####
    #####################
    
    with ctrl_c_handler() as ctrl_c:
        while not (quit_keypress() or ctrl_c):
            
            img, success = provider.get_frame(Streams.RGB)
            
            if success: #when an img rgb is captured
                last_img = img
                    
                corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(last_img)
                
                if len(marker_ids)== 0:
                    continue
                
                
                last_img = detectAndPoseEstimator.drawAllFrames(last_img, corners, marker_ids, rvecs, tvecs)
                
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
                        boxes_yaws = [box.getPositionInRobotFrame().tolist()[2] for _, box in boxes.items() ]
                    )
                    
                    socket_client.send_message(dumps_CobotSocketMessage(msg))
                    
                    
                elif BOX_POSITION_SAVED:
                    updateCobot(cobot, marker_ids, rvecs, tvecs)
                    
                                                     
                    

                
            img_et, et_success = provider.get_frame(Streams.ET, rotated=False, undistorted=False)
            if et_success: #when an et is captured
                yaw, pitch = eye_gaze.predict(img_et)
                
                gaze_center_in_cpf, gaze_center_in_pixels = eye_gaze.get_gaze_center_raw(
                    yaw, pitch, config["gaze_depth"]
                )
                                
                gaze_center_in_rgb_frame=(
                    np.linalg.inv(rbg2cpf_camera_extrinsic)
                    @ np.append(gaze_center_in_cpf, [1])
                )[:3]
                
                    
            
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
            
                if et_success:
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
                                boxes_yaws = [box.getPositionInRobotFrame().tolist()[2] for _, box in boxes.items() ]

                            )
                            os.system(f"say 'cobot triggered toward box {id}' &")
                            socket_client.send_message(dumps_CobotSocketMessage(msg))
                            
            plt.draw()

            try:
                cv2.circle(last_img, gaze_center_in_pixels, 5, (255, 255, 0), 2)         
                cv2.imshow('RGB', cv2.cvtColor(last_img, cv2.COLOR_RGB2BGR))    
            except:
                pass
            
    provider.unsubscribe()


if __name__ == "__main__":
    main()
