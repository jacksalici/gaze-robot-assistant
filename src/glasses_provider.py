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

import tomllib
import time

from common import *



def main():
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
    
    last_img = [[0, 0], [0, 0]]
    et_image = [[0, 0], [0, 0]]

    boxes = {}
    cobot = None
    
    with ctrl_c_handler() as ctrl_c:
        while not (quit_keypress() or ctrl_c):
            img, success = provider.get_frame(Streams.RGB)
            if success:
                last_img = img
                    
                if True:
                    corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(last_img)
                    
                    if len(marker_ids)== 0:
                        continue
                    
                    last_img = detectAndPoseEstimator.drawAllFrames(last_img, corners, marker_ids, rvecs, tvecs)
                    
                    if len(marker_ids) == config["n_boxes"] and BOX_POSITION_SAVED == False:
                        boxes, cobot = generateBoxes(marker_ids, config["robot_aruco_id"], rvecs, tvecs)
                        BOX_POSITION_SAVED = True
                        
                        for id, box in boxes.items():
                            print(box.getPositionInRobotFrame())
                    
                else:#except Exception as e:
                    print("ERROR: Error during markers")
                    print(e)
                    
                
            img_et, success2 = provider.get_frame(Streams.ET, rotated=False, undistorted=False)
            if success2:
                yaw, pitch = eye_gaze.predict(img_et)
                
                gaze_center_in_cpf, gaze_center_in_pixels = eye_gaze.get_gaze_center_raw(
                    yaw, pitch, config["gaze_depth"]
                )
               
            try:
                cv2.circle(last_img, gaze_center_in_pixels, 5, (255, 255, 0), 2)         
                cv2.imshow('RGB', cv2.cvtColor(last_img, cv2.COLOR_RGB2BGR))    
            except:
                pass
            
    provider.unsubscribe()


if __name__ == "__main__":
    main()
