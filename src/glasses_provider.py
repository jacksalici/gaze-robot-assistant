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

def main():
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
    
    with ctrl_c_handler() as ctrl_c:
        while not (quit_keypress() or ctrl_c):
            img, success = provider.get_frame(Streams.RGB)
            if success:
                last_img = img
                
            try:
                corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(last_img)
                last_img = detectAndPoseEstimator.drawAllFrames(last_img, corners, marker_ids, rvecs, tvecs)    
                
            except:
                print("MARKER NOT PRINT")
                
                
            img_et, success = provider.get_frame(Streams.ET)
            if success:
                yaw, pitch = eye_gaze.predict(img_et)

                gaze_center_in_cpf, gaze_center_in_pixels = eye_gaze.get_gaze_center_raw(
                    yaw, pitch, 1
                )
                cv2.circle(last_img, gaze_center_in_pixels, 5, (255, 255, 0), 2)
           
                    
            cv2.imshow('RGB', cv2.cvtColor(last_img, cv2.COLOR_RGB2BGR))
        
    provider.unsubscribe()


if __name__ == "__main__":
    main()
