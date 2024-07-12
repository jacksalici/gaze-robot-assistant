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

def main():
    import tomllib

    config = tomllib.load(open("config.toml", "rb"))

    vrs_file = config["vrs_path"]
    cv2.namedWindow("test", cv2.WINDOW_NORMAL)

    provider = BetterAriaProvider(live=False, vrs=vrs_file)

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

    for time in provider.get_time_range(100_000_000):
        img, _ = provider.get_frame(Streams.RGB, time_ns=time)
        img_et, _ = provider.get_frame(Streams.ET, time_ns=time)

        yaw, pitch = eye_gaze.predict(img_et)

        gaze_center_in_cpf2, gaze_center_in_pixels2 = eye_gaze.get_gaze_center_raw(
            yaw, pitch, 1
        )

        corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(img)
        img = detectAndPoseEstimator.drawAllFrames(img, corners, marker_ids, rvecs, tvecs)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow("test", img)

        cv2.waitKey()
        if quit_keypress():
            break


if __name__ == "__main__":
    main()
