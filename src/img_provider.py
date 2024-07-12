import numpy as np
import cv2, glob, re, tomllib, os
from detection_and_pose_estimation import DetectAndPoseEstimator
from common import inv_transformation_matrix

ROBOT_MARKER_ID = 1

if __name__ == "__main__":
    config = tomllib.load(open("config.toml", "rb"))

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
        
        
        robot_marker_index = marker_ids.flatten().tolist().index(ROBOT_MARKER_ID)
        robot_rvec = rvecs[robot_marker_index]
        
        observed_marker_index = 0
        distance_observed_marker = np.nan
        
        for i, marker in enumerate(marker_ids):
            if i == robot_marker_index:
                R = cv2.Rodrigues(np.array(rvecs[i]))[0]
                E_robot2glasses = inv_transformation_matrix(np.hstack((R, tvecs[i])))
                continue
        
            distance_marker_current = np.linalg.norm(tvecs[i].reshape(1,3)-npz_file["gaze_center_in_rgb_frame"])
            print(distance_marker_current)
            print(tvecs[i])
            if distance_observed_marker>distance_marker_current:
                observed_marker_index = i
                distance_observed_marker = distance_marker_current
            
            
        print(E_robot2glasses)
        print(marker_ids[observed_marker_index])
            
        
        frame = cv2.circle(frame, npz_file["gaze_center_in_rgb_pixels"], 2, (255, 255, 0), 2)
        
        cv2.imshow("frame", frame)
        cv2.waitKey()

    cv2.destroyAllWindows()
