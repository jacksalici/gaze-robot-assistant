import numpy as np
import cv2, glob, re, tomllib, os
from detection_and_pose_estimation import DetectAndPoseEstimator



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
        
        frame = cv2.circle(frame, npz_file["gaze_center_in_rgb_pixels"], 2, (255, 255, 255))
        
        cv2.imshow("frame", frame)
        cv2.waitKey()

    cv2.destroyAllWindows()
