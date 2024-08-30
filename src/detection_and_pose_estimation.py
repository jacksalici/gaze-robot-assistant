import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math
import sys


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles; returns **roll** is rotation around *x*,
    **pitch** is rotation around *y*, and **yaw** is rotation around *z*. [rad, counterclockwise]
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class DetectAndPoseEstimator():
    def __init__(self, K, D) -> None:
        
        aruco_dict = cv2.aruco.DICT_6X6_250
        
        self.camera_matrix = K
        self.camera_distortion = D

        parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(aruco_dict), parameters
        )

        self.markerLength = 0.0706

        self.objPoints = np.array(
            [
                [-self.markerLength / 2, self.markerLength / 2, 0],
                [self.markerLength / 2, self.markerLength / 2, 0],
                [self.markerLength / 2, -self.markerLength / 2, 0],
                [-self.markerLength / 2, -self.markerLength / 2, 0],
            ],
            dtype=np.float32,
        )

    def solve(self, frame, verbose=False):
        corners, marker_ids = [], []
        
        (corners, marker_ids, _) = self.detector.detectMarkers(frame)

        rvecs = []
        tvecs = []
        
        if marker_ids is not None:

            for i, marker_id in enumerate(marker_ids):

                success, R, t = cv2.solvePnP(
                    self.objPoints, corners[i], self.camera_matrix, self.camera_distortion
                )

                if success:
                    rvecs.append(R)
                    tvecs.append(t)
                    
                    if (verbose):
                    
                        roll_x, pitch_y, yaw_z = self.rvec2RPY(rvecs[i])

                        roll_x = math.degrees(roll_x)
                        pitch_y = math.degrees(pitch_y)
                        yaw_z = math.degrees(yaw_z)

                        print(f"""MARKER {marker_id}
Roll Pitch Yaw: {roll_x, pitch_y, yaw_z}
X Y Z: {tvecs[i][0][0], tvecs[i][1][0], tvecs[i][2][0]}
""")

        return corners, marker_ids, rvecs, tvecs

    def rvec2RPY(self, rvec):
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec))[0]
        r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()

        roll_x, pitch_y, yaw_z = euler_from_quaternion(*quat)
        
        return roll_x, pitch_y, yaw_z
    
    def drawFrame(self, frame, rvec, tvec, size = 0.05):
        return cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.camera_distortion,
                        rvec,
                        tvec,
                        size
                    )
    
    def drawAllFrames(self, frame, corners, marker_ids, rvecs, tvecs):
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
        for i, marker_id in enumerate(marker_ids):
            frame = self.drawFrame(frame, rvecs[i], tvecs[i])
        return frame

        
def resize_image(image, max_width, max_height):
    h, w = image.shape[:2]
    scale = min(max_width / w, max_height / h)
    return cv2.resize(image, (int(w * scale), int(h * scale)))

if __name__ == "__main__":
    
    K = np.array([[800, 0, 665], [0, 800, 400], [0, 0, 1]], dtype=np.float32) #intrisic of Google Pixel
    D = np.array([[0, 0, 0, 0, 0]], dtype=np.float32)

    screen_width = 1920  
    screen_height = 1080  

    detectAndPoseEstimator = DetectAndPoseEstimator(K, D)
    frame = cv2.imread("imgs/photo_example.jpg")
    corners, marker_ids, rvecs, tvecs = detectAndPoseEstimator.solve(frame)
    
    cv2.imshow("frame", detectAndPoseEstimator.drawAllFrames(frame, corners, marker_ids, rvecs, tvecs))

    cv2.waitKey()

    cv2.destroyAllWindows()

