import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math
import sys


def euler_from_quaternion(x, y, z, w):
    # TODO please double check this
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
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


def main():
    aruco_dict = cv2.aruco.DICT_6X6_250
    camera_calibration_parameters_filename = "calibration.yaml"

    cv_file = cv2.FileStorage(
        camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ
    )
    camera_matrix = cv_file.getNode("K").mat()
    camera_distortion = cv_file.getNode("D").mat()
    cv_file.release()

    parameters = cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(aruco_dict), parameters
    )

    markerLength = 0.0706

    objPoints = np.array(
        [
            [-markerLength / 2, markerLength / 2, 0],
            [markerLength / 2, markerLength / 2, 0],
            [markerLength / 2, -markerLength / 2, 0],
            [-markerLength / 2, -markerLength / 2, 0],
        ],
        dtype=np.float32,
    )

    while True:

        frame = cv2.imread("imgs/photo_example.jpg")

        (corners, marker_ids, _) = detector.detectMarkers(frame)

        if marker_ids is not None:

            # Draw a square around detected markers
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

            rvecs = []
            tvecs = []

            for i, marker_id in enumerate(marker_ids):

                success, R, t = cv2.solvePnP(
                    objPoints, corners[i], camera_matrix, camera_distortion
                )

                if success:
                    rvecs.append(R)
                    tvecs.append(t)

                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i]))[0]
                    r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
                    quat = r.as_quat()

                    roll_x, pitch_y, yaw_z = euler_from_quaternion(*quat)

                    roll_x = math.degrees(roll_x)
                    pitch_y = math.degrees(pitch_y)
                    yaw_z = math.degrees(yaw_z)

                    print(f"""MARKER {marker_id}
Roll Pitch Yaw: {roll_x, pitch_y, yaw_z}
X Y Z: {tvecs[i][0][0], tvecs[i][1][0], tvecs[i][2][0]}
""")
                    

                    frame = cv2.drawFrameAxes(
                        frame,
                        camera_matrix,
                        camera_distortion,
                        rvecs[i],
                        tvecs[i],
                        0.05,
                    )

        cv2.imshow("frame", frame)

        if cv2.waitKey() & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
