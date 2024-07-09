import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math
import sys

aruco_dictionary_name = "DICT_6X6_250"
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'


# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


camera_calibration_parameters_filename = "calibration_chessboard.yaml"


def euler_from_quaternion(x, y, z, w):
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
    """
    Main method of the program.
    """
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
        print(f"[INFO] ArUCo tag of '{ARUCO_DICT}' is not supported")
        sys.exit(0)

    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
        camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ
    )
    mtx = cv_file.getNode("K").mat()
    dst = cv_file.getNode("D").mat()
    cv_file.release()

    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(aruco_dictionary_name))
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(
        ARUCO_DICT[aruco_dictionary_name]
    )
    
    parameters =  cv2.aruco.DetectorParameters()

    detector = cv2.aruco.ArucoDetector(this_aruco_dictionary, parameters)


    # Start the video stream
    #cap = cv2.VideoCapture(0)
    
    markerLength = 10  # example value, replace with your actual value

    objPoints = np.array([
        [-markerLength / 2,  markerLength / 2, 0],
        [ markerLength / 2,  markerLength / 2, 0],
        [ markerLength / 2, -markerLength / 2, 0],
        [-markerLength / 2, -markerLength / 2, 0]
    ], dtype=np.float32)

    while True:

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        #ret, frame = cap.read()
        frame = cv2.imread('singlemarkersoriginal.jpg')

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = detector.detectMarkers(
            frame
        )
        

        # Check that at least one ArUco marker was detected
        if marker_ids is not None:

            # Draw a square around detected markers in the video frame
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

            trash = []
            rvecs = []
            tvecs = []
            
            
     
            for i, marker_id in enumerate(marker_ids):
                
                nada, R, t = cv2.solvePnP(objPoints, corners[i], mtx, dst)
                
                
                rvecs.append(R)
                tvecs.append(t)
                trash.append(nada)
                
                print(R)

    

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i]))[0]
                r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()


                roll_x, pitch_y, yaw_z = euler_from_quaternion(
                    *quat
                )

                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                
                
                print("transform_translation: {}".format(tvecs[i]))
         
                print("roll_x: {}".format(roll_x))
                print("pitch_y: {}".format(pitch_y))
                print("yaw_z: {}".format(yaw_z))
                print()

                # Draw the axes on the marker
                frame = cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 10)

        # Display the resulting frame
        cv2.imshow("frame", frame)

        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Close down the video stream
    #cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()




