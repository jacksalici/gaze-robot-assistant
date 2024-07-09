import cv2
import cv2.aruco as aruco

def generate_aruco_marker(id, size, save_path):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    img = aruco.generateImageMarker(aruco_dict, id, size)
    cv2.imwrite(save_path, img)

generate_aruco_marker(42, 200, 'aruco_marker_42.png')
