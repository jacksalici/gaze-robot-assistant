import cv2
import cv2.aruco as aruco
from pathlib import Path
import tomllib

def generate_aruco_marker(id, size, save_path):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    img = aruco.generateImageMarker(aruco_dict, id, size)
    cv2.imwrite(str(save_path), img)

config = tomllib.load(open('config.toml', 'rb'))   
path = Path(config['aruco_export_path'])
generate_aruco_marker(249, 200, path / 'aruco_marker_249.png')
generate_aruco_marker(1, 200, path / 'aruco_marker_01.png')
generate_aruco_marker(2, 200, path / 'aruco_marker_02.png')
generate_aruco_marker(3, 200, path / 'aruco_marker_03.png')
generate_aruco_marker(4, 200, path / 'aruco_marker_04.png')
