from aria_glasses_utils.ariaStreaming import AriaStreaming
from aria_glasses_utils.utils import *
from aria_glasses_utils.BetterAriaProvider import Streams, BetterAriaProvider, CustomCalibration
from aria_glasses_utils.BetterEyeGaze import BetterEyeGaze

import aria.sdk as aria

def main():
    import tomllib
    config = tomllib.load( open('config.toml', 'rb'))

    

    vrs_file = config["vrs_path"]
    cv2.namedWindow("test", cv2.WINDOW_NORMAL)


    eye_gaze = BetterEyeGaze(
        live=False, correct_distorsion=True, rotate_image=True, vrs_file=vrs_file, init_inference=True
    )

    for time in eye_gaze.get_time_range():
        img = eye_gaze.get_rgb_image(time_ns=time)
        img_et = eye_gaze.get_et_image(time_ns=time)
        

        yaw, pitch = eye_gaze.predict(img_et)
        gaze_center_in_cpf2, gaze_center_in_pixels2 = eye_gaze.get_gaze_center_raw(
            yaw, pitch, 1
        )

        
        cv2.circle(img, gaze_center_in_pixels2, 5, (255, 255, 0), 2)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow("test", img)

        if quit_keypress():
            break


if __name__ == "__main__":
    main()