from picarx import Picarx
from picamera2 import Picamera2
from time import sleep, strftime, localtime

px = Picarx()

# define date and time
DATE_TIME = strftime("%y%m%d_%H%M%S", localtime())

# path were images are stored
SAVE_PATH = f"/home/admin/Pictures/{DATE_TIME}.jpg"

# take screenshot
def take_photo(path):
    px.set_cam_pan_angle(5)
    px.set_cam_tilt_angle(-20)

    sleep(5)
    
    camera = Picamera2()
    config = camera.create_still_configuration(
        main={"size": (2592, 1944)}  # full 5MP resolution per hardware specs
    )
    camera.configure(config)
    camera.start()
    sleep(2)  # warm-up time

    camera.capture_file(path)
    print(f"Photo saved to: {path}")

    camera.stop()
    camera.close()

if __name__ == "__main__":
    take_photo(SAVE_PATH)