from picamera2 import Picamera2
from time import strftime, localtime
import cv2

# define date and time
DATE_TIME = strftime("%y%m%d_%H%M%S", localtime())

# path were images are stored
SAVE_PATH = f"/home/admin/Pictures/{DATE_TIME}.jpg"

def save_screenshot(frame):
    DATE_TIME = strftime("%y%m%d_%H%M%S", localtime())
    path = f"{SAVE_DIR}{DATE_TIME}.jpg"
    cv2.imwrite(path, frame)
    print(f"Screenshot saved to: {path}")

def main():
    camera = Picamera2()
    config = camera.create_preview_configuration(
        main={"size": (1280, 720), "format": "BGR888"}  # BGR for direct OpenCV compatibility
    )
    camera.configure(config)
    camera.start()

    print("Live feed started. Press 's' to screenshot, 'q' to quit.")

    while True:
        frame = camera.capture_array()
        cv2.imshow("PiCar-X Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            save_screenshot(frame)
        elif key == ord('q'):
            break

    camera.stop()
    camera.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()