import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2

SERIAL_PORT = "/dev/ttyAMA0"   # Try /dev/ttyUSB0 if using USB serial
BAUD_RATE = 115200

signStatus = "none"

candidate_status = "none"
candidate_start_time = None
last_sent_status = "none"

DETECTION_HOLD_TIME = 1.0
ser = None
picam2 = None


def initialize_runtime():
    global ser
    global picam2

    if ser is None:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    if picam2 is None:
        picam2 = Picamera2()
        picam2.configure(
            picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
        )
        picam2.start()


def shutdown_runtime():
    global ser
    global picam2

    if picam2 is not None:
        picam2.stop()
        picam2 = None

    if ser is not None:
        ser.close()
        ser = None


def clean_mask(mask, size=5):
    kernel = np.ones((size, size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def circularity(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0
    return 4 * np.pi * area / (perimeter * perimeter)


def distance_label(w, h, frame):
    frame_area = frame.shape[0] * frame.shape[1]
    sign_area = w * h
    ratio = sign_area / frame_area

    if ratio < 0.035:
        return "_far"

    return ""


def find_colored_sign(mask, min_area=120):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = 0

    for c in contours:
        area = cv2.contourArea(c)

        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(c)

        aspect = w / float(h)
        circ = circularity(c)

        if aspect < 0.60 or aspect > 1.45:
            continue

        if circ < 0.32:
            continue

        score = area * circ

        if score > best_score:
            best_score = score
            best = (x, y, w, h)

    return best


def is_stop_sign(frame, box):
    x, y, w, h = box
    roi = frame[y:y + h, x:x + w]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red1 = cv2.inRange(hsv, np.array([0, 135, 70]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 135, 70]), np.array([180, 255, 255]))
    red_mask = red1 | red2

    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 135]),
        np.array([180, 85, 255])
    )

    red_pixels = cv2.countNonZero(red_mask)
    white_pixels = cv2.countNonZero(white_mask)

    total_pixels = w * h

    red_ratio = red_pixels / total_pixels
    white_ratio = white_pixels / total_pixels

    if red_ratio < 0.35:
        return False

    if white_ratio < 0.035:
        return False

    center_text_area = white_mask[
        int(h * 0.30):int(h * 0.72),
        int(w * 0.12):int(w * 0.88)
    ]

    center_white = cv2.countNonZero(center_text_area)

    if center_white < max(18, total_pixels * 0.012):
        return False

    return True


def detect_arrow_direction(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 115]),
        np.array([180, 105, 255])
    )

    white_mask = clean_mask(white_mask, 3)

    h, w = white_mask.shape

    cropped = white_mask[
        int(h * 0.22):int(h * 0.78),
        int(w * 0.08):int(w * 0.92)
    ]

    ch, cw = cropped.shape

    if ch <= 0 or cw <= 0:
        return None

    left_pixels = cv2.countNonZero(cropped[:, :cw // 2])
    right_pixels = cv2.countNonZero(cropped[:, cw // 2:])
    total_white = left_pixels + right_pixels

    if total_white < max(30, roi.shape[0] * roi.shape[1] * 0.015):
        return None

    difference = abs(left_pixels - right_pixels)
    confidence = difference / max(total_white, 1)

    if confidence < 0.12:
        return None

    if left_pixels > right_pixels:
        return "left"

    return "right"


def detect_sign(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red1 = cv2.inRange(hsv, np.array([0, 135, 70]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 135, 70]), np.array([180, 255, 255]))
    red_mask = clean_mask(red1 | red2, 5)

    blue_mask = cv2.inRange(
        hsv,
        np.array([85, 55, 30]),
        np.array([140, 255, 255])
    )
    blue_mask = clean_mask(blue_mask, 5)

    red_box = find_colored_sign(red_mask, min_area=120)

    if red_box is not None:
        x, y, w, h = red_box

        if is_stop_sign(frame, red_box):
            return "stop" + distance_label(w, h, frame)

    blue_box = find_colored_sign(blue_mask, min_area=120)

    if blue_box is not None:
        x, y, w, h = blue_box

        margin = 4

        if x > margin and x + w < frame.shape[1] - margin:
            roi = frame[y:y + h, x:x + w]
            direction = detect_arrow_direction(roi)

            if direction is not None:
                return direction + distance_label(w, h, frame)

    return "none"


def update_stable_detection(raw_status):
    global candidate_status
    global candidate_start_time
    global last_sent_status
    global signStatus

    now = time.time()

    if raw_status == "none":
        candidate_status = "none"
        candidate_start_time = None
        signStatus = "none"
        return

    if raw_status != candidate_status:
        candidate_status = raw_status
        candidate_start_time = now
        return

    if candidate_start_time is None:
        candidate_start_time = now
        return

    if now - candidate_start_time >= DETECTION_HOLD_TIME:
        signStatus = candidate_status

        if signStatus != last_sent_status and ser is not None:
            message = signStatus + "\n"
            ser.write(message.encode("utf-8"))
            print("Serial out:", signStatus)
            last_sent_status = signStatus


def capture_frame():
    if picam2 is None:
        raise RuntimeError("Picamera2 runtime is not initialized.")
    return picam2.capture_array()


def main():
    initialize_runtime()

    try:
        print("PiCar-X sign detector running...")

        while True:
            frame = capture_frame()
            raw_status = detect_sign(frame)
            update_stable_detection(raw_status)
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        shutdown_runtime()


if __name__ == "__main__":
    main()
