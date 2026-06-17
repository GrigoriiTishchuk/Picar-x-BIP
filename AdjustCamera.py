from picarx import Picarx
import cv2
import time
import os

px = Picarx()

# 1. Create a folder for high-resolution photos
PHOTO_DIR = "./high_res_photos"
if not os.path.exists(PHOTO_DIR):
    os.makedirs(PHOTO_DIR)

# 2. Setup standard OpenCV camera and force HD resolution
print("[INFO] Warming up High-Res camera. Please wait...")
cap = cv2.VideoCapture(0)

# Set resolution to 720p (You can try 1920 and 1080 for 1080p if the hardware supports it)
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Check if the camera opened successfully
if not cap.isOpened():
    print("[ERROR] Cannot open camera. Make sure Vilib or other scripts are not using it!")
    exit()

# 3. Set starting angles
pan_angle = 0
tilt_angle = 0
px.set_cam_pan_angle(pan_angle)
px.set_cam_tilt_angle(tilt_angle)

print(f"[INFO] Camera ready at {FRAME_WIDTH}x{FRAME_HEIGHT}!")
print(f"[INFO] Photos will be saved in: {os.path.abspath(PHOTO_DIR)}")
print("--- Camera Controls ---")
print(" w -> Tilt Up")
print(" s -> Tilt Down")
print(" a -> Pan Left")
print(" d -> Pan Right")
print(" x -> Take High-Res Photo (Shoot!)")
print(" q -> Quit test")
print("-" * 23)

try:
    while True:
        cmd = input("Enter command and press Enter: ").strip().lower()
        
        if cmd == 'w':
            tilt_angle += 5
        elif cmd == 's':
            tilt_angle -= 5
        elif cmd == 'a':
            pan_angle += 5
        elif cmd == 'd':
            pan_angle -= 5
        elif cmd == 'x':
            # Read a fresh, high-quality frame from the camera
            ret, frame = cap.read()
            if ret:
                # Generate a unique name
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                photo_name = f"hd_img_{timestamp}.jpg"
                filepath = os.path.join(PHOTO_DIR, photo_name)
                
                # Save the image using OpenCV
                cv2.imwrite(filepath, frame)
                print(f"[CAMERA] SNAP! High-Res photo saved as: {photo_name}")
            else:
                print("[ERROR] Failed to grab frame from camera.")
            continue # Skip moving servos, just return to waiting for input
        elif cmd == 'q':
            break
        else:
            continue
            
        # Hardware limits: Prevent forcing the servos
        pan_angle = max(-90, min(90, pan_angle))
        tilt_angle = max(-35, min(35, tilt_angle)) 
        
        # Send angles to motors
        px.set_cam_pan_angle(pan_angle)
        px.set_cam_tilt_angle(tilt_angle)
        
        print(f"Current Angle -> Pan: {pan_angle}° | Tilt: {tilt_angle}°")
        
except KeyboardInterrupt:
    pass
finally:
    # Safely release the camera and reset servos
    cap.release()
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    print("\n[INFO] Camera released and program closed safely.")