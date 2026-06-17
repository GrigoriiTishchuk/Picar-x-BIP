from picarx import Picarx
from vilib import Vilib
import time
import os

px = Picarx()

# 1. Create a folder for photos in the current directory
PHOTO_DIR = "./saved_photos"
if not os.path.exists(PHOTO_DIR):
    os.makedirs(PHOTO_DIR)

# Set starting angles (0 means perfectly centered)
pan_angle = 0
tilt_angle = 0

px.set_cam_pan_angle(pan_angle)
px.set_cam_tilt_angle(tilt_angle)

# Start the web camera stream
Vilib.camera_start()
Vilib.display(local=False, web=True)

print("[INFO] Video stream started!")
print(f"[INFO] Photos will be saved in: {os.path.abspath(PHOTO_DIR)}")
print("--- Camera Controls ---")
print(" w -> Tilt Up")
print(" s -> Tilt Down")
print(" a -> Pan Left")
print(" d -> Pan Right")
print(" x -> Take Photo (Shoot!)")
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
            # Generate a unique name using the current date and time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            photo_name = f"img_{timestamp}"
            
            # The Vilib library takes the photo and automatically adds the .jpg extension
            Vilib.take_photo(photo_name=photo_name, path=PHOTO_DIR)
            print(f"[CAMERA] SNAP! Photo saved as: {photo_name}.jpg")
            continue # Skip moving the camera, just take the photo
        elif cmd == 'q':
            break
        else:
            continue
            
        # Hardware limits: Prevent forcing the servos past their physical max/min
        pan_angle = max(-90, min(90, pan_angle))
        tilt_angle = max(-35, min(35, tilt_angle)) 
        
        # Send the calculated angles to the motors
        px.set_cam_pan_angle(pan_angle)
        px.set_cam_tilt_angle(tilt_angle)
        
        # Print the current status in the terminal
        print(f"Current Angle -> Pan (Left/Right): {pan_angle}° | Tilt (Up/Down): {tilt_angle}°")
        
except KeyboardInterrupt:
    pass
finally:
    # Reset camera to center and safely close the stream
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    Vilib.camera_close()
    print("\n[INFO] Program closed safely.")