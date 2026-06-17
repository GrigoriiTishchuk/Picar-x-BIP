from picarx import Picarx
from vilib import Vilib
import time

px = Picarx()

# Set starting angles (0 means perfectly centered)
pan_angle = 0
tilt_angle = 0

px.set_cam_pan_angle(pan_angle)
px.set_cam_tilt_angle(tilt_angle)

# Start the web camera stream
Vilib.camera_start()
Vilib.display(local=False, web=True)

print("[INFO] Video stream started!")
print("--- Camera Controls ---")
print(" w -> Tilt Up")
print(" s -> Tilt Down")
print(" a -> Pan Left")
print(" d -> Pan Right")
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
        elif cmd == 'q':
            break
        else:
            continue
            
        # Hardware limits: Prevent forcing the servos past their physical max/min
        pan_angle = max(-90, min(90, pan_angle))
        tilt_angle = max(-35, min(35, tilt_angle)) # Tilt has a tighter restriction
        
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