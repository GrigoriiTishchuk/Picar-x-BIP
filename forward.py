from picarx import Picarx
import time

px = Picarx()

print("[INFO] Driving forward. Press Ctrl+C in the terminal to stop.")

try:
    # 1. Force the front wheels to be perfectly straight
    px.set_dir_servo_angle(0)
    time.sleep(0.5)

    # 2. Drive forward using the basic, built-in function
    # You can adjust the 30 to any speed you need (0 to 100)
    px.forward(30)
    
    # 3. Keep the script alive so the car keeps moving
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    # This block catches the Ctrl+C command
    pass
finally:
    # 4. Stop the car safely when the script closes
    px.stop()
    print("\n[INFO] Car stopped safely.")