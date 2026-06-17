from picarx import Picarx
import time

px = Picarx()

# --- MOTOR TUNING VALUES ---
# Change these values independently to test and balance the wheels
LEFT_SPEED = 30
RIGHT_SPEED = -25

print("--- Independent Motor Calibration Test ---")
print(" w -> Drive Forward (Using custom left/right speeds)")
print(" s -> Stop")
print(" q -> Quit")
print("------------------------------------------")

try:
    # 1. Force the front wheels to be perfectly straight before testing
    px.set_dir_servo_angle(0)
    time.sleep(0.5) 

    while True:
        cmd = input("Enter command (w/s/q) and press Enter: ").strip().lower()
        
        if cmd == 'w':
            print(f"[ACTION] Left Motor Power: {LEFT_SPEED} | Right Motor Power: {RIGHT_SPEED}")
            
            # Send individual power values to each motor
            # Note: Motor 1 is usually Left, Motor 2 is Right. 
            # If your car spins backwards or incorrectly, just swap the 1 and 2.
            px.set_motor_speed(1, LEFT_SPEED)
            px.set_motor_speed(2, RIGHT_SPEED)
            
        elif cmd == 's':
            print("[ACTION] Motors stopped.")
            # Stop each motor individually by setting speed to 0
            px.set_motor_speed(1, 0)
            px.set_motor_speed(2, 0)
            
        elif cmd == 'q':
            break
            
        else:
            continue

except KeyboardInterrupt:
    pass
finally:
    # Always ensure motors stop when the script is closed
    px.stop()
    print("\n[INFO] Test finished safely.")