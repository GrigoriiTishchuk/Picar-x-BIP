from picarx import Picarx
import time

px = Picarx()

# Initial starting speeds
LEFT_SPEED = 30
RIGHT_SPEED = 30
is_driving = False

def update_motors():
    """Applies power to the motors only if the car is in 'driving' mode."""
    if is_driving:
        px.set_motor_speed(1, LEFT_SPEED)
        # We apply the negative sign here to fix the physical mirror effect of the right motor
        px.set_motor_speed(2, -RIGHT_SPEED) 
    else:
        px.stop()

def print_status():
    status = "DRIVING" if is_driving else "STOPPED"
    print(f"[{status}] Left Power: {LEFT_SPEED} | Right Power: {RIGHT_SPEED}")

print("--- Live Motor Calibration ---")
print(" w -> START Driving")
print(" s -> STOP Driving")
print(" e -> Left Motor +2")
print(" d -> Left Motor -2")
print(" r -> Right Motor +2")
print(" f -> Right Motor -2")
print(" q -> Quit")
print("-" * 30)

try:
    # Force the front wheels to be perfectly straight
    px.set_dir_servo_angle(0)
    time.sleep(0.5)
    
    print_status()

    while True:
        cmd = input("Command: ").strip().lower()
        
        if cmd == 'w':
            is_driving = True
            update_motors()
            print_status()
            
        elif cmd == 's':
            is_driving = False
            update_motors()
            print_status()
            
        elif cmd == 'e':
            LEFT_SPEED += 2
            update_motors()
            print_status()
            
        elif cmd == 'd':
            LEFT_SPEED -= 2
            update_motors()
            print_status()
            
        elif cmd == 'r':
            RIGHT_SPEED += 2
            update_motors()
            print_status()
            
        elif cmd == 'f':
            RIGHT_SPEED -= 2
            update_motors()
            print_status()
            
        elif cmd == 'q':
            break
            
        else:
            continue

except KeyboardInterrupt:
    pass
finally:
    # Safe shutdown
    px.stop()
    print("\n[INFO] Calibration test finished. Motors stopped.")