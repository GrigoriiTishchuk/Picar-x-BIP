from picarx import Picarx
import time
from navigation_rule import evaluate_intersection


# ==========================================
# 1. CENTRAL CONFIGURATION (Change parameters ONLY here)
# ==========================================
SETTINGS = {
    "SPEED_FORWARD": 30,       # Base speed for driving (0-100)
    "SPEED_REVERSE": -20,      # Speed for backing up when lost
    "STEERING_LIMIT": 45,      # Max physical angle left/right
    "KP": 0.25,                # Proportional gain
    "KI": 0.0,                 # Integral gain
    "KD": 0.1,                 # Derivative gain
    "DEBUG_MODE": True,         # Set to False to stop terminal prints
    "LINE_REF": 10000,         # Threshold for white line detection
    "TURN_SPEED": 30,          # Speed used during the 90-degree turn
    "TURN_DURATION": 2.5,       # Time (seconds) it takes to complete a 90-deg turn
    "STOP_DURATION": 3.0        # Time (seconds) to wait at a STOP sign before proceeding
}

# ==========================================
# 2. PID CONTROLLER MODULE (The Math Brain)
# ==========================================
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.prev_time = time.time()

    def compute(self, error):
        """Calculates the steering adjustment based on the error."""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.001 # Prevent division by zero
        # Proportional term
        p_out = self.kp * error
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative
        # Update memory for the next loop
        self.prev_error = error
        self.prev_time = current_time
        return p_out + d_out

# ==========================================
# 3. CAR HARDWARE MODULE (The Physical Body)
# ==========================================
class AutonomousCar:
    def __init__(self, settings):
        self.cfg = settings
        self.hardware = Picarx()
        self.pid = PIDController(self.cfg["KP"], self.cfg["KI"], self.cfg["KD"])
        
    def clamp_steering(self, angle):
        """Forces the computed angle to stay within safe hardware limits."""
        limit = self.cfg["STEERING_LIMIT"]
        return max(-limit, min(limit, angle))

    def drive_forward(self):
        self.hardware.forward(self.cfg["SPEED_FORWARD"])

    def reverse(self):
        self.hardware.set_dir_servo_angle(0) # Straighten wheels first
        self.hardware.backward(abs(self.cfg["SPEED_REVERSE"]))

    def update_steering(self, error):
        """Passes error to PID, clamps the result, and turns the wheels."""
        raw_angle = self.pid.compute(error)
        safe_angle = self.clamp_steering(raw_angle)
        
        self.hardware.set_dir_servo_angle(safe_angle)
        
        if self.cfg["DEBUG_MODE"]:
            print(f"[DEBUG] Error: {error} | Raw Angle: {raw_angle:.2f} | Safe Angle: {safe_angle:.2f}")
    
    def execute_90_degree_turn(self, direction):
        if self.cfg["DEBUG_MODE"]:
            print(f"[ACTION] Initiating 90-degree turn to the: {direction.upper()}")
        # Turn the wheels
        if direction == "left":
            self.hardware.set_dir_servo_angle(-35)
        elif direction == "right":
            self.hardware.set_dir_servo_angle(35)
        # Drive the arc
        self.hardware.forward(self.cfg["TURN_SPEED"])
        time.sleep(self.cfg["TURN_DURATION"])
        
        # Straighten out
        self.hardware.set_dir_servo_angle(0)

    def execute_stop_sign(self):
        """Halts the car completely for the calibrated stop duration."""
        if self.cfg["DEBUG_MODE"]:
            print("[ACTION] STOP sign triggered. Halting at intersection...")
        self.hardware.stop()
        time.sleep(self.cfg["STOP_DURATION"])
        if self.cfg["DEBUG_MODE"]:
            print("[ACTION] Stop duration complete. Proceeding straight.")
        # Note: We don't need to explicitly drive forward here, because the 
        # main loop will automatically resume `drive_forward()` on its next cycle!

    def check_ground_sensors(self):
        """Fetches data and updates navigation actions using separate file logic."""
        gm_val_list = self.hardware.get_grayscale_data()
        action, clear_memory = evaluate_intersection(
            gm_val_list, 
            self.remembered_sign, 
            self.cfg["LINE_REF"],
            self.cfg["DEBUG_MODE"]
        )
        # Route the action to the correct physical maneuver
        if action in ["left", "right"]:
            self.execute_90_degree_turn(action)
        elif action == "stop":
            self.execute_stop_sign()
        if clear_memory:
            self.remembered_sign = None

    def stop(self):
        self.hardware.stop()

# ==========================================
# 4. MAIN EXECUTION LOOP (The Logic Flow)
# ==========================================
def get_camera_line_error():
    return 0 

def check_camera_for_signs():
    return None

def main():
    # Initialize the car using the centralized settings
    car = AutonomousCar(SETTINGS)
    print("[INFO] System ready. Waiting for vision data...")
    
    try:
        while True:
            #  Check for signs via camera
            detected_sign = check_camera_for_signs()
            if detected_sign:
                car.remembered_sign = detected_sign
            #  Check for intersections (Uses our external file)
            car.check_ground_sensors()
            #  Handle normal line following
            line_error = get_camera_line_error()

            if line_error == "LOST" or line_error is None:
                car.reverse()
                time.sleep(0.1)
                continue
            
            if isinstance(line_error, (int, float)):
                car.drive_forward()
                car.update_steering(line_error)
                
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\n[INFO] Manual stop triggered.")
    finally:
        car.stop()
        print("[INFO] Hardware shutdown safe.")

if __name__ == "__main__":
    main()