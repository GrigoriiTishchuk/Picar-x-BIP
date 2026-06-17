from picarx import Picarx
import time

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
    "DEBUG_MODE": True         # Set to False to stop terminal prints
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

    def stop(self):
        self.hardware.stop()

# ==========================================
# 4. MAIN EXECUTION LOOP (The Logic Flow)
# ==========================================
def get_camera_data():
    """
    Placeholder for receiving data from the Vision Team.
    Insert socket/network reading code here.
    """
    # Simulated data for testing purposes
    # return 120 
    pass

def get_ultrasonic_data():
    """
    Placeholder for receiving data from the Ultrasonic Team.
    Insert socket/network reading code here.
    """
    # Simulated data for testing purposes
    # return 15 
    pass

def get_grey_scale_data():
    """
    Placeholder for receiving data from the Grey Scale Team.
    Insert socket/network reading code here.
    """
    # Simulated data for testing purposes
    # return 0.75 
    pass

def main():
    # Initialize the car using the centralized settings
    car = AutonomousCar(SETTINGS)
    print("[INFO] System ready. Waiting for vision data...")
    
    try:
        while True:
            data = get_camera_data()

            # Handle edge case: Track is lost
            if data == "LOST" or data is None:
                if SETTINGS["DEBUG_MODE"]: 
                    print("[WARNING] Line lost! Reversing...")
                car.reverse()
                time.sleep(0.1)
                continue
            
            # Normal driving operation
            if isinstance(data, (int, float)):
                car.drive_forward()
                car.update_steering(data)
                
            time.sleep(0.05) # Prevent CPU overload
            
    except KeyboardInterrupt:
        print("\n[INFO] Manual stop triggered.")
    finally:
        car.stop()
        print("[INFO] Hardware shutdown safe.")

if __name__ == "__main__":
    main()