from picamera2 import Picamera2
from time import strftime, localtime

def start_camera_session():
    # Initialize the camera
    camera = Picamera2()
    
    # Create preview configuration to keep the video window open
    config = camera.create_preview_configuration(
        main={"size": (2592, 1944)}  # full 5MP resolution per hardware specs
    )
    
    camera.configure(config)
    camera.start()
    
    print("Camera preview started. You can now adjust your frame.")
    
    try:
        # Continuous loop to keep the camera running
        while True:
            # Wait for user input in the terminal
            user_input = input("Press ENTER to take a photo, or type 'q' (and ENTER) to quit: ")
            
            # Exit the loop if the user types 'q'
            if user_input.lower() == 'q':
                break
            
            # Generate a new timestamp for each photo so they don't overwrite
            current_time = strftime("%y%m%d_%H%M%S", localtime())
            save_path = f"/home/admin/Pictures/{current_time}.jpg"
            
            # Capture the image
            camera.capture_file(save_path)
            print(f"Photo successfully saved to: {save_path}")
            
    finally:
        # Ensure the camera stops and closes properly when exiting
        print("Stopping camera...")
        camera.stop()
        camera.close()

if __name__ == "__main__":
    start_camera_session()