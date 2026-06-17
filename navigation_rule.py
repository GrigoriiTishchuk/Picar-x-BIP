import time

def evaluate_intersection(gm_val_list, current_memory, line_ref, debug_mode=True):
    """
    Analyzes the grayscale sensor data to check if a thick line is present.
    Returns:
        action_needed (str or None): "left", "right", or None if no turn should happen.
        should_wipe_memory (bool): True if the car turned and memory should be reset.
    """
    # Check if ALL THREE sensors read dark values (thick horizontal cross-line)
    is_thick_line = (gm_val_list[0] > line_ref and 
                     gm_val_list[1] > line_ref and 
                     gm_val_list[2] > line_ref)
    
    if is_thick_line:
        if current_memory in ["left", "right"]:
            return current_memory, True  # Return the direction to turn and tell main to clear memory
        else:
            if debug_mode:
                print("[WARNING] Thick horizontal line crossed, but no turn sign in memory.")
            # Small delay to prevent spamming prints as the sensors slide across the thick line
            time.sleep(0.4) 
            
    return None, False