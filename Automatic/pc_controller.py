import cv2
import serial
import serial.tools.list_ports
import threading
import time
import sys
import numpy as np

# ==========================================
# 1. ROBOT CONFIGURATION
# ==========================================
# 4 Links, 2 Motors per link (Pan, Tilt)
# Format: [Pan_Motor_ID, Tilt_Motor_ID]
LINKS = [
    [0, 1], # Link 1 (Tip)
    [2, 3], # Link 2
    [4, 5], # Link 3
    [6, 7]  # Link 4 (Base)
]

MAX_ANGLE = 45.0  # Maximum bend per link in degrees
STEPS_PER_PUSH = 10 
PWM_SPEED = 60

# We keep track of the ABSOLUTE target angles for each link
# link_targets[0] = [pan_angle, tilt_angle] for Link 1
link_targets = [[0.0, 0.0] for _ in range(4)]
current_angles = [[0.0, 0.0] for _ in range(4)]

# Memory for backward retraction later
trajectory_memory = []

# ==========================================
# 2. SERIAL CONNECTION
# ==========================================
def connect_pico():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Pico" in port.description or "Serial" in port.description:
            return serial.Serial(port.device, 115200, timeout=1)
    return None

try:
    ser = connect_pico()
    if not ser:
        print("[ERROR] Pico not found!")
        sys.exit()
    print("[SYSTEM] Connected to Pico.")
except Exception as e:
    print(f"[ERROR] {e}")
    sys.exit()

# ==========================================
# 3. KINEMATIC COMMAND SENDER
# ==========================================
def move_motor_relative(motor_id, delta_angle):
    """Sends the command to move a motor by a specific delta"""
    delta_angle = int(round(delta_angle))
    if delta_angle == 0:
        return # Save I2C traffic if it doesn't need to move

    direction = 1 if delta_angle > 0 else 0
    abs_angle = abs(delta_angle)
    
    cmd = f"START,{motor_id},{direction},{abs_angle},{PWM_SPEED}\n"
    ser.write(cmd.encode('utf-8'))
    print(f"Sent: {cmd.strip()}")


####################################################################
def push_lead_screw_step():
    """
    Moves the 9th motor forward by L/10.
    Currently uses the open-loop GRIP command. 
    TODO: Upgrade this to a START command with an encoder for precision!
    """
    ser.write(b"GRIP,1\n")
    time.sleep(0.2) # Run for 0.2 seconds (Tune this to equal L/10!)
    ser.write(b"GRIP,0\n")
#######################################################################
# ==========================================
# 4. FOLLOW-THE-LEADER EXECUTION THREAD
# ==========================================
def execute_discrete_insertion():
    """Runs in the background so the camera feed doesn't freeze"""
    print("\n[ROBOT] Beginning Discrete FTL Insertion...")
    
    # Store starting angles for interpolation
    start_angles = [[current_angles[i][0], current_angles[i][1]] for i in range(4)]
    
    for step in range(1, STEPS_PER_PUSH + 1):
        print(f"  -> Executing Step {step}/{STEPS_PER_PUSH}")
        
        # 1. Move the Lead Screw forward by L/10
        ################################################################
        # push_lead_screw_step()
        ################################################################
        time.sleep(0.1) # Breather
        
        # 2. Bend all links simultaneously
        for i in range(4): # For each Link
            for m in range(2): # For Pan(0) and Tilt(1)
                
                # Calculate the exact absolute angle we SHOULD be at for this step
                ideal_absolute = start_angles[i][m] + ((link_targets[i][m] - start_angles[i][m]) * (step / STEPS_PER_PUSH))
                
                # How much further do we need to move from where we currently are?
                delta = ideal_absolute - current_angles[i][m]
                
                # We only send integer commands to the Pico
                integer_delta = int(round(delta))
                
                if integer_delta != 0:
                    motor_id = LINKS[i][m]
                    move_motor_relative(motor_id, integer_delta)
                    current_angles[i][m] += integer_delta # Update our tracker

        # Wait for this discrete step to finish bending before pushing again
        # (You can tune this sleep, or write a function to read the [AUTO-STOP] telemetry)
        time.sleep(0.5) 

    print("[ROBOT] Insertion Step Complete. Awaiting next click.")

# ==========================================
# 5. CAMERA & GUI LOGIC
# ==========================================
def mouse_click(event, x, y, flags, param):
    global link_targets, trajectory_memory
    
    if event == cv2.EVENT_LBUTTONDOWN:
        frame_w, frame_h = param
        center_x, center_y = frame_w // 2, frame_h // 2
        
        # Calculate distance from center (Pixels)
        dx = x - center_x
        dy = center_y - y # Invert Y so up is positive
        
        # Scale pixels to Angles (-MAX_ANGLE to +MAX_ANGLE)
        # Using half-width as the maximum possible distance
        pan_target = (dx / (frame_w / 2)) * MAX_ANGLE
        tilt_target = (dy / (frame_h / 2)) * MAX_ANGLE
        
        # Clamp to max constraints
        pan_target = max(min(pan_target, MAX_ANGLE), -MAX_ANGLE)
        tilt_target = max(min(tilt_target, MAX_ANGLE), -MAX_ANGLE)
        
        print(f"\n[GUI] Clicked! Target Angle: Pan={pan_target:.1f}°, Tilt={tilt_target:.1f}°")

        # --- THE SHIFT REGISTER (Follow the Leader) ---
        # Shift old targets down the chain
        link_targets[3] = list(link_targets[2]) # Link 4 takes Link 3's old target
        link_targets[2] = list(link_targets[1]) # Link 3 takes Link 2's old target
        link_targets[1] = list(link_targets[0]) # Link 2 takes Link 1's old target
        
        # Give Link 1 the new target
        link_targets[0] = [pan_target, tilt_target]
        
        # Save to memory for backward retraction
        trajectory_memory.append(list(link_targets))

        # Start the insertion in a background thread
        threading.Thread(target=execute_discrete_insertion, daemon=True).start()

def main():
    cap = cv2.VideoCapture(0) # Open default webcam
    
    # Read one frame to get dimensions
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Cannot access camera.")
        return
    h, w, _ = frame.shape
    
    cv2.namedWindow("Continuum Eye")
    cv2.setMouseCallback("Continuum Eye", mouse_click, param=(w, h))
    
    print("\n--- AUTO CONTROLLER READY ---")
    print("Click on the video feed to steer the robot.")
    print("Press 'SPACE' to Emergency Stop.")
    print("Press 'Q' to Quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # Draw Crosshair
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)
        
        cv2.imshow("Continuum Eye", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            ser.write(b"STOP_ALL\n")
            break
        elif key == ord(' '): # SPACEBAR
            ser.write(b"STOP_ALL\n")
            print("[EMERGENCY STOP SENT]")

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()