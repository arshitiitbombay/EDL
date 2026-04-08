import cv2
import serial
import serial.tools.list_ports
import threading
import time
import sys
import numpy as np

# ==========================================
# 1. ROBOT HARDWARE MAPPING
# ==========================================
MAX_ANGLE = 500.0  # Maximum bend per link in degrees
STEPS_PER_PUSH = 10 
PWM_SPEED = 60

# We map your specific motor directions here!
# Format: { 0 (Pan Axis): (Motor_ID, Pos_Dir, Neg_Dir), 1 (Tilt Axis): (Motor_ID, Pos_Dir, Neg_Dir) }
# Positive Pan = Right | Negative Pan = Left
# Positive Tilt = Up   | Negative Tilt = Down
LINK_CONFIG = [
    # LINK 1 (TIP) - Pan (Right/Left: M1), Tilt (Up/Down: M0)
    { 0: (1, 0, 1), 1: (0, 1, 0) }, 
    
    # LINK 2 - Pan (Right/Left: M3), Tilt (Up/Down: M2)
    { 0: (3, 0, 1), 1: (2, 0, 1) }, 
    
    # LINK 3 - Pan (Right/Left: M5), Tilt (Up/Down: M4)
    { 0: (5, 1, 0), 1: (4, 1, 0) }, 
    
    # LINK 4 (BASE) - Pan (Right/Left: M7), Tilt (Up/Down: M6)
    { 0: (7, 0, 1), 1: (6, 0, 1) }  
]

# State Trackers: [pan_angle, tilt_angle] for each of the 4 links
link_targets   = [[0.0, 0.0] for _ in range(4)]
current_angles = [[0.0, 0.0] for _ in range(4)]

trajectory_memory = []
tentacle_closed = False # Track the state of Motor 8

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
def send_motor_command(motor_id, direction, abs_angle):
    """Sends the raw command to the Pico"""
    if abs_angle == 0: return 
    cmd = f"START,{motor_id},{direction},{abs_angle},{PWM_SPEED}\n"
    ser.write(cmd.encode('utf-8'))

# ==========================================
# 4. FOLLOW-THE-LEADER EXECUTION THREAD
# ==========================================
def execute_discrete_insertion():
    """Runs in the background, slicing the movement into 10 steps"""
    print("\n[ROBOT] Beginning Discrete FTL Insertion...")
    
    # 1. FIRE THE LEAD SCREW (Motor 9)
    # The Pico will automatically run this for exactly 2 seconds!
    ls_cmd = f"START,9,1,0,{PWM_SPEED}\n"
    ser.write(ls_cmd.encode('utf-8'))
    print(" -> Lead Screw Engaged (2-second automatic timer)")
    
    # Snapshot of where we are right now before we start stepping
    start_angles = [[current_angles[i][m] for m in range(2)] for i in range(4)]
    
    # 2. DISCRETIZE BENDING INTO 10 STEPS
    for step in range(1, STEPS_PER_PUSH + 1):
        for i in range(4): # Loop through Links 0 to 3
            for m in range(2): # m=0 is Pan (X-axis), m=1 is Tilt (Y-axis)
                
                # Where exactly should we be at this specific step?
                ideal_absolute = start_angles[i][m] + ((link_targets[i][m] - start_angles[i][m]) * (step / STEPS_PER_PUSH))
                
                # How much further do we need to move to get there?
                delta = ideal_absolute - current_angles[i][m]
                integer_delta = int(round(delta))
                
                if integer_delta != 0:
                    # Look up the specific motor and direction in our Hardware Map
                    motor_id, pos_dir, neg_dir = LINK_CONFIG[i][m]
                    
                    # Choose the direction based on whether we are moving Positive or Negative
                    direction = pos_dir if integer_delta > 0 else neg_dir
                    
                    # Send the command and update our local tracker
                    send_motor_command(motor_id, direction, abs(integer_delta))
                    current_angles[i][m] += integer_delta

        # Wait for 0.2 seconds so the 10 bending steps take exactly 2 seconds total,
        # perfectly matching the linear push of the lead screw!
        time.sleep(0.2) 

    print("[ROBOT] Insertion Step Complete. Awaiting next click.")

def execute_discrete_retraction():
    """Runs in the background, pulling the robot back one step along its memory"""
    global link_targets, trajectory_memory
    
    # 1. Check if we actually have steps to remember!
    if len(trajectory_memory) <= 1:
        print("[ROBOT] At home position! Nothing to retract.")
        return

    print("\n[ROBOT] Beginning Discrete FTL Retraction...")
    
    # 2. POP the current state, and load the PREVIOUS state into targets
    trajectory_memory.pop() # Discard current
    previous_state = trajectory_memory[-1] # Look at the previous
    
    # Deep copy the memory back into active targets
    link_targets = [[previous_state[i][m] for m in range(2)] for i in range(4)]
    
    # 3. FIRE THE LEAD SCREW BACKWARDS (Direction 0)
    ls_cmd = f"START,9,0,0,{PWM_SPEED}\n"
    ser.write(ls_cmd.encode('utf-8'))
    print(" -> Lead Screw Retracting (2-second automatic timer)")
    
    start_angles = [[current_angles[i][m] for m in range(2)] for i in range(4)]
    
    # 4. DISCRETIZE BENDING INTO 10 STEPS
    for step in range(1, STEPS_PER_PUSH + 1):
        for i in range(4): 
            for m in range(2): 
                
                ideal_absolute = start_angles[i][m] + ((link_targets[i][m] - start_angles[i][m]) * (step / STEPS_PER_PUSH))
                delta = ideal_absolute - current_angles[i][m]
                integer_delta = int(round(delta))
                
                if integer_delta != 0:
                    motor_id, pos_dir, neg_dir = LINK_CONFIG[i][m]
                    direction = pos_dir if integer_delta > 0 else neg_dir
                    
                    send_motor_command(motor_id, direction, abs(integer_delta))
                    current_angles[i][m] += integer_delta

        # Wait for 0.2s so the bending matches the 2-second lead screw pull
        time.sleep(0.2) 

    print("[ROBOT] Retraction Step Complete. Ready.")   
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
        pan_target = (dx / (frame_w / 2)) * MAX_ANGLE
        tilt_target = (dy / (frame_h / 2)) * MAX_ANGLE
        
        # Clamp to max physical constraints
        pan_target = max(min(pan_target, MAX_ANGLE), -MAX_ANGLE)
        tilt_target = max(min(tilt_target, MAX_ANGLE), -MAX_ANGLE)
        
        print(f"\n[GUI] Clicked! Target Angle: Pan={pan_target:.1f}°, Tilt={tilt_target:.1f}°")

        # Shift old targets down the chain (Follow-the-leader behavior)
        link_targets[3] = list(link_targets[2]) 
        link_targets[2] = list(link_targets[1]) 
        link_targets[1] = list(link_targets[0]) 
        link_targets[0] = [pan_target, tilt_target]
        
        trajectory_memory.append(list(link_targets))

        # Start the discrete execution in a background thread
        threading.Thread(target=execute_discrete_insertion, daemon=True).start()

def main():
    global tentacle_closed
    cap = cv2.VideoCapture(1) # Index 1 for the Robot USB Camera
    
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Cannot access camera. Try changing VideoCapture(1) to 0 or 2.")
        return
    h, w, _ = frame.shape
    
    cv2.namedWindow("Continuum Eye")
    cv2.setMouseCallback("Continuum Eye", mouse_click, param=(w, h))
    
    print("\n--- AUTO CONTROLLER READY ---")
    print("Click on the video feed to steer the robot.")
    print("Press 'T' to toggle the Tentacle (Gripper).")
    print("Press 'R' to RETRACT the Lead Screw (Pull back).")
    print("Press 'SPACE' to Emergency Stop.")
    print("Press 'Q' to Quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # Draw Targeting Crosshair
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)
        
        # Display Tentacle Status on screen
        status_text = "Tentacle: CLOSED" if tentacle_closed else "Tentacle: OPEN"
        color = (0, 0, 255) if tentacle_closed else (0, 255, 0)
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow("Continuum Eye", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            ser.write(b"S\n")
            break
        elif key == ord(' '): 
            ser.write(b"S\n")
            print("[EMERGENCY STOP SENT]")
        elif key == ord('t'):
            # Toggle Motor 8 (Tentacle)
            tentacle_closed = not tentacle_closed
            state_val = 1 if tentacle_closed else 0
            ser.write(f"GRIP,{state_val}\n".encode('utf-8'))
            print(f"[SYSTEM] Tentacle {'Closed' if tentacle_closed else 'Opened'}")
        elif key == ord('r'):
            # Manual Retract: Motor 9, Direction 0 (Backward)
            cmd = f"START,9,0,0,{PWM_SPEED}\n"
            ser.write(cmd.encode('utf-8'))
            print("[SYSTEM] Retracting Lead Screw for 2 seconds...")

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
