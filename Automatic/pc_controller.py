import cv2
import serial
import serial.tools.list_ports
import threading
import time
import sys
import copy
import numpy as np

# ==========================================
# 1. ROBOT HARDWARE MAPPING & TUNING
# ==========================================
MAX_ANGLE = 250.0            # Absolute maximum clamp for any link
STEPS_PER_PUSH = 12
PWM_SPEED = 100
STEERING_TIME = 2.0          # Time to bend when steering (no lead screw)

# Multipliers to compensate for mechanical friction/weight during INSERTION
# Format: [[Pan, Tilt], [Pan, Tilt], [Pan, Tilt], [Pan, Tilt]]
LINK_MULTIPLIERS = [
    [0.75, 0.75],  # LINK 1 (Tip)
    [1.0, 1.0],  # LINK 2
    [1.1, 1.1],  # LINK 3
    [1.1, 1.1]   # LINK 4 (Base)
]

# Multipliers to compensate for tendon slack/hysteresis during RETRACTION
# Format: [[Pan, Tilt], [Pan, Tilt], [Pan, Tilt], [Pan, Tilt]]
LINK_RETRACTION_MULTIPLIERS = [
    [0.5, 0.5],    # LINK 1
    [0.5, 0.5],  # LINK 2
    [0.5, 0.5],  # LINK 3
    [0.5, 0.5]   # LINK 4
]

# Specific time required to push each link out
# [Link 1 (Tip), Link 2, Link 3, Link 4 (Base)]
INSERTION_TIMES = [3.75, 5.3, 6.0, 3.0]

# Format: { 0 (Pan): (Motor_ID, Pos_Dir, Neg_Dir), 1 (Tilt): (Motor_ID, Pos_Dir, Neg_Dir) }
LINK_CONFIG = [
    { 0: (1, 0, 1), 1: (0, 1, 0) }, # LINK 1 (TIP)
    { 0: (3, 0, 1), 1: (2, 0, 1) }, # LINK 2
    { 0: (5, 1, 0), 1: (4, 1, 0) }, # LINK 3
    { 0: (7, 0, 1), 1: (6, 0, 1) }  # LINK 4 (BASE)
]

# State Trackers
link_targets   = [[0.0, 0.0] for _ in range(4)]
current_angles = [[0.0, 0.0] for _ in range(4)]

# Memory now starts empty. It will store Actions instead of States.
trajectory_memory = [] 

tentacle_closed = False 
is_moving = False 

# --- SYSTEM LOCK FLAGS ---
limit_active = False 
HARD_FAULT_LOCKED = False 

# ==========================================
# 2. SERIAL CONNECTION
# ==========================================
def connect_pico():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Pico" in port.description or "Serial" in port.description:
            return serial.Serial(port.device, 115200, timeout=0.1)
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

# --- NEW: Master Serial Transmission Wrapper ---
def tx_command(cmd_str):
    """Prints the command to the console, then sends it to the Pico."""
    print(f"[SERIAL TX] {cmd_str.strip()}")
    ser.write(cmd_str.encode('utf-8'))

def send_motor_command(motor_id, direction, abs_angle):
    if abs_angle == 0: return 
    cmd = f"START,{motor_id},{direction},{abs_angle},{PWM_SPEED}\n"
    tx_command(cmd)

# ==========================================
# 3. BACKGROUND SERIAL READER & LIMIT LOGIC
# ==========================================
def serial_reader_thread():
    """Constantly listens to the Pico in the background."""
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("LIMIT"):
                    _, mcp_id, pin = line.split(',')
                    threading.Thread(target=handle_limit_switch, args=(int(mcp_id), int(pin)), daemon=True).start()
                elif line != "":
                    print(f"[PICO] {line}")
            except: pass
        time.sleep(0.01)

threading.Thread(target=serial_reader_thread, daemon=True).start()

def handle_limit_switch(mcp_id, pin):
    """Case-wise logic for what to do when a limit switch is hit."""
    global limit_active, current_angles
    limit_active = True 
    
    print(f"\n======================================")
    print(f"!!! LIMIT SWITCH TRIGGERED: MCP{mcp_id} PIN {pin} !!!")
    print(f"======================================")
    
    # CASE 1: Pin 14 Hit Example
    if mcp_id == 1 and pin == 14:
        print("[LIMIT LOGIC] Escaping Pin 14. Backing off M4 and M2 by 50 deg...")
        send_motor_command(4, 0, 50)
        send_motor_command(2, 0, 50)
        current_angles[2][1] -= 50.0  # Update M4 (Link 3 Tilt)
        current_angles[1][1] -= 50.0  # Update M2 (Link 2 Tilt)
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")
    
    # CASE 2: Pin 7 Hit Example
    elif mcp_id == 1 and pin == 7: 
        print("[LIMIT LOGIC] Escaping Pin 7. Backing off M1 by 50 deg...")
        send_motor_command(1, 1, 50)
        current_angles[0][0] -= 50.0  # Update M1 (Link 1 Pan)
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    limit_active = False 

# ==========================================
# 4. EXECUTION THREADS (THE NEW LOGIC)
# ==========================================
def execute_discrete_step(ls_level):
    global is_moving, HARD_FAULT_LOCKED
    is_moving = True
    
    current_memory = trajectory_memory[-1] # Grab the active memory node
    
    if ls_level > 0:
        total_time = INSERTION_TIMES[ls_level - 1]
        print(f"\n[ROBOT] Inserting Link {ls_level} (Duration: {total_time}s)...")
        tx_command("START,9,1,0,100\n") 
    else:
        total_time = STEERING_TIME
        print(f"\n[ROBOT] Steering outside box. Links shifting. (Duration: {total_time}s)...")
        
    step_delay = total_time / STEPS_PER_PUSH
    start_angles = copy.deepcopy(current_angles)
    
    for step in range(1, STEPS_PER_PUSH + 1):
        while limit_active:
            time.sleep(0.1)
            
        if HARD_FAULT_LOCKED:
            break

        for i in range(4): 
            for m in range(2): 
                scaled_target = link_targets[i][m] * LINK_MULTIPLIERS[i][m]
                ideal_absolute = start_angles[i][m] + ((scaled_target - start_angles[i][m]) * (step / STEPS_PER_PUSH))
                delta = ideal_absolute - current_angles[i][m]
                integer_delta = int(round(delta))
                
                # --- MAX ANGLE ABSOLUTE FAULT CHECK ---
                if abs(current_angles[i][m] + integer_delta) >= MAX_ANGLE:
                    print(f"\n[CRITICAL FAULT] Link {i+1} attempted to exceed {MAX_ANGLE} deg!")
                    tx_command("S\n") 
                    HARD_FAULT_LOCKED = True
                    is_moving = False
                    return 

                if integer_delta != 0:
                    motor_id, pos_dir, neg_dir = LINK_CONFIG[i][m]
                    direction = pos_dir if integer_delta > 0 else neg_dir
                    
                    send_motor_command(motor_id, direction, abs(integer_delta))
                    
                    # Update Tracker AND record the exact action taken in memory!
                    current_angles[i][m] += integer_delta
                    current_memory["motor_deltas"][i][m] += integer_delta

        time.sleep(step_delay) 

    if ls_level > 0 and not HARD_FAULT_LOCKED:
        tx_command("START,9,1,0,0\n") 
        
    if not HARD_FAULT_LOCKED:
        print("[ROBOT] Step Complete. Awaiting next command.")
    is_moving = False


def execute_discrete_retraction():
    global link_targets, trajectory_memory, is_moving, HARD_FAULT_LOCKED
    
    if len(trajectory_memory) == 0:
        print("[ROBOT] At home position! Nothing to retract.")
        return

    is_moving = True
    HARD_FAULT_LOCKED = False 
    
    last_move = trajectory_memory.pop() 
    ls_level_to_reverse = last_move["ls_level"]
    
    # 1. Restore the FTL link targets so next forward push is mathematically correct
    link_targets = copy.deepcopy(last_move["old_targets"])

    # 2. Restore current_angles instantly. We assume the retraction will be successful.
    for i in range(4):
        for m in range(2):
            current_angles[i][m] -= last_move["motor_deltas"][i][m]

    if ls_level_to_reverse > 0:
        total_time = INSERTION_TIMES[ls_level_to_reverse - 1]
        print(f"\n[ROBOT] Retracting Link {ls_level_to_reverse} via Action Undo (Duration: {total_time}s)...")
        tx_command("START,9,0,0,100\n") 
    else:
        total_time = STEERING_TIME
        print(f"\n[ROBOT] Retracting Steering Move via Action Undo (Duration: {total_time}s)...")

    step_delay = total_time / STEPS_PER_PUSH
    undone_so_far = [[0, 0] for _ in range(4)]
    
    # 3. Physically Undo the motor actions step-by-step
    for step in range(1, STEPS_PER_PUSH + 1):
        while limit_active:
            time.sleep(0.1)
            
        for i in range(4): 
            for m in range(2): 
                forward_delta = last_move["motor_deltas"][i][m]
                if forward_delta == 0: continue
                
                # Calculate total reversal needed (inverted sign, scaled by retraction multiplier)
                total_reversal = -forward_delta * LINK_RETRACTION_MULTIPLIERS[i][m]
                ideal_reversal_by_now = total_reversal * (step / STEPS_PER_PUSH)
                
                step_delta = ideal_reversal_by_now - undone_so_far[i][m]
                integer_step_delta = int(round(step_delta))
                
                if integer_step_delta != 0:
                    motor_id, pos_dir, neg_dir = LINK_CONFIG[i][m]
                    direction = pos_dir if integer_step_delta > 0 else neg_dir
                    
                    send_motor_command(motor_id, direction, abs(integer_step_delta))
                    undone_so_far[i][m] += integer_step_delta

        time.sleep(step_delay) 

    if ls_level_to_reverse > 0:
        tx_command("START,9,0,0,0\n") 
        
    print("[ROBOT] Retraction Step Complete.")
    is_moving = False

def pure_retract_thread():
    global is_moving
    is_moving = True
    print("\n[SYSTEM] Pure Lead Screw Retraction (2 seconds)...")
    tx_command("START,9,0,0,100\n")
    time.sleep(2.0)
    tx_command("START,9,0,0,0\n") 
    print("[SYSTEM] Pure Retraction Complete.")
    is_moving = False

# ==========================================
# 5. CAMERA & GUI LOGIC
# ==========================================
def push_trajectory_and_execute(delta_pan, delta_tilt):
    global link_targets, trajectory_memory
    
    new_pan_target = link_targets[0][0] + delta_pan
    new_tilt_target = link_targets[0][1] + delta_tilt
    
    pan_target = max(min(new_pan_target, MAX_ANGLE), -MAX_ANGLE)
    tilt_target = max(min(new_tilt_target, MAX_ANGLE), -MAX_ANGLE)
    
    print(f"\n[TARGET] Click Added. New Absolute Target: Pan={pan_target:.1f}°, Tilt={tilt_target:.1f}°")

    # Save the CURRENT targets before we shift them, so we can restore them on retract
    old_targets = copy.deepcopy(link_targets)

    # Shift old targets down the chain
    link_targets[3] = list(link_targets[2]) 
    link_targets[2] = list(link_targets[1]) 
    link_targets[1] = list(link_targets[0]) 
    link_targets[0] = [pan_target, tilt_target]
    
    current_step = len(trajectory_memory) 
    ls_level = current_step + 1 if current_step < 4 else 0
    
    # Generate the Memory Node
    trajectory_memory.append({
        "old_targets": old_targets,
        "ls_level": ls_level,
        "motor_deltas": [[0, 0] for _ in range(4)] # Will be filled during execution
    })

    threading.Thread(target=execute_discrete_step, args=(ls_level,), daemon=True).start()

def mouse_click(event, x, y, flags, param):
    global is_moving
    if event == cv2.EVENT_LBUTTONDOWN:
        if is_moving:
            print("[GUI] Ignoring click: Robot is currently moving!")
            return

        frame_w, frame_h = param
        center_x, center_y = frame_w // 2, frame_h // 2
        
        dx = x - center_x
        dy = center_y - y 
        
        delta_pan = (dx / (frame_w / 2)) * MAX_ANGLE
        delta_tilt = (dy / (frame_h / 2)) * MAX_ANGLE
        push_trajectory_and_execute(delta_pan, delta_tilt)

def main():
    global tentacle_closed, is_moving
    
    cap = cv2.VideoCapture(1) 
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Cannot access camera.")
        return
    h, w, _ = frame.shape
    
    cv2.namedWindow("Continuum Eye", cv2.WINDOW_NORMAL)
    is_fullscreen = False
    cv2.setMouseCallback("Continuum Eye", mouse_click, param=(w, h))
    
    print("\n--- AUTO CONTROLLER READY ---")
    print("Click Feed: Insert/Steer FTL Trajectory")
    print("Press 'R': Revert one step via FTL Memory")
    print("Press 'T': Toggle the Tentacle (Gripper)")
    print("Press 'B': PURE Retract (Lead Screw only, no memory)")
    print("Press 'F': Toggle Fullscreen")
    print("Press 'SPACE': Emergency Stop")
    print("Press 'Q': Quit")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)
        
        current_step = len(trajectory_memory)
        seq_text = f"Inserted Links: {min(current_step, 4)}/4 | Steer Clicks: {max(0, current_step - 4)}"
        cv2.putText(frame, seq_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        if HARD_FAULT_LOCKED:
            cv2.putText(frame, "MAX ANGLE REACHED! LOCKED.", (w//2 - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        elif limit_active:
            cv2.putText(frame, "LIMIT SWITCH ESCAPE ACTIVE", (w//2 - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        elif is_moving:
            cv2.putText(frame, "MOVING... PLEASE WAIT", (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        cv2.imshow("Continuum Eye", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            tx_command("S\n")
            break
        elif key == ord(' '): 
            tx_command("S\n")
            print("[EMERGENCY STOP SENT]")
        elif key == ord('t'):
            if not is_moving:
                tentacle_closed = not tentacle_closed
                state_val = 1 if tentacle_closed else 0
                tx_command(f"GRIP,{state_val}\n")
        elif key == ord('b'):
            if not is_moving:
                threading.Thread(target=pure_retract_thread, daemon=True).start()
        elif key == ord('r'):
            if not is_moving:
                threading.Thread(target=execute_discrete_retraction, daemon=True).start()
        elif key == ord('f'): 
            is_fullscreen = not is_fullscreen
            if is_fullscreen:
                cv2.setWindowProperty("Continuum Eye", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty("Continuum Eye", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
