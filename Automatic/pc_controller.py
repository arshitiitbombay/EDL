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
LEAD_SCREW_TIMEOUT = 1.0    # Matches the Master Pico 10s timeout

# Multipliers to compensate for mechanical friction/weight during INSERTION
# Format: [[Pan, Tilt], [Pan, Tilt], [Pan, Tilt], [Pan, Tilt]]
LINK_MULTIPLIERS = [
    [0.75, 0.75],  # LINK 1 (Tip)
    [1.0, 0.8],    # LINK 2
    [1.0, 0.9],    # LINK 3
    [0, 0]     # LINK 4 (Base)
]

# Multipliers to compensate for tendon slack/hysteresis during RETRACTION
# Format: [[Pan, Tilt], [Pan, Tilt], [Pan, Tilt], [Pan, Tilt]]
LINK_RETRACTION_MULTIPLIERS = [
    [0.6, 0.45],    # LINK 1
    [0.75, 0.1],  # LINK 2
    [0.7, 0.07],  # LINK 3
    [0, 0]   # LINK 4
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

# Memory Stack (Stores independent Actions)
trajectory_memory = [] 

tentacle_pwm = 0 # Starts ungripped at 0
is_moving = False 

# --- SYSTEM LOCK FLAGS ---
limit_active = False 
HARD_FAULT_LOCKED = False 

# ==========================================
# 2. SERIAL CONNECTION & TRANSMISSION
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

def tx_command(cmd_str):
    """The master funnel: Prints the command to the console, then sends it to the Pico."""
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
    global limit_active, current_angles
    limit_active = True 
    
    print(f"\n======================================")
    print(f"!!! LIMIT SWITCH TRIGGERED: MCP{mcp_id} PIN {pin} !!!")
    print(f"======================================")
    
    
    if mcp_id == 2 and pin == 3: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(0, 0, 50)
        current_angles[0][1] -= 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    elif mcp_id == 1 and pin == 9: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(0, 1, 50)
        current_angles[0][1] += 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    elif mcp_id == 2 and pin == 15: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(1, 0, 50)
        current_angles[1][0] += 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    elif mcp_id == 2 and pin == 5: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(1, 1, 50)
        current_angles[1][0] -= 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    elif mcp_id == 2 and pin == 13: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(2, 1, 50)
        current_angles[2][1] -= 50.0  
        time.sleep(1.5)
        send_motor_command(0, 1, 50)
        current_angles[0][1] += 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    elif mcp_id == 2 and pin == 12: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(2, 0, 50)
        current_angles[2][1] += 50.0  
        time.sleep(1.5)
        send_motor_command(0, 0, 50)
        current_angles[0][0] -= 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")
    
    elif mcp_id == 2 and pin == 2: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(3, 1, 50)
        current_angles[3][0] -= 50.0  
        time.sleep(1.5)
        send_motor_command(1, 0, 50)
        current_angles[1][0] += 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")
    
    elif mcp_id == 2 and pin == 0: 
        print(f"[LIMIT LOGIC] Escaping Pin {pin}, MCP {mcp_id}.")
        send_motor_command(3, 0, 50)
        current_angles[3][0] += 50.0  
        time.sleep(1.5)
        send_motor_command(1, 1, 50)
        current_angles[1][1] -= 50.0  
        time.sleep(1.5)
        print("[LIMIT LOGIC] Escape complete. Resuming previous operation.")

    limit_active = False 

# ==========================================
# 4. EXECUTION THREADS (STACK LOGIC)
# ==========================================
def execute_lead_screw(forward=True):
    """Fires the lead screw and lets the Master time it out independently."""
    global is_moving
    is_moving = True
    
    direction_val = 1 if forward else 0
    print(f"\n[ROBOT] Moving Lead Screw (Forward: {forward}). Waiting for Pico Timeout...")
    
    # We do NOT send a stop command. The Master's 10s timeout handles it.
    tx_command(f"START,9,{direction_val},0,100\n") 
    time.sleep(LEAD_SCREW_TIMEOUT) 
    
    print("[ROBOT] Lead Screw Step Complete.")
    is_moving = False

def execute_link_step():
    """Bends the continuum links without touching the lead screw."""
    global is_moving, HARD_FAULT_LOCKED
    is_moving = True
    
    current_memory = trajectory_memory[-1] 
    print(f"\n[ROBOT] Steering links. (Duration: {STEERING_TIME}s)...")
        
    step_delay = STEERING_TIME / STEPS_PER_PUSH
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
                    current_angles[i][m] += integer_delta
                    current_memory["motor_deltas"][i][m] += integer_delta

        time.sleep(step_delay) 
        
    if not HARD_FAULT_LOCKED:
        print("[ROBOT] Step Complete. Awaiting next command.")
    is_moving = False

def execute_discrete_retraction():
    global link_targets, trajectory_memory, is_moving, HARD_FAULT_LOCKED
    
    if len(trajectory_memory) == 0:
        print("[ROBOT] Stack Empty! At home position.")
        return

    is_moving = True
    HARD_FAULT_LOCKED = False 
    last_move = trajectory_memory.pop() 
    
    # ---------------------------------------------------------
    # UNDO A LEAD SCREW MOVE
    # ---------------------------------------------------------
    if last_move["type"] == "LEAD_SCREW":
        was_forward = last_move["forward"]
        reverse_dir = 0 if was_forward else 1
        
        print(f"\n[ROBOT] Retracting Lead Screw via Stack Memory...")
        tx_command(f"START,9,{reverse_dir},0,100\n") 
        time.sleep(LEAD_SCREW_TIMEOUT)
        print("[ROBOT] Retraction Step Complete.")
        is_moving = False
        return

    # ---------------------------------------------------------
    # UNDO A LINK STEERING MOVE
    # ---------------------------------------------------------
    link_targets = copy.deepcopy(last_move["old_targets"])
    for i in range(4):
        for m in range(2):
            current_angles[i][m] -= last_move["motor_deltas"][i][m]

    print(f"\n[ROBOT] Retracting Steering Move via Stack Memory (Duration: {STEERING_TIME}s)...")
    step_delay = STEERING_TIME / STEPS_PER_PUSH
    undone_so_far = [[0, 0] for _ in range(4)]
    
    for step in range(1, STEPS_PER_PUSH + 1):
        while limit_active:
            time.sleep(0.1)
            
        for i in range(4): 
            for m in range(2): 
                forward_delta = last_move["motor_deltas"][i][m]
                if forward_delta == 0: continue
                
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

    old_targets = copy.deepcopy(link_targets)

    link_targets[3] = list(link_targets[2]) 
    link_targets[2] = list(link_targets[1]) 
    link_targets[1] = list(link_targets[0]) 
    link_targets[0] = [pan_target, tilt_target]
    
    trajectory_memory.append({
        "type": "LINK",
        "old_targets": old_targets,
        "motor_deltas": [[0, 0] for _ in range(4)]
    })

    threading.Thread(target=execute_link_step, daemon=True).start()

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
    global tentacle_pwm, is_moving
    
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
    print("Click Feed: Steer FTL Trajectory")
    print("W / S Keys: Push / Pull Lead Screw")
    print("U / D Keys: Increase / Decrease Gripper PWM")
    print("Press 'R': Revert one step via FTL Memory Stack")
    print("Press 'F': Toggle Fullscreen")
    print("Press 'SPACE': Emergency Stop")
    print("Press 'Q': Quit")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)
        
        seq_text = f"Stack Depth: {len(trajectory_memory)} | Gripper PWM: {tentacle_pwm}%"
        cv2.putText(frame, seq_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        if HARD_FAULT_LOCKED:
            cv2.putText(frame, "MAX ANGLE REACHED! LOCKED.", (w//2 - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        elif limit_active:
            cv2.putText(frame, "LIMIT SWITCH ESCAPE ACTIVE", (w//2 - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        elif is_moving:
            cv2.putText(frame, "MOVING... PLEASE WAIT", (w - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        cv2.imshow("Continuum Eye", frame)
        
        key = cv2.waitKeyEx(1) 
        
        if key == -1: 
            continue
        elif key == ord('q'):
            tx_command("S\n")
            break
        elif key == ord(' '): 
            tx_command("S\n")
            print("[EMERGENCY STOP SENT]")
            
        # --- INDEPENDENT LEAD SCREW CONTROLS ---
        elif key == ord('w') and not is_moving:
            trajectory_memory.append({"type": "LEAD_SCREW", "forward": True})
            threading.Thread(target=execute_lead_screw, args=(True,), daemon=True).start()
        elif key == ord('s') and not is_moving:
            trajectory_memory.append({"type": "LEAD_SCREW", "forward": False})
            threading.Thread(target=execute_lead_screw, args=(False,), daemon=True).start()
            
        # --- TENTACLE GRIPPER CONTROLS ---
        # Fixed: Using ord('u') and ord('d') to properly check the integer keycode
        elif (key == ord('u') or key == ord('U')) and not is_moving: 
            tentacle_pwm += 10
            if tentacle_pwm > 100: tentacle_pwm = 100
            if tentacle_pwm > 0:
                tx_command(f"GRIP,0,{tentacle_pwm}\n")
                print(f"[GRIPPER] Engaging. Tension: {tentacle_pwm}%")
                
        elif (key == ord('d') or key == ord('D')) and not is_moving: 
            tentacle_pwm -= 10
            if tentacle_pwm <= 0:
                tx_command(f"GRIP,1,min({abs(tentacle_pwm)},100)\n") 
                print("[GRIPPER] UNGRIPPED. Releasing Tension.")
            else:
                tx_command(f"GRIP,0,{tentacle_pwm}\n")
                print(f"[GRIPPER] Releasing. Tension: {tentacle_pwm}%")

        # --- UTILITY CONTROLS ---
        elif key == ord('r') and not is_moving:
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
