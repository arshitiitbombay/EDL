import serial
import serial.tools.list_ports
import threading
import time
import sys
import queue
from pynput import keyboard

# ==========================================
# 1. HARDWARE MAPPING
# ==========================================
# Maps keys to (Motor_ID, Direction)
# Direction: 1 = Forward, 0 = Backward
KEY_MAP = {
    # LINK 1 (Motors 0, 1)
    'w': (0, 1), 's': (0, 0), 'a': (1, 1), 'd': (1, 0),
    # LINK 2 (Motors 2, 3)
    't': (2, 1), 'g': (2, 0), 'f': (3, 1), 'h': (3, 0),
    # LINK 3 (Motors 4, 5)
    'i': (4, 1), 'k': (4, 0), 'j': (5, 1), 'l': (5, 0),
    # LINK 4 (Motors 6, 7)
    keyboard.Key.up: (6, 1), keyboard.Key.down: (6, 0), 
    keyboard.Key.left: (7, 1), keyboard.Key.right: (7, 0)
}

FIXED_ANGLE = 50

# ==========================================
# 2. AUTO-DETECT PICO COM PORT
# ==========================================
def find_pico_port():
    print("Scanning for connected Raspberry Pi Pico...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Pico" in port.description or "Board in FS mode" in port.description or "Serial Device" in port.description:
            return port.device
    if ports:
        return ports[0].device
    return None

# ==========================================
# 3. BACKGROUND TELEMETRY LISTENER
# ==========================================
def listen_to_pico(ser):
    while True:
        try:
            if ser.in_waiting > 0:
                incoming_data = ser.readline().decode('utf-8', errors='ignore').strip()
                if incoming_data:
                    print(f"\n[PICO] {incoming_data}")
        except:
            break
        time.sleep(0.01)

# ==========================================
# 4. MAIN PROGRAM
# ==========================================
def main():
    print("===========================================")
    print("      CONTINUUM TELEOPERATION CONSOLE      ")
    print("===========================================")
    
    port_name = find_pico_port()
    if not port_name:
        print("[PC ERROR] No COM ports found. Plug in the Master Pico!")
        sys.exit()

    try:
        ser = serial.Serial(port_name, baudrate=115200, timeout=1)
        print(f"[PC SYSTEM] Successfully connected to {port_name}.")
        time.sleep(1) 
    except Exception as e:
        print(f"[PC ERROR] Failed to open {port_name}: {e}")
        sys.exit()

    # Start Pico listener
    threading.Thread(target=listen_to_pico, args=(ser,), daemon=True).start()

    # Create a thread-safe queue for keyboard actions
    action_queue = queue.Queue()

    def on_press(key):
        # 1. EMERGENCY STOP (Spacebar)
        if key == keyboard.Key.space:
            action_queue.put(('STOP_ALL', None, None))
            return
            
        # 2. QUIT (Escape)
        if key == keyboard.Key.esc:
            action_queue.put(('QUIT', None, None))
            return

        # 3. KINEMATIC MOVEMENT
        try:
            k = key.char.lower()
        except AttributeError:
            k = key

        if k in KEY_MAP:
            action_queue.put(('MOVE', k, KEY_MAP[k]))

    # Start the global keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print("\n--- CONTROLS ---")
    print(" Link 1 : W A S D")
    print(" Link 2 : T F G H")
    print(" Link 3 : I J K L")
    print(" Link 4 : ARROW KEYS")
    print(" E-Stop : SPACEBAR (Instantly halts all motors)")
    print(" Quit   : ESC")
    print("----------------\n")
    print("Waiting for key press...")

    # The main loop processing your keyboard actions
    while True:
        try:
            # Check the queue for a pressed key
            action = action_queue.get(timeout=0.1)
            cmd = action[0]
            
            if cmd == 'QUIT':
                print("\n[PC SYSTEM] Shutting down controller...")
                ser.write(b"STOP_ALL\n")
                ser.close()
                break
                
            elif cmd == 'STOP_ALL':
                ser.write(b"STOP_ALL\n")
                print("\n[EMERGENCY STOP SENT]")
                
                # Flush out any pending move commands so the robot stays dead
                with action_queue.mutex:
                    action_queue.queue.clear()
                    
            elif cmd == 'MOVE':
                key_pressed, (motor, direction) = action[1], action[2]
                
                # Convert arrow keys to printable names for the UI
                key_name = key_pressed.name.upper() if hasattr(key_pressed, 'name') else str(key_pressed).upper()
                dir_str = "Forward" if direction == 1 else "Reverse"
                
                print(f"\n>> Key [{key_name}] pressed -> Motor {motor} ({dir_str})")
                
                # Clear the queue to ignore keys you accidentally press while typing the PWM
                with action_queue.mutex:
                    action_queue.queue.clear() 

                pwm_str = input(f"   Enter PWM (0-100) or hit Enter to cancel: ")
                
                if pwm_str.isdigit():
                    pwm = int(pwm_str)
                    send_cmd = f"START,{motor},{direction},{FIXED_ANGLE},{pwm}\n"
                    ser.write(send_cmd.encode('utf-8'))
                    print(f"   [SENT] {send_cmd.strip()}")
                else:
                    print("   [CANCELLED] Movement aborted.")
                    
        except queue.Empty:
            pass
            
        except KeyboardInterrupt:
            ser.write(b"STOP_ALL\n")
            ser.close()
            break

if __name__ == '__main__':
    main()