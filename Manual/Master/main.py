import machine
import time
import sys
import select
from feather_driver import FeatherDriver

led = machine.Pin("LED", machine.Pin.OUT)
led.value(1)

# ---------------- HARDWARE SETUP ----------------
"""mcp_reset = machine.Pin(0, machine.Pin.OUT)
mcp_reset.value(1)
time.sleep_ms(10)"""

i2c_slave = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=100000)
device = i2c_slave.scan()
print(device)
SLAVE_ADDR = 0x50

i2c_motor = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16), freq=100000)
device2 = i2c_motor.scan()
feathers = FeatherDriver(i2c_motor, addresses=[0x60,0x61,0x62]) 

print(device2)


usb_poll = select.poll()
usb_poll.register(sys.stdin, select.POLLIN)

NUM_ENCODERS = 8 
NUM_MOTORS = 9   

move_in_progress = [False] * NUM_ENCODERS
current_directions = [True] * NUM_ENCODERS
current_targets = [0] * NUM_ENCODERS
move_start_times = [0] * NUM_ENCODERS 

print("MASTER READY. Initializing braking sequence...")

for m in range(NUM_MOTORS):
    feathers.state_hold(m)
print("[SYSTEM] All 9 motors locked in HOLD state.")

while True:
    # ==========================================
    # TASK 1: KEYBOARD COMMANDS
    # ==========================================
    if usb_poll.poll(0):
        command = sys.stdin.readline().strip().upper()

        if command == "STOP_ALL":
            for m in range(NUM_MOTORS):
                feathers.state_hold(m)
                if m < NUM_ENCODERS:
                    move_in_progress[m] = False
                    current_targets[m] = 0
            print("[SYSTEM] EMERGENCY STOP. All motors braked.")
            
        elif command.startswith("GRIP"):
            parts = command.split(',')
            state = int(parts[1])
            if state == 1:
                feathers.state_move(8, forward=True, speed=100)
            else:
                feathers.state_move(8, forward=False, speed=100)
            
        elif command.startswith("START"):
            parts = command.split(',')
            if len(parts) == 5:
                motor_id = int(parts[1])
                direction = int(parts[2])
                angle = int(parts[3]) 
                pwm_percent = int(parts[4])

                if 0 <= motor_id < NUM_ENCODERS:
                    if not move_in_progress[motor_id]:
                        fwd = bool(direction)
                        current_directions[motor_id] = fwd
                        
                        target_ticks = int((angle / 360.0) * 1035)
                        coast_ticks = int((pwm_percent - 10) * 6.94) if pwm_percent > 10 else 0
                        if target_ticks <= coast_ticks:
                            coast_ticks = target_ticks - 10 
                        
                        brake_target = target_ticks - coast_ticks
                        current_targets[motor_id] = brake_target
                        
                        # 1. SEND RESET COMMAND
                        try:
                            i2c_slave.writeto(SLAVE_ADDR, bytearray([0x01, motor_id]))
                        except:
                            pass

                        # 2. THE CONFIRMATION HANDSHAKE
                        reset_confirmed = False
                        for _ in range(50): # Wait up to 50ms
                            try:
                                check_data = i2c_slave.readfrom(SLAVE_ADDR, 16) #changed 17 to 16 here
                                check_ticks = int.from_bytes(check_data[(motor_id*2) : 2 + (motor_id*2)], 'little')
                                
                                # Check if Slave successfully zeroed out the encoder
                                if check_ticks == 0:
                                    reset_confirmed = True
                                    break
                            except:
                                pass
                            time.sleep_ms(1)
                            
                        if not reset_confirmed:
                            print(f"[ERROR] Slave failed to reset M{motor_id}. Aborting start.")
                            continue

                        # 3. FIRE MOTOR
                        feathers.state_move(motor_id, forward=fwd, speed=pwm_percent)
                        move_in_progress[motor_id] = True
                        move_start_times[motor_id] = time.ticks_ms() 
                        print(f"[SYSTEM] M{motor_id} Handshake Complete. Moving to {angle} deg.")

    # ==========================================
    # TASK 2: LIVE TELEMETRY & TIMEOUT CHECK
    # ==========================================
    if any(move_in_progress):
        try:
            data = i2c_slave.readfrom(SLAVE_ADDR, 16) #changed 17 to 16 here

            for m in range(NUM_ENCODERS):
                if move_in_progress[m]:
                    live_ticks = int.from_bytes(data[(m*2) : 2 + (m*2)], 'little')
                    
                    if live_ticks >= current_targets[m]:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        current_targets[m] = 0
                    
                    elif time.ticks_diff(time.ticks_ms(), move_start_times[m]) > 2000:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        current_targets[m] = 0
                        actual_deg = int((live_ticks / 1035.0) * 360)
                        print(f"TIMEOUT,{m},{actual_deg},{int(current_directions[m])}")

        except Exception as e:
            pass

    time.sleep_ms(2)
