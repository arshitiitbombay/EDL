import machine
import time
import sys
import select
from feather_driver import FeatherDriver
from mcp23s17 import MCP23S17

#GRIP,1 for UNgrip
#GRIP,0 for grip
led = machine.Pin("LED", machine.Pin.OUT)
led.value(1)

# ---------------- HARDWARE SETUP ----------------
mcp_reset = machine.Pin(0, machine.Pin.OUT)
mcp_reset.value(1)
time.sleep_ms(10)

ACTIVE_LIMIT_PINS_1 = list(range(4, 16))                
ACTIVE_LIMIT_PINS_2 = list(range(4))  

print("Initializing SPI 0 (MCP1)...")
spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0,
                  sck=machine.Pin(2), mosi=machine.Pin(3), miso=machine.Pin(4))
cs = machine.Pin(5, machine.Pin.OUT)
mcp = MCP23S17(spi, cs)

print("Initializing SPI 1 (MCP2)...")
spi2 = machine.SPI(1, baudrate=1000000, polarity=0, phase=0,
                  sck=machine.Pin(14), mosi=machine.Pin(15), miso=machine.Pin(8))
cs2 = machine.Pin(9, machine.Pin.OUT)
mcp2 = MCP23S17(spi2, cs2)

print("Initializing I2C Busses...")
i2c_slave = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=100000)
print(i2c_slave.scan())
SLAVE_ADDR = 0x50

i2c_motor = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16), freq=100000)
print(i2c_motor.scan())
feathers = FeatherDriver(i2c_motor, addresses=[0x60,0x61,0x62]) 

usb_poll = select.poll()
usb_poll.register(sys.stdin, select.POLLIN)

NUM_ENCODERS = 8 
NUM_MOTORS = 10   

move_in_progress = [False] * NUM_MOTORS
current_directions = [True] * NUM_MOTORS
current_targets = [0] * NUM_MOTORS
move_start_times = [0] * NUM_MOTORS 

# --- NEW: EDGE DETECTION TRACKERS ---
prev_limit_state_1 = 0xFFFF
prev_limit_state_2 = 0xFFFF

print("MASTER READY. Initializing braking sequence...")

for m in range(NUM_MOTORS):
    feathers.state_hold(m)
print("[SYSTEM] All 10 motors locked in HOLD state.")

while True:
    # ==========================================
    # TASK 0: LIMIT SWITCH EDGE DETECTION
    # ==========================================
    try:
        limit_state_1 = mcp.read_all()
        limit_state_2 = mcp2.read_all()
        
        triggered_mcp = -1
        triggered_pin = -1
        
     
        # Check MCP1 for a fresh press (HIGH now, but was LOW previously)
        for pin in ACTIVE_LIMIT_PINS_1:
            if (limit_state_1 & (1 << pin)) != 0 and (prev_limit_state_1 & (1 << pin)) == 0:
                triggered_mcp = 1
                triggered_pin = pin
                break 
                
        # Check MCP2 for a fresh press
        if triggered_mcp == -1:
            for pin in ACTIVE_LIMIT_PINS_2:
                if (limit_state_2 & (1 << pin)) != 0 and (prev_limit_state_2 & (1 << pin)) == 0:
                    triggered_mcp = 2
                    triggered_pin = pin
                    break

        if triggered_mcp != -1:
            any_motor_stopped = False
            feathers.state_hold(8) 
            feathers.state_hold(9) 
            
            for m in range(NUM_ENCODERS):
                if move_in_progress[m]:
                    feathers.state_hold(m)
                    move_in_progress[m] = False
                    any_motor_stopped = True
                    
            if move_in_progress[9]:
                move_in_progress[9] = False
                any_motor_stopped = True

            # ALERT THE PC CONTROLLER!
            print(f"LIMIT,{triggered_mcp},{triggered_pin}")
            
        # Update states for the next loop
        prev_limit_state_1 = limit_state_1
        prev_limit_state_2 = limit_state_2
            
    except Exception as e:
        pass
        
    # ==========================================
    # TASK 1: USB / PC COMMANDS
    # ==========================================
    if usb_poll.poll(0):
        command = sys.stdin.readline().strip().upper()

        if command == "S":
            for m in range(NUM_MOTORS):
                feathers.state_hold(m)
                move_in_progress[m] = False
                current_targets[m] = 0
            print("[SYSTEM] EMERGENCY STOP. All motors braked.")
            
        elif command.startswith("GRIP"):
            parts = command.split(',')
            state = int(parts[1])
            pwm_percent = int(parts[2])
            if state == 1:
                feathers.state_move(8, forward=True, speed=pwm_percent)
            else:
                feathers.state_move(8, forward=False, speed=pwm_percent)
            
        elif command.startswith("START"):
            parts = command.split(',')
            if len(parts) == 5:
                motor_id = int(parts[1])
                direction = int(parts[2])
                angle = int(parts[3]) 
                pwm_percent = int(parts[4])

                if motor_id == 9:
                    if pwm_percent == 0:
                        feathers.state_hold(9)
                        move_in_progress[9] = False
                    else:
                        fwd = bool(direction)
                        feathers.state_move(9, forward=fwd, speed=pwm_percent)
                        move_in_progress[9] = True
                        move_start_times[9] = time.ticks_ms()

                elif 0 <= motor_id < NUM_ENCODERS:
                    if not move_in_progress[motor_id]:
                        fwd = bool(direction)
                        current_directions[motor_id] = fwd
                        
                        target_ticks = int((angle / 360.0) * 1035)
                        brake_target = target_ticks 
                        current_targets[motor_id] = brake_target
                        
                        try: i2c_slave.writeto(SLAVE_ADDR, bytearray([0x01, motor_id]))
                        except: pass

                        reset_confirmed = False
                        for _ in range(50): 
                            try:
                                check_data = i2c_slave.readfrom(SLAVE_ADDR, 16)
                                check_ticks = int.from_bytes(check_data[(motor_id*2) : 2 + (motor_id*2)], 'little')
                                if check_ticks == 0:
                                    reset_confirmed = True
                                    break
                            except: pass
                            time.sleep_ms(1)
                            
                        if not reset_confirmed:
                            continue

                        feathers.state_move(motor_id, forward=fwd, speed=pwm_percent)
                        move_in_progress[motor_id] = True
                        move_start_times[motor_id] = time.ticks_ms() 

    # ==========================================
    # TASK 2: LIVE TELEMETRY & TIMEOUT CHECK
    # ==========================================
    if any(move_in_progress):
        try:
            data = i2c_slave.readfrom(SLAVE_ADDR, 16)

            for m in range(NUM_ENCODERS):
                if move_in_progress[m]:
                    live_ticks = int.from_bytes(data[(m*2) : 2 + (m*2)], 'little')
                    print(live_ticks)
                    if live_ticks >= current_targets[m]:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        current_targets[m] = 0
                        print(f"[SYSTEM] M{m} Reached Target.")
                    
                    elif time.ticks_diff(time.ticks_ms(), move_start_times[m]) > 2000:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        current_targets[m] = 0
                        actual_deg = int((live_ticks / 1035.0) * 360)
                        print(f"TIMEOUT,{m},{actual_deg},{int(current_directions[m])}")

        except Exception as e:
            pass

        if move_in_progress[9]:
            if time.ticks_diff(time.ticks_ms(), move_start_times[9]) > 10000:
                feathers.state_hold(9)
                move_in_progress[9] = False
                print("[SYSTEM] M9 Lead Screw 10s TIMEOUT - Halted.")

    time.sleep_ms(2)
