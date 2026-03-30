import machine
import time
import sys
import select
from feather_driver import FeatherDriver
from mcp23s17 import MCP23S17

# ---------------- HARDWARE SETUP ----------------

mcp_reset = machine.Pin(0, machine.Pin.OUT)
mcp_reset.value(1)
time.sleep_ms(10)

i2c_motor = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16), freq=100000)
feathers = FeatherDriver(i2c_motor, addresses=[0x60]) 

i2c_slave = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10), freq=100000)
SLAVE_ADDR = 0x50

spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0,
                  sck=machine.Pin(2), mosi=machine.Pin(3), miso=machine.Pin(4))
cs = machine.Pin(5, machine.Pin.OUT)
mcp = MCP23S17(spi, cs)

usb_poll = select.poll()
usb_poll.register(sys.stdin, select.POLLIN)

NUM_MOTORS = 4
move_in_progress = [False] * NUM_MOTORS
LIMIT_SWITCH_PIN = 4

print("MASTER READY. Format: START,<motor>,<dir>,<angle> | STOP,<motor>")

while True:
    # ==========================================
    # TASK 1: THE LIMIT SWITCH OVERRIDE
    # ==========================================
    try:
        limit_state = mcp.read_all()
        if (limit_state & (1 << LIMIT_SWITCH_PIN)) == 0: 
            any_motor_stopped = False
            for m in range(NUM_MOTORS):
                if move_in_progress[m]:
                    feathers.state_hold(m)
                    move_in_progress[m] = False
                    any_motor_stopped = True
            
            if any_motor_stopped:
                print("[SAFETY] Limit Switch Triggered! All motors HALTED.")
            continue 
    except Exception as e:
        print(f"[ERROR] Port Expander read failed: {e}")

    # ==========================================
    # TASK 2: KEYBOARD COMMANDS
    # ==========================================
    if usb_poll.poll(0):
        command = sys.stdin.readline().strip().upper()

        # --- DYNAMIC START COMMAND ---
        if command.startswith("START"):
            parts = command.split(',')
            
            # Now expecting 4 parts: START, 0, 1, 90
            if len(parts) == 4:
                motor_id = int(parts[1])
                direction = int(parts[2])
                angle = int(parts[3]) # Capture the target angle

                if 0 <= motor_id < NUM_MOTORS:
                    if not move_in_progress[motor_id]:
                        fwd = bool(direction)
                        
                        # Convert the angle into 2 bytes (allows up to 65,535 degrees)
                        angle_bytes = angle.to_bytes(2, 'little')
                        
                        # Send 4 bytes: [Cmd, Motor, Angle_Low, Angle_High]
                        try:
                            i2c_slave.writeto(SLAVE_ADDR, bytearray([0x01, motor_id, angle_bytes[0], angle_bytes[1]]))
                        except Exception as e:
                            print(f"[ERROR] Failed to send target to Slave: {e}")

                        # Start physical motor
                        feathers.state_move(motor_id, forward=fwd)
                        move_in_progress[motor_id] = True
                        print(f"[SYSTEM] Motor {motor_id} Started. Target: {angle} degrees.")

        # --- STOP COMMAND ---
        elif command.startswith("STOP"):
            parts = command.split(',')
            if len(parts) == 2:
                motor_id = int(parts[1])
                if 0 <= motor_id < NUM_MOTORS and move_in_progress[motor_id]:
                    feathers.state_hold(motor_id)
                    move_in_progress[motor_id] = False
                    print(f"[SYSTEM] Motor {motor_id} stopped by user.")

    # ==========================================
    # TASK 3: THE DYNAMIC AUTO-STOP CHECK
    # ==========================================
    if any(move_in_progress):
        try:
            data = i2c_slave.readfrom(SLAVE_ADDR, 5)
            threshold_flags = data[0] 
            
            if threshold_flags > 0:
                for m in range(NUM_MOTORS):
                    if (threshold_flags & (1 << m)) and move_in_progress[m]:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        print(f"[AUTO-STOP] Motor {m} reached target angle!")

        except Exception as e:
            pass # Suppress temporary I2C blips

    time.sleep_us(200)
