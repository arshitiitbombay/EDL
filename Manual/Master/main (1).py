import machine
import time
import sys
import select
from feather_driver import FeatherDriver
from mcp23s17 import MCP23S17 # Assuming you saved the corrected class here

mcp_reset = machine.Pin(0, machine.Pin.OUT)
mcp_reset.value(1)
time.sleep_ms(10)

# ---------------- HARDWARE SETUP ----------------
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
LIMIT_SWITCH_PIN = 4 # Example: Bit 10 (GPB2) from our previous discussion

print("MASTER READY. ALGORITHM v2 (45-Degree Auto-Stop)")



while True:
    # ==========================================
    # TASK 1: THE LIMIT SWITCH OVERRIDE
    # (Moved to the top so it blocks new commands if physically stuck)
    # ==========================================
    try:
        limit_state = mcp.read_all()
        # If the specific limit switch is pressed (pulled to 0V)
        if (limit_state & (1 << LIMIT_SWITCH_PIN)) == 0: 
            
            any_motor_stopped = False
            for m in range(NUM_MOTORS):
                if move_in_progress[m]:
                    feathers.state_hold(m)
                    move_in_progress[m] = False
                    any_motor_stopped = True
            
            if any_motor_stopped:
                print("[SAFETY] Limit Switch Triggered! All motors HALTED.")
            
            # Restart the loop immediately. Do not process USB or I2C.
            continue 
            
    except Exception as e:
        print(f"[ERROR] Port Expander read failed: {e}")


    # ==========================================
    # TASK 2: KEYBOARD COMMANDS
    # ==========================================
    if usb_poll.poll(0):
        command = sys.stdin.readline().strip().upper()

        if command.startswith("START"):
            parts = command.split(',')
            if len(parts) == 3:
                motor_id = int(parts[1])
                direction = int(parts[2])

                if 0 <= motor_id < NUM_MOTORS:
                    if not move_in_progress[motor_id]:
                        fwd = bool(direction)
                        
                        # Tell Slave to RESET this specific encoder to 0 via I2C Write
                        # Command format: [0x01 (Reset Cmd), Motor_ID]
                        try:
                            i2c_slave.writeto(SLAVE_ADDR, bytearray([0x01, motor_id]))
                        except:
                            print(f"[ERROR] Failed to reset Slave encoder {motor_id}")

                        # Start the physical motor
                        feathers.state_move(motor_id, forward=fwd)
                        move_in_progress[motor_id] = True
                        print(f"[SYSTEM] Motor {motor_id} Started.")

        elif command.startswith("STOP"):
            parts = command.split(',')
            if len(parts) == 2:
                motor_id = int(parts[1])
                if 0 <= motor_id < NUM_MOTORS and move_in_progress[motor_id]:
                    feathers.state_hold(motor_id)
                    move_in_progress[motor_id] = False
                    print(f"[SYSTEM] Motor {motor_id} stopped by user.")


    # ==========================================
    # TASK 3: THE 45-DEGREE (125 TICK) CHECK
    # ==========================================
    # Only bother asking the Slave if at least one motor is actually moving
    if any(move_in_progress):
        try:
            # Ask Slave for exactly 5 bytes: 
            # [Byte 0: Threshold Flags] [Bytes 1-4: Raw ticks for debugging]
            data = i2c_slave.readfrom(SLAVE_ADDR, 5)
            
            threshold_flags = data[0] # Bit 0 = Motor 0, Bit 1 = Motor 1, etc.
            
            # If any flag is thrown (value is greater than 0)
            if threshold_flags > 0:
                for m in range(NUM_MOTORS):
                    # Check if the specific bit for this motor is a '1'
                    if (threshold_flags & (1 << m)) and move_in_progress[m]:
                        feathers.state_hold(m)
                        move_in_progress[m] = False
                        print(f"[AUTO-STOP] Motor {m} reached 125 ticks (45 degrees)!")

        except Exception as e:
            print(f"[ERROR] I2C Slave read failed: {e}")

    # Very short sleep to keep the bus from saturating
    time.sleep_ms(2)