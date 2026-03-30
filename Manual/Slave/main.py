import machine
import time
from encoder_pio import PIOEncoder 
from i2c_responder import I2CResponder 

# ==========================================
# 1. HARDWARE SETUP
# ==========================================
NUM_MOTORS = 4

encoders = [
    PIOEncoder(0, machine.Pin(0), machine.Pin(1)), # Motor 0
    PIOEncoder(1, machine.Pin(2), machine.Pin(3)), # Motor 1
    PIOEncoder(2, machine.Pin(8), machine.Pin(9)), # Motor 2
    PIOEncoder(3, machine.Pin(10), machine.Pin(11))  # Motor 3
]

i2c_target = I2CResponder(i2c_device_id=0, sda_gpio=4, scl_gpio=5, responder_address=0x50)

# Array to hold the dynamic target tick limits for each motor
target_ticks = [0] * NUM_MOTORS

print("SLAVE READY. Awaiting dynamic targets...")

# ==========================================
# 2. MAIN LOOP
# ==========================================
while True:
    
    # -------------------------------------------------
    # TASK 1: RECEIVE DYNAMIC TARGETS (Master -> Slave)
    # -------------------------------------------------
    if i2c_target.write_data_is_available():
        
        # Grab up to 4 bytes now
        incoming_data = i2c_target.get_write_data(max_size=4)
        
        if len(incoming_data) == 4:
            command = incoming_data[0]
            motor_id = incoming_data[1]
            
            # Reconstruct the 2-byte angle
            # Extract bytes 2 and 3, convert back to a standard integer
            angle = int.from_bytes(bytes(incoming_data[2:4]), 'little')
            
            if command == 0x01 and 0 <= motor_id < NUM_MOTORS:
                
                encoders[motor_id].read() 
                encoders[motor_id].ticks = 0 
                
                # Calculate the specific tick threshold requested by Master
                target_ticks[motor_id] = int((angle / 360.0) * 1000)
                
                print(f"[RESET] Motor {motor_id} zeroed. Target set to {target_ticks[motor_id]} ticks ({angle} deg)")

    # -------------------------------------------------
    # TASK 2: SEND THRESHOLD DATA (Slave -> Master)
    # -------------------------------------------------
    if i2c_target.read_is_pending():
        
        threshold_flags = 0
        debug_bytes = []
        
        for m in range(NUM_MOTORS):
            current_ticks = encoders[m].read()
            
            # Compare current position against the dynamic target!
            # (Only flag it if the target is > 0 to prevent instant stops on start)
            if target_ticks[m] > 0 and current_ticks >= target_ticks[m]:
                threshold_flags |= (1 << m)
                
            clamped_ticks = min(current_ticks, 255)
            debug_bytes.append(clamped_ticks)

        data_packet = [threshold_flags] + debug_bytes
        
        for byte in data_packet:
            i2c_target.put_read_data(byte)

    time.sleep_ms(1)
