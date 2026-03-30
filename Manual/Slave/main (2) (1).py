import machine
import time
from encoder_pio import PIOEncoder 
from i2c_responder import I2CResponder 

# ==========================================
# 1. HARDWARE SETUP
# ==========================================
# Initialize 4 PIO State Machines (0, 1, 2, 3) for the 4 motors
encoders = [
    PIOEncoder(0, machine.Pin(0), machine.Pin(1)), # Motor 0
    PIOEncoder(1, machine.Pin(2), machine.Pin(3)), # Motor 1
    PIOEncoder(2, machine.Pin(8), machine.Pin(9)), # Motor 2
    PIOEncoder(3, machine.Pin(10), machine.Pin(11))  # Motor 3
]

NUM_MOTORS = 4

# We use "Software Zeroing" so we don't have to restart the hardware PIO
# Real Ticks = Hardware Ticks - Offset
encoder_offsets = [0] * NUM_MOTORS

# Initialize I2C Target on Bus 1 (Address 0x50)
i2c_target = I2CResponder(i2c_device_id=0, sda_gpio=4, scl_gpio=5, responder_address=0x50)

print("SLAVE READY. Monitoring 4 encoders and listening on 0x50...")


# ==========================================
# 2. HELPER FUNCTION
# ==========================================
def get_relative_ticks(motor_id):
    """Calculates how far the motor has moved since the last reset command."""
    raw_ticks = encoders[motor_id].read()
    return raw_ticks - encoder_offsets[motor_id]


# ==========================================
# 3. MAIN LOOP
# ==========================================
while True:
    
    # -------------------------------------------------
    # TASK 1: LISTEN FOR RESET COMMANDS (Master -> Slave)
    # -------------------------------------------------
    if i2c_target.write_data_is_available():
        
        # Pull the incoming bytes out of the hardware buffer
        incoming_data = i2c_target.get_write_data(max_size=2)
        
        if len(incoming_data) == 2:
            command = incoming_data[0]
            motor_id = incoming_data[1]
            
            # If the Master sent the 0x01 Reset command
            if command == 0x01 and 0 <= motor_id < NUM_MOTORS:
                # Capture the current hardware tick count as the new "Zero" line
                encoder_offsets[motor_id] = encoders[motor_id].read()
                print(f"[RESET] Motor {motor_id} zeroed.")


    # -------------------------------------------------
    # TASK 2: SEND THRESHOLD DATA (Slave -> Master)
    # -------------------------------------------------
    if i2c_target.read_is_pending():
        
        threshold_flags = 0
        debug_bytes = []
        
        for m in range(NUM_MOTORS):
            current_ticks = get_relative_ticks(m)
            
            # Check if it hit the 45-degree mark (Forward OR Reverse)
            if abs(current_ticks) >= 125:
                # Flip the specific bit for this motor to a '1'
                threshold_flags |= (1 << m)
                
            # Clamp the raw ticks to 255 maximum so it fits in a single 8-bit byte
            # This is purely so the Master can print debugging numbers
            clamped_ticks = min(abs(current_ticks), 255)
            debug_bytes.append(clamped_ticks)

        # Assemble the 5-byte packet exactly as the Master expects it:
        # [Flags, M0_Ticks, M1_Ticks, M2_Ticks, M3_Ticks]
        data_packet = [threshold_flags] + debug_bytes
        
        # Shove all 5 bytes into the transmit wire
        for byte in data_packet:
            i2c_target.put_read_data(byte)

    # Tight loop, minimal sleep for fastest response time
    time.sleep_ms(1)